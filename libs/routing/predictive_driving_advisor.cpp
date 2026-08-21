#include "routing/predictive_driving_advisor.hpp"

#include "base/math.hpp"

namespace
{

double constexpr kGravitationalAccelerationMps2 = 9.80655;

// "Normal" sensitivity deceleration thresholds, in m/s^2.
// These are the defaults from the product spec; sensitivity scales them.
double constexpr kCoastThresholdMps2 = 0.08;
double constexpr kBrakeEarlierThresholdMps2 = 0.25;
double constexpr kBrakeNowThresholdMps2 = 0.45;

// When two consecutive limits are closer than this (in meters), they form a
// "sequence" and the driver should not re-accelerate between them. Chosen so
// that a realistic run of signs (e.g. 70 in 800 m, then 50 in 1600 m) is
// treated as one sequence and covered by a single smooth deceleration.
double constexpr kSequenceGapMeters = 1000.0;

// Scale factor for the deceleration thresholds, per sensitivity.
// A higher factor means the same physical deceleration is judged "softer", so
// advice comes later (more permissive). A lower factor means advice comes
// earlier (more conservative).
double SensitivityThresholdScale(routing::AdvisorConfig::Sensitivity sensitivity)
{
  switch (sensitivity)
  {
  case routing::AdvisorConfig::Sensitivity::Soft:
    return 0.8;   // advice earlier
  case routing::AdvisorConfig::Sensitivity::Aggressive:
    return 1.25;  // advice later
  case routing::AdvisorConfig::Sensitivity::Normal:
    return 1.0;
  }
  return 1.0;
}

double MpsToKmph(double mps)
{
  return mps * 3.6;
}

}  // namespace

namespace routing
{

std::string EcoRecommendationToString(EcoRecommendation recommendation)
{
  switch (recommendation)
  {
  case EcoRecommendation::KeepSpeed:
    return "keep_speed";
  case EcoRecommendation::Coast:
    return "coast";
  case EcoRecommendation::IncreaseRegen:
    return "increase_regen";
  case EcoRecommendation::BrakeEarlier:
    return "brake_earlier";
  case EcoRecommendation::BrakeNow:
    return "brake_now";
  case EcoRecommendation::ReduceAccelerationTarget:
    return "reduce_acceleration_target";
  }
  return "keep_speed";
}

double PredictiveDrivingAdvisor::RequiredDecel(double vFromMps, double vToMps, double distanceMeters)
{
  if (distanceMeters <= 0.0 || vFromMps <= vToMps)
    return 0.0;

  // Constant-deceleration model: a = (v0^2 - v1^2) / (2 d), in SI units.
  return (vFromMps * vFromMps - vToMps * vToMps) / (2.0 * distanceMeters);
}

PredictionResult PredictiveDrivingAdvisor::Advise(double vCurrentMps, double currentAltitudeMeters,
                                                  std::vector<SpeedLimitChange> const & changes,
                                                  AdvisorConfig const & cfg)
{
  PredictionResult result;

  if (!cfg.m_enabled)
    return result;  // m_hasData == false, KeepSpeed.

  if (vCurrentMps < 0.0)
    vCurrentMps = 0.0;

  // Keep only explicit, in-horizon, positive-speed limits.
  std::vector<SpeedLimitChange> relevant;
  relevant.reserve(changes.size());
  for (auto const & c : changes)
  {
    if (!c.m_hasExplicitData)
      continue;
    if (c.m_distFromCurrentMeters <= 0.0)
      continue;
    if (c.m_distFromCurrentMeters > cfg.m_horizonMeters)
      continue;
    if (c.m_speedMps < 0.0)
      continue;
    relevant.push_back(c);
  }

  // No reliable data -> stay silent.
  if (relevant.empty())
    return result;  // m_hasData == false, KeepSpeed.

  result.m_hasData = true;
  result.m_upcoming = std::move(relevant);

  // Determine whether we are in a "sequence": several limits close together.
  bool isSequence = false;
  for (size_t i = 0; i + 1 < result.m_upcoming.size(); ++i)
  {
    double const gap = result.m_upcoming[i + 1].m_distFromCurrentMeters -
                       result.m_upcoming[i].m_distFromCurrentMeters;
    if (gap <= kSequenceGapMeters)
    {
      isSequence = true;
      break;
    }
  }
  result.m_isSequence = isSequence;

  // Find the effective target: the limit we should actually aim for.
  //
  // We walk the upcoming limits in order and pick the first one that is lower
  // than the current speed. If limits are in a sequence, aiming only at the
  // nearest one can force re-acceleration followed by another hard braking; in
  // that case we instead aim at the lowest limit in the sequence so a single
  // smooth deceleration covers the whole stretch.
  auto const nearest = result.m_upcoming.front();
  double targetSpeedMps = 0.0;
  double targetDistMeters = 0.0;

  if (nearest.m_speedMps >= vCurrentMps)
  {
    // We are already at/below the next limit. Nothing to brake for.
    // (We still surface the list so the driver sees the upcoming values.)
    result.m_recommendation = EcoRecommendation::KeepSpeed;
    result.m_requiredDecelMps2 = 0.0;
    result.m_targetSpeedMps = nearest.m_speedMps;
    return result;
  }

  if (isSequence)
  {
    // In a sequence, aim at the lowest speed in the sequence and at the
    // distance where that lowest speed starts (the furthest point of the run of
    // closely-following limits).
    double lowestSpeed = nearest.m_speedMps;
    double lowestDist = nearest.m_distFromCurrentMeters;
    for (size_t i = 1; i < result.m_upcoming.size(); ++i)
    {
      double const gap = result.m_upcoming[i].m_distFromCurrentMeters -
                         result.m_upcoming[i - 1].m_distFromCurrentMeters;
      if (gap > kSequenceGapMeters)
        break;  // sequence ended
      if (result.m_upcoming[i].m_speedMps < lowestSpeed)
      {
        lowestSpeed = result.m_upcoming[i].m_speedMps;
        lowestDist = result.m_upcoming[i].m_distFromCurrentMeters;
      }
    }
    targetSpeedMps = lowestSpeed;
    targetDistMeters = lowestDist;
  }
  else
  {
    targetSpeedMps = nearest.m_speedMps;
    targetDistMeters = nearest.m_distFromCurrentMeters;
  }

  result.m_targetSpeedMps = targetSpeedMps;

  // The speed difference must be relevant, otherwise keep speed.
  double const speedDiffKmPH = MpsToKmph(vCurrentMps - targetSpeedMps);
  if (speedDiffKmPH < cfg.m_minSpeedDiffKmPH)
  {
    result.m_recommendation = EcoRecommendation::KeepSpeed;
    result.m_requiredDecelMps2 = 0.0;
    return result;
  }

  // Effective distance: shorten by the distance covered during the driver's
  // reaction time and by the assumed GPS error, to be conservative. Never
  // below a small epsilon to avoid division blow-ups.
  double const reactionDistMeters = vCurrentMps * cfg.m_reactionTimeSeconds;
  double const effectiveDist = targetDistMeters - reactionDistMeters - cfg.m_gpsAccuracyMeters;
  double const d = std::max(effectiveDist, 1.0);

  double decel = RequiredDecel(vCurrentMps, targetSpeedMps, d);
  if (decel < 0.0)
    decel = 0.0;

  // Grade (slope) correction: on a downhill the car accelerates for free, so
  // coasting/regen must start earlier. On an uphill gravity already helps slow
  // the car down. We correct the required deceleration by the average grade:
  //     a_eff = a + g * (h_start - h_target) / d
  // A downhill (h_target < h_start) increases a_eff (coast earlier); an uphill
  // decreases it. Only applied when altitudes are plausibly known (> 0).
  double const targetAltitude = nearest.m_altitudeMeters;
  bool const altitudeKnown = currentAltitudeMeters > 0.0 && targetAltitude > 0.0;
  if (altitudeKnown)
  {
    double const dh = currentAltitudeMeters - targetAltitude;
    decel = decel + kGravitationalAccelerationMps2 * (dh / d);
    if (decel < 0.0)
      decel = 0.0;
  }

  result.m_requiredDecelMps2 = decel;

  double const scale = SensitivityThresholdScale(cfg.m_sensitivity);
  double const coastT = kCoastThresholdMps2 * scale;
  double const brakeEarlierT = kBrakeEarlierThresholdMps2 * scale;
  double const brakeNowT = kBrakeNowThresholdMps2 * scale;

  if (decel < coastT)
  {
    result.m_recommendation = EcoRecommendation::KeepSpeed;
  }
  else if (decel < brakeEarlierT)
  {
    result.m_recommendation = (cfg.m_vehicleType == AdvisorConfig::VehicleType::Electric)
                                  ? EcoRecommendation::IncreaseRegen
                                  : EcoRecommendation::Coast;
  }
  else if (decel < brakeNowT)
  {
    result.m_recommendation = EcoRecommendation::BrakeEarlier;
  }
  else
  {
    result.m_recommendation = EcoRecommendation::BrakeNow;
  }

  return result;
}

}  // namespace routing