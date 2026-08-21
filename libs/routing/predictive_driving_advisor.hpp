#pragma once

// Pure, UI-independent predictive speed limit / eco driving logic.
//
// This module must stay free of Qt / Android / map-data dependencies so that it
// can be unit tested in isolation. It only consumes plain numbers (speeds in m/s,
// distances in meters, altitudes in meters) and produces a plain recommendation.
//
// The core physics is a constant-deceleration estimate:
//     a = (v_current^2 - v_target^2) / (2 * d)
// computed internally in SI units (m/s, meters, m/s^2).

#include <cstddef>
#include <string>
#include <vector>

namespace routing
{

/// A single upcoming speed limit change along the route.
/// Distances and speeds are expressed in SI units (meters, m/s).
struct SpeedLimitChange
{
  /// Distance from the current position to the start of the new limit. Always > 0.
  double m_distFromCurrentMeters = 0.0;

  /// The new speed limit in meters per second.
  double m_speedMps = 0.0;

  /// Altitude (in meters) at the point where the new limit starts. Used for the
  /// grade (slope) correction. |kInvalidAltitudeMeters| when unknown.
  double m_altitudeMeters = 0.0;

  /// True only when this limit comes from an explicit OSM `maxspeed` tag.
  /// Default/estimated speeds must NOT be surfaced to the user as a real sign.
  bool m_hasExplicitData = true;
};

/// The eco driving recommendation produced by |PredictiveDrivingAdvisor|.
enum class EcoRecommendation
{
  /// No relevant limit change, or the current speed already fits.
  KeepSpeed = 0,

  /// Take the foot off the gas and coast (internal combustion vehicle).
  Coast,

  /// Like Coast, but for an electric vehicle: increase regeneration.
  IncreaseRegen,

  /// Begin braking earlier and smoothly.
  BrakeEarlier,

  /// A hard deceleration is required now. Still advisory only, never automated.
  BrakeNow,

  /// When accelerating, pick a lower target so the upcoming limit sequence is
  /// met without a later hard braking phase.
  ReduceAccelerationTarget
};

/// A short, human-readable summary of the recommendation. Kept here (in the
/// pure logic layer) so the UI does not re-implement the decision.
std::string EcoRecommendationToString(EcoRecommendation recommendation);

/// Tunable parameters of the advisor. All distances/speeds in SI units.
struct AdvisorConfig
{
  enum class VehicleType
  {
    Combustion = 0,
    Electric = 1
  };

  enum class Sensitivity
  {
    Soft = 0,      // Earlier, gentler advice.
    Normal = 1,
    Aggressive = 2 // Later, more permissive advice.
  };

  bool m_enabled = true;

  /// Look-ahead horizon in meters (e.g. 1000 / 2000 / 5000).
  double m_horizonMeters = 2000.0;

  VehicleType m_vehicleType = VehicleType::Combustion;
  Sensitivity m_sensitivity = Sensitivity::Normal;

  /// Minimum speed difference (km/h) required to produce any non-KeepSpeed advice.
  /// Avoids reacting to tiny differences / noise.
  double m_minSpeedDiffKmPH = 10.0;

  /// Assumed human reaction time in seconds. The effective braking distance is
  /// shortened by |v_current * m_reactionTimeSeconds| to be conservative.
  double m_reactionTimeSeconds = 3.0;

  /// Assumed GPS horizontal error in meters, subtracted from the effective
  /// braking distance to be conservative.
  double m_gpsAccuracyMeters = 10.0;
};

/// The full result of an advice computation.
struct PredictionResult
{
  /// False when there is no reliable data (no explicit limit within the
  /// horizon). In that case the UI must stay silent and must not show numbers.
  bool m_hasData = false;

  EcoRecommendation m_recommendation = EcoRecommendation::KeepSpeed;

  /// The average deceleration required to reach the effective target, in m/s^2.
  /// Non-negative. 0.0 when not applicable.
  double m_requiredDecelMps2 = 0.0;

  /// Upcoming limit changes, sorted by route distance (nearest first). Used for
  /// the UI list. May be empty.
  std::vector<SpeedLimitChange> m_upcoming;

  /// True when several limits follow each other within a short distance, so the
  /// driver should not re-accelerate between them. Drives the "then 50 in 1.6 km"
  /// part of the message.
  bool m_isSequence = false;

  /// The effective target speed (m/s) the recommendation is computed against, or
  /// 0.0 when none. Useful for tests and for the UI.
  double m_targetSpeedMps = 0.0;
};

/// Pure, stateless advice computation. No globals, no I/O, no threads.
class PredictiveDrivingAdvisor
{
public:
  /// Compute an eco driving recommendation.
  ///
  /// @param vCurrentMps          Current vehicle speed in m/s (>= 0).
  /// @param currentAltitudeMeters Current altitude in meters, or
  ///                              |kInvalidAltitudeMeters| when unknown.
  /// @param changes              Upcoming limit changes (nearest first). May be
  ///                             empty.
  /// @param cfg                  Tunable parameters.
  ///
  /// The result never throws and never dereferences invalid input. When
  /// |cfg.m_enabled| is false or there is no explicit data, |m_hasData| is false
  /// and the recommendation is |KeepSpeed|.
  static PredictionResult Advise(double vCurrentMps, double currentAltitudeMeters,
                                 std::vector<SpeedLimitChange> const & changes, AdvisorConfig const & cfg);

  /// A sentinel for "altitude unknown". Mirrors geometry::kDefaultAltitudeMeters
  /// but kept local to avoid a hard dependency for tests.
  static double constexpr kInvalidAltitudeMeters = 0.0;

  /// Compute the required average deceleration (m/s^2) to go from |vFromMps| to
  /// |vToMps| over |distanceMeters|, using the constant-deceleration model.
  /// Returns 0.0 when |vFromMps| <= |vToMps| or when the distance is non-positive.
  static double RequiredDecel(double vFromMps, double vToMps, double distanceMeters);
};

}  // namespace routing