#include "testing/testing.hpp"

#include "routing/predictive_driving_advisor.hpp"
#include "routing/route.hpp"
#include "routing/segment.hpp"
#include "routing/turns.hpp"
#include "routing/upcoming_speed_limit_provider.hpp"

#include "geometry/point_with_altitude.hpp"
#include "platform/measurement_utils.hpp"
#include "routing_common/maxspeed_conversion.hpp"

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace predictive_driving_tests
{
using namespace routing;
using namespace measurement_utils;
using namespace std;

double KphToMps(double kmph)
{
  return kmph * 1000.0 / 3600.0;
}

SpeedLimitChange MakeChange(double distMeters, double speedKmPH, bool explicitData = true)
{
  SpeedLimitChange c;
  c.m_distFromCurrentMeters = distMeters;
  c.m_speedMps = KphToMps(speedKmPH);
  c.m_hasExplicitData = explicitData;
  return c;
}

AdvisorConfig DefaultConfig()
{
  AdvisorConfig cfg;
  cfg.m_enabled = true;
  cfg.m_horizonMeters = 5000.0;
  cfg.m_vehicleType = AdvisorConfig::VehicleType::Combustion;
  cfg.m_sensitivity = AdvisorConfig::Sensitivity::Normal;
  cfg.m_minSpeedDiffKmPH = 10.0;
  cfg.m_reactionTimeSeconds = 3.0;
  cfg.m_gpsAccuracyMeters = 10.0;
  return cfg;
}

// 1. No upcoming limit change within the horizon.
UNIT_TEST(Advise_NoLimitInHorizon)
{
  AdvisorConfig cfg = DefaultConfig();
  cfg.m_horizonMeters = 500.0;

  vector<SpeedLimitChange> changes = {MakeChange(800.0, 70.0)};  // beyond horizon
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, changes, cfg);

  TEST(!res.m_hasData, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::KeepSpeed, ());
}

// 2. Current speed below the target speed -> keep speed.
UNIT_TEST(Advise_CurrentBelowTarget)
{
  AdvisorConfig cfg = DefaultConfig();
  vector<SpeedLimitChange> changes = {MakeChange(800.0, 120.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::KeepSpeed, ());
}

// 3. Current speed equal to target speed -> keep speed.
UNIT_TEST(Advise_CurrentEqualsTarget)
{
  AdvisorConfig cfg = DefaultConfig();
  vector<SpeedLimitChange> changes = {MakeChange(800.0, 100.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::KeepSpeed, ());
}

// 4. Gentle deceleration (90 -> 70 over 1500 m) -> Coast (combustion).
UNIT_TEST(Advise_GentleDecel_Coast)
{
  AdvisorConfig cfg = DefaultConfig();
  vector<SpeedLimitChange> changes = {MakeChange(1500.0, 70.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(90.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST_GREATER(res.m_requiredDecelMps2, 0.0, ());
  TEST_LESS(res.m_requiredDecelMps2, 0.25, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::Coast, ());
}

// 5. Medium deceleration (100 -> 70 over 800 m) -> BrakeEarlier.
UNIT_TEST(Advise_MediumDecel_BrakeEarlier)
{
  AdvisorConfig cfg = DefaultConfig();
  vector<SpeedLimitChange> changes = {MakeChange(800.0, 70.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST_GREATER_OR_EQUAL(res.m_requiredDecelMps2, 0.25, ());
  TEST_LESS(res.m_requiredDecelMps2, 0.45, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::BrakeEarlier, ());
}

// 6. High deceleration (100 -> 50 over 400 m) -> BrakeNow.
UNIT_TEST(Advise_HighDecel_BrakeNow)
{
  AdvisorConfig cfg = DefaultConfig();
  vector<SpeedLimitChange> changes = {MakeChange(400.0, 50.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST_GREATER_OR_EQUAL(res.m_requiredDecelMps2, 0.45, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::BrakeNow, ());
}

// 7. Several limits close together: 100 now, 70 in 800 m, 50 in 1600 m.
//    Treated as a sequence; aim at 50 over 1600 m -> gentle enough for Coast.
UNIT_TEST(Advise_Sequence_AimsAtLowest)
{
  AdvisorConfig cfg = DefaultConfig();
  vector<SpeedLimitChange> changes = {MakeChange(800.0, 70.0), MakeChange(1600.0, 50.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST(res.m_isSequence, ());
  TEST_ALMOST_EQUAL_ABS(res.m_targetSpeedMps, KphToMps(50.0), 1e-6, ());
  // A single smooth deceleration over 1600 m to 50 is gentle.
  TEST_LESS(res.m_requiredDecelMps2, 0.25, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::Coast, ());
}

// 8. Electric vehicle, gentle deceleration -> IncreaseRegen.
UNIT_TEST(Advise_Electric_Gentle_IncreaseRegen)
{
  AdvisorConfig cfg = DefaultConfig();
  cfg.m_vehicleType = AdvisorConfig::VehicleType::Electric;
  vector<SpeedLimitChange> changes = {MakeChange(1500.0, 70.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(90.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::IncreaseRegen, ());
}

// 9. Combustion vehicle, gentle deceleration -> Coast (not IncreaseRegen).
UNIT_TEST(Advise_Combustion_Gentle_Coast)
{
  AdvisorConfig cfg = DefaultConfig();
  cfg.m_vehicleType = AdvisorConfig::VehicleType::Combustion;
  vector<SpeedLimitChange> changes = {MakeChange(1500.0, 70.0)};
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(90.0), 0.0, changes, cfg);

  TEST(res.m_hasData, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::Coast, ());
}

// 10. Missing / invalid data -> no recommendation, no crash.
UNIT_TEST(Advise_NoValidData)
{
  AdvisorConfig cfg = DefaultConfig();

  vector<SpeedLimitChange> changes = {MakeChange(800.0, 70.0, /*explicitData=*/false),
                                      MakeChange(0.0, 50.0),    // distance <= 0 -> ignored
                                      MakeChange(1600.0, -1.0)};  // negative speed -> ignored
  auto res = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, changes, cfg);
  TEST(!res.m_hasData, ());
  TEST_EQUAL(res.m_recommendation, EcoRecommendation::KeepSpeed, ());

  // Disabled feature: even with valid data, nothing is reported.
  cfg.m_enabled = false;
  vector<SpeedLimitChange> valid = {MakeChange(800.0, 70.0)};
  auto res2 = PredictiveDrivingAdvisor::Advise(KphToMps(100.0), 0.0, valid, cfg);
  TEST(!res2.m_hasData, ());
}

// RequiredDecel basic sanity.
UNIT_TEST(RequiredDecel_Basic)
{
  TEST_EQUAL(PredictiveDrivingAdvisor::RequiredDecel(10.0, 10.0, 100.0), 0.0, ());
  TEST_EQUAL(PredictiveDrivingAdvisor::RequiredDecel(10.0, 20.0, 100.0), 0.0, ());
  TEST_EQUAL(PredictiveDrivingAdvisor::RequiredDecel(10.0, 0.0, 50.0), 1.0, ());  // (100-0)/(2*50)
  TEST_EQUAL(PredictiveDrivingAdvisor::RequiredDecel(20.0, 10.0, 0.0), 0.0, ());
}

// ---------------------------------------------------------------------------
// UpcomingSpeedLimitProvider tests (data access on a real Route).
// ---------------------------------------------------------------------------

// Build a route from (distFromBegin, limitKmph) pairs. A limitKmph of 0 means
// "no explicit maxspeed" on that segment.
shared_ptr<Route> MakeRoute(vector<pair<double, int>> const & distAndLimit)
{
  auto route = make_shared<Route>();

  vector<RouteSegment> segments;
  segments.reserve(distAndLimit.size());
  for (auto const & [dist, limit] : distAndLimit)
  {
    Segment seg(0, 0, 0, true);
    turns::TurnItem turn;
    geometry::PointWithAltitude junction(m2::PointD(0.0, 0.0), 0);
    RouteSegment::RoadNameInfo name;
    RouteSegment rs(seg, turn, junction, name);
    rs.SetDistancesAndTime(dist, 0.0, 0.0);

    if (limit > 0)
      rs.SetSpeedLimit(Maxspeed(Units::Metric, static_cast<MaxspeedType>(limit), kInvalidSpeed));
    else
      rs.SetSpeedLimit(Maxspeed(Units::Metric, kInvalidSpeed, kInvalidSpeed));

    segments.push_back(std::move(rs));
  }
  route->SetRouteSegments(std::move(segments));
  return route;
}

// Limits sorted by route distance, nearest first.
UNIT_TEST(Provider_SortedByDistance)
{
  auto route = MakeRoute({{800, 100}, {1600, 70}, {2400, 50}});
  UpcomingSpeedLimitProvider provider;
  provider.SetRoute(route);

  vector<SpeedLimitChange> out;
  provider.Collect(0.0, 5000.0, out);

  // Current segment (0) has limit 100, so changes are 70 @ 800 and 50 @ 1600.
  TEST_EQUAL(out.size(), 2, ());
  TEST(out.empty() || out[0].m_distFromCurrentMeters < out[1].m_distFromCurrentMeters, ());
  TEST_ALMOST_EQUAL_ABS(out[0].m_distFromCurrentMeters, 800.0, 1e-3, ());
  TEST_ALMOST_EQUAL_ABS(out[1].m_distFromCurrentMeters, 1600.0, 1e-3, ());
  TEST_ALMOST_EQUAL_ABS(out[0].m_speedMps, KphToMps(70.0), 1e-6, ());
  TEST_ALMOST_EQUAL_ABS(out[1].m_speedMps, KphToMps(50.0), 1e-6, ());
}

// Duplicate consecutive limits are filtered.
UNIT_TEST(Provider_FiltersDuplicates)
{
  auto route = MakeRoute({{800, 100}, {1600, 100}, {2400, 50}});
  UpcomingSpeedLimitProvider provider;
  provider.SetRoute(route);

  vector<SpeedLimitChange> out;
  provider.Collect(0.0, 5000.0, out);

  TEST_EQUAL(out.size(), 1, ());
  TEST_ALMOST_EQUAL_ABS(out[0].m_speedMps, KphToMps(50.0), 1e-6, ());
  TEST_ALMOST_EQUAL_ABS(out[0].m_distFromCurrentMeters, 1600.0, 1e-3, ());
}

// Only limits within the horizon are returned.
UNIT_TEST(Provider_RespectsHorizon)
{
  auto route = MakeRoute({{800, 100}, {1600, 70}, {2400, 50}});
  UpcomingSpeedLimitProvider provider;
  provider.SetRoute(route);

  vector<SpeedLimitChange> out;
  provider.Collect(0.0, 900.0, out);

  TEST_EQUAL(out.size(), 1, ());
  TEST_ALMOST_EQUAL_ABS(out[0].m_distFromCurrentMeters, 800.0, 1e-3, ());
}

// Segments without an explicit maxspeed are never surfaced as a limit.
UNIT_TEST(Provider_SkipsNoLimitSegments)
{
  auto route = MakeRoute({{800, 100}, {1600, 0}, {2400, 50}});
  UpcomingSpeedLimitProvider provider;
  provider.SetRoute(route);

  vector<SpeedLimitChange> out;
  provider.Collect(0.0, 5000.0, out);

  // The "no limit" segment is skipped; the 50 limit still shows up.
  bool saw50 = false;
  for (auto const & c : out)
    if (AlmostEqualAbs(c.m_speedMps, KphToMps(50.0), 1e-6))
      saw50 = true;
  TEST(saw50, ());
  for (auto const & c : out)
    TEST(c.m_hasExplicitData, ());
}

// No data at all -> empty result.
UNIT_TEST(Provider_NoData)
{
  auto route = MakeRoute({{800, 0}, {1600, 0}, {2400, 0}});
  UpcomingSpeedLimitProvider provider;
  provider.SetRoute(route);

  vector<SpeedLimitChange> out;
  provider.Collect(0.0, 5000.0, out);
  TEST(out.empty(), ());
}

// Distance is measured from the current position, not the route start.
UNIT_TEST(Provider_MeasuresFromCurrentPosition)
{
  auto route = MakeRoute({{800, 100}, {1600, 70}, {2400, 50}});
  UpcomingSpeedLimitProvider provider;
  provider.SetRoute(route);

  vector<SpeedLimitChange> out;
  provider.Collect(300.0, 5000.0, out);  // inside the first segment

  TEST_EQUAL(out.size(), 2, ());
  TEST_ALMOST_EQUAL_ABS(out[0].m_distFromCurrentMeters, 500.0, 1e-3, ());   // 800 - 300
  TEST_ALMOST_EQUAL_ABS(out[1].m_distFromCurrentMeters, 1300.0, 1e-3, ());  // 1600 - 300
}

}  // namespace predictive_driving_tests