#pragma once

// Collects the upcoming speed limit changes along the active route.
//
// This is the "data access" layer of the Predictive Speed Limit / Eco Driving
// feature. It reads explicit OSM `maxspeed` values stored on the route
// segments, orders the changes by route distance and filters out duplicates
// and non-explicit values. It does NOT decide anything about the driver:
// that is the job of |PredictiveDrivingAdvisor|.

#include "routing/predictive_driving_advisor.hpp"  // SpeedLimitChange

#include <cstddef>
#include <memory>
#include <vector>

namespace routing
{
class Route;

/// Supplies the upcoming speed limit changes for the active route.
///
/// The provider holds a weak reference to the route (owned by the routing
/// session) and can be asked, for the current position, which limits change
/// within a given look-ahead horizon. It is intentionally stateless apart
/// from a small cache of the current segment index, which only ever grows
/// while driving.
class UpcomingSpeedLimitProvider
{
public:
  /// Set (or clear, with an empty |route|) the route to inspect.
  void SetRoute(std::weak_ptr<Route> route);

  /// Collect the upcoming speed limit changes.
  ///
  /// @param passedDistanceMeters  Distance travelled along the route so far (meters).
  /// @param horizonMeters         Look-ahead horizon (meters). Must be > 0.
  /// @param out                   Filled with the changes, nearest first. Always
  ///                              cleared before use. Only explicit, numeric OSM
  ///                              limits are included; duplicates and non-numeric
  ///                              values are dropped.
  void Collect(double passedDistanceMeters, double horizonMeters,
               std::vector<SpeedLimitChange> & out) const;

private:
  std::weak_ptr<Route> m_route;
  mutable size_t m_cachedSegmentIndex = 0;
};

}  // namespace routing