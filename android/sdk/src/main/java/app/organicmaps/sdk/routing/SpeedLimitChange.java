package app.organicmaps.sdk.routing;

import androidx.annotation.Keep;

// A single upcoming speed limit change along the route, as reported by the C++ layer.
// Speeds are in km/h, distances in meters (converted from SI units on the JNI side).
@Keep
public final class SpeedLimitChange
{
  // Distance from the current position to where this new limit starts, in meters.
  public final double distanceMeters;
  // The new speed limit in km/h.
  public final int speedKmph;

  private SpeedLimitChange(double distanceMeters, int speedKmph)
  {
    this.distanceMeters = distanceMeters;
    this.speedKmph = speedKmph;
  }
}