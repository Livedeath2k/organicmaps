package app.organicmaps.sdk.routing;

import androidx.annotation.Keep;

// Mirrored one to one by the C++ enum routing::EcoRecommendation, so each constant is looked up
// by name from JNI (see ToJavaEnum in RoutingJni.cpp). Keep the names identical on both sides.
@Keep
public enum EcoRecommendation
{
  // No relevant limit change, or the current speed already fits.
  KeepSpeed,
  // Take the foot off the gas and coast (internal combustion vehicle).
  Coast,
  // Like Coast, but for an electric vehicle: increase regeneration.
  IncreaseRegen,
  // Begin braking earlier and smoothly.
  BrakeEarlier,
  // A hard deceleration is required now. Advisory only, never automated.
  BrakeNow,
  // When accelerating, pick a lower target so the upcoming limit sequence is met without a later
  // hard braking phase.
  ReduceAccelerationTarget;
}