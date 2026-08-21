package app.organicmaps.sdk.routing;

import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.routing.roadshield.RoadShieldInfo;
import app.organicmaps.sdk.util.Distance;

// Called from JNI.
@Keep
@SuppressWarnings("unused")
public final class RoutingInfo
{
  // Target (end point of route).
  public final Distance distToTarget;
  // Next turn.
  public final Distance distToTurn;

  public final int totalTimeInSeconds;
  // Current street name.
  public final String currentStreet;
  // The next street name.
  public final String nextStreet;
  @Nullable
  public final RoadShieldInfo nextStreetRoadShields;
  // The next next street name.
  public final String nextNextStreet;
  @Nullable
  public final RoadShieldInfo nextNextStreetRoadShields;
  public final double completionPercent;
  // For vehicle routing.
  @NonNull
  public final CarDirection carDirection;
  @NonNull
  public final CarDirection nextCarDirection;
  public final int exitNum;
  @Nullable
  public final LaneInfo[] lanes;
  // For pedestrian routing.
  @NonNull
  public final PedestrianDirection pedestrianDirection;
  // Current speed limit in meters per second.
  // If no info about speed limit then speedLimitMps < 0.
  public final double speedLimitMps;
  private final boolean speedCamLimitExceeded;
  private final boolean shouldPlayWarningSignal;

  /// @name Predictive Speed Limit / Eco Driving (advisory only, never automated).
  //@{
  // True when there is reliable explicit OSM data to show. When false the UI must stay silent and
  // must not display any numbers or recommendation.
  public final boolean ecoHasData;
  // The eco driving recommendation.
  @NonNull
  public final EcoRecommendation ecoRecommendation;
  // Upcoming speed limit changes, nearest first. May be null when there is no data.
  @Nullable
  public final SpeedLimitChange[] upcomingSpeedLimits;
  // True when several limits follow each other within a short distance.
  public final boolean ecoIsSequence;
  //@}

  private RoutingInfo(Distance distToTarget, Distance distToTurn, String currentStreet, String nextStreet,
                      @Nullable RoadShieldInfo nextStreetRoadShields, String nextNextStreet,
                      @Nullable RoadShieldInfo nextNextStreetRoadShields, double completionPercent,
                      @NonNull CarDirection carTurnDirection, @NonNull CarDirection carNextTurnDirection,
                      @NonNull PedestrianDirection pedestrianDirection, int exitNum, int totalTime,
                      @Nullable LaneInfo[] lanes, double speedLimitMps, boolean speedLimitExceeded,
                      boolean shouldPlayWarningSignal, boolean ecoHasData,
                      @NonNull EcoRecommendation ecoRecommendation,
                      @Nullable SpeedLimitChange[] upcomingSpeedLimits, boolean ecoIsSequence)
  {
    this.distToTarget = distToTarget;
    this.distToTurn = distToTurn;
    this.currentStreet = currentStreet;
    this.nextStreet = nextStreet;
    this.nextStreetRoadShields = nextStreetRoadShields;
    this.nextNextStreet = nextNextStreet;
    this.nextNextStreetRoadShields = nextNextStreetRoadShields;
    this.totalTimeInSeconds = totalTime;
    this.completionPercent = completionPercent;
    this.carDirection = carTurnDirection;
    this.nextCarDirection = carNextTurnDirection;
    this.lanes = lanes;
    this.exitNum = exitNum;
    this.pedestrianDirection = pedestrianDirection;
    this.speedLimitMps = speedLimitMps;
    this.speedCamLimitExceeded = speedLimitExceeded;
    this.shouldPlayWarningSignal = shouldPlayWarningSignal;
    this.ecoHasData = ecoHasData;
    this.ecoRecommendation = ecoRecommendation;
    this.upcomingSpeedLimits = upcomingSpeedLimits;
    this.ecoIsSequence = ecoIsSequence;
  }

  public boolean isSpeedCamLimitExceeded()
  {
    return speedCamLimitExceeded;
  }

  public boolean shouldPlayWarningSignal()
  {
    return shouldPlayWarningSignal;
  }

  public boolean hasNextNextTurn()
  {
    return nextNextStreet != null && !nextNextStreet.isEmpty() && nextCarDirection != CarDirection.NoTurn
 && nextCarDirection != CarDirection.GoStraight;
  }
}
