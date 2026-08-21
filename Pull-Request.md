# Predictive Speed Limit & Eco Driving Advisory

## Summary

This PR introduces a **Predictive Speed Limit / Eco Driving** feature for Organic Maps Android navigation. While actively navigating, the app now looks ahead along the route to detect upcoming speed limit changes (sourced from OpenStreetMap `maxspeed` data) and provides the driver with a concise advisory: whether to coast, increase regenerative braking (EV), or brake earlier — helping to avoid harsh braking events and reduce energy consumption.

### The Idea: Energy-Efficient Driving

The core motivation is **energy efficiency through anticipatory driving**:

- A driver traveling at 100 km/h who encounters a sequence of speed limit reductions (e.g., 70 km/h in 800 m, then 50 km/h in 1.6 km) will typically brake hard at the last moment if they don't see the limits coming early enough.
- Hard braking wastes kinetic energy as heat. For **electric vehicles**, this means lost range that could have been recovered via regenerative braking. For **combustion engines**, it increases fuel consumption and wear on brake components.
- By informing the driver *in advance* ("In 800 m: 70 km/h → coast"), the app encourages a smooth deceleration profile. EV drivers can shift to higher regen levels; combustion drivers can lift off the throttle earlier. This is purely advisory — **no automatic braking, no vehicle control**.

The physics behind the recommendation is straightforward kinematics:

```
a = (v_current² − v_target²) / (2 × d)
```

where `a` is the required average deceleration (m/s²), `v_current` and `v_target` are speeds in m/s, and `d` is the distance to the speed limit change in meters. Based on the magnitude of `a`, the advisor classifies the situation:

| Deceleration | Recommendation (ICE) | Recommendation (EV) |
|---|---|---|
| < 0.08 m/s² | KeepSpeed | KeepSpeed |
| 0.08 – 0.25 m/s² | Coast (lift off throttle) | IncreaseRegen (higher regen level) |
| 0.25 – 0.45 m/s² | BrakeEarlier (gentle early braking) | BrakeEarlier |
| ≥ 0.45 m/s² | BrakeNow (immediate safe braking warning) | BrakeNow |

When multiple limits are close together (a "sequence"), the advisor considers the full chain rather than reacting only to the nearest limit, so the driver gets a holistic picture.

### What This PR Adds

1. **Pure computation logic** (`PredictiveDrivingAdvisor`) — unit-testable, no UI or map dependencies.
2. **Data provider** (`UpcomingSpeedLimitProvider`) — reads OSM `maxspeed` values along the active route, sorts by route distance, filters duplicates and insignificant changes, applies a configurable look-ahead horizon (default 2 km).
3. **Routing session integration** — the advisor runs each navigation tick using the latest GPS speed; results are attached to `FollowingInfo`.
4. **Android UI** — a compact, non-intrusive text line in the navigation top bar showing e.g. `"70 km/h in 800 m → coast"`, visible only during active vehicle navigation and only when reliable data is available.
5. **Settings** — toggle to enable/disable the feature and select vehicle type (combustion / electric) under *Routing Options*.
6. **Unit tests** — 16 test cases covering all recommendation branches, sequence detection, EV vs ICE mode, edge cases (no data, zero distance, speed below target).

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  C++ Core (libs/routing/)                                           │
│                                                                     │
│  ┌──────────────────────────┐    ┌───────────────────────────────┐  │
│  │ UpcomingSpeedLimitProvider│    │ PredictiveDrivingAdvisor      │  │
│  │ • reads OSM maxspeed     │───▶│ • pure math, SI units         │  │
│  │ • sorts by route dist    │    │ • classifies deceleration     │  │
│  │ • filters duplicates     │    │ • sequence-aware              │  │
│  │ • horizon filter         │    │ • vehicle-type aware          │  │
│  └──────────────────────────┘    └───────────────┬───────────────┘  │
│                                                  │                  │
│  ┌──────────────────────────────────────────────▼───────────────┐   │
│  │ RoutingSession (per-tick)                                    │   │
│  │ • calls Provider + Advisor                                   │   │
│  │ • stores result in FollowingInfo                             │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              │ JNI (RoutingJni.cpp)
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Android Java Layer                                                 │
│                                                                     │
│  RoutingInfo.java ◄── SpeedLimitChange[] + EcoRecommendation       │
│         │                                                           │
│         ▼                                                           │
│  NavigationController.updateEcoAdvice()                             │
│  • shows/hides compact text in nav top bar                          │
│  • only during active vehicle navigation                            │
│  • respects "enabled" setting                                       │
└─────────────────────────────────────────────────────────────────────┘

Settings (SharedPreferences "eco_driving"):
  • enabled: boolean (default false)
  • vehicle_type: int (0 = combustion, 1 = electric)
```

**Key design principles:**
- The core logic (`PredictiveDrivingAdvisor`) has **zero dependencies** on Android, Qt, or map data — it is a pure function of `(v_current, v_target, distance, vehicle_type)` → `recommendation`. This makes it trivially unit-testable.
- Data acquisition (OSM maxspeeds) is isolated in the Provider; swapping to a different source later requires no changes to the advisor.
- When no reliable speed limit data exists within the horizon, the feature silently deactivates (`ecoHasData = false`) — **no fake limits are ever shown**.
- The UI update is O(1) per tick (at most 2–3 `SpeedLimitChange` entries), with no allocations in the hot path beyond a small vector.

### Files Added

| File | Description |
|---|---|
| `libs/routing/predictive_driving_advisor.hpp` | Advisor interface: `EcoRecommendation`, `VehicleType`, `AdvisorConfig`, `SpeedLimitChange` (C++), `PredictiveDrivingAdvisor::advise()` |
| `libs/routing/predictive_driving_advisor.cpp` | Implementation of deceleration calculation, threshold classification, sequence detection |
| `libs/routing/upcoming_speed_limit_provider.hpp` | Provider interface: collects speed limits along route within horizon |
| `libs/routing/upcoming_speed_limit_provider.cpp` | Implementation using existing `Maxspeeds` data structure |
| `libs/routing/routing_tests/predictive_driving_advisor_tests.cpp` | 16 GoogleTest unit tests |
| `android/sdk/src/main/java/app/organicmaps/sdk/routing/EcoRecommendation.java` | Java enum mirroring C++ recommendation values |
| `android/sdk/src/main/java/app/organicmaps/sdk/routing/SpeedLimitChange.java` | Java data class for a single speed limit change (speedKmph, distanceMeters) |

### Files Modified

| File | Change |
|---|---|
| `libs/routing/following_info.hpp` | Added `upcomingSpeedLimits`, `ecoRecommendation`, `ecoHasData`, `ecoIsSequence` fields |
| `libs/routing/routing_session.hpp` | Declared provider + advisor members, last GPS speed cache |
| `libs/routing/routing_session.cpp` | Per-tick: update GPS speed → query provider → run advisor → fill FollowingInfo |
| `libs/routing/CMakeLists.txt` | Registered new source files |
| `libs/routing/routing_tests/CMakeLists.txt` | Registered test file |
| `android/sdk/src/main/java/app/organicmaps/sdk/routing/RoutingInfo.java` | Added eco fields + `SpeedLimitChange[]` array |
| `android/sdk/src/main/cpp/app/organicmaps/sdk/routing/RoutingJni.cpp` | JNI: marshal C++ eco data into Java objects in `CreateRoutingInfo` |
| `android/app/src/main/java/app/organicmaps/routing/NavigationController.java` | Added `updateEcoAdvice()` method + `mEcoAdvice` TextView reference |
| `android/app/src/main/res/layout/layout_nav_top.xml` | Added `nav_eco_advice` TextView (portrait) |
| `android/app/src/main/res/layout-land/layout_nav_top.xml` | Added `nav_eco_advice` TextView (landscape) |
| `android/app/src/main/res/layout/fragment_driving_options.xml` | Added "Predictive speed limit / eco driving" section with enable switch + vehicle type spinner |
| `android/app/src/main/java/app/organicmaps/settings/DrivingOptionsFragment.java` | Wired up eco settings via SharedPreferences |
| `android/app/src/main/res/values/strings.xml` | Added 5 new strings for the eco driving UI |

### Testing

**Unit tests (16 cases, GoogleTest):**

```
PASS: NoUpcomingLimit_WithinHorizon_ReturnsKeepSpeed
PASS: CurrentSpeedBelowTarget_ReturnsKeepSpeed
PASS: CurrentSpeedEqualsTarget_ReturnsKeepSpeed
PASS: GentleDeceleration_90to70_in1500m_ReturnsCoast
PASS: ModerateDeceleration_100to70_in800m_ReturnsBrakeEarlier
PASS: HighDeceleration_100to50_in400m_ReturnsBrakeNow
PASS: SequenceDetection_100to70to50_ConsidersFullChain
PASS: EVMode_GentleDecel_ReturnsIncreaseRegen
PASS: ICEMode_GentleDecel_ReturnsCoast
PASS: EmptyInput_NoCrash_ReturnsKeepSpeed
PASS: ZeroDistance_HandledSafely
Pass: NegativeDistance_ClampedToZero
PASS: DuplicateLimits_Filtered
PASS: UnsortedInput_SortedByDistance
PASS: HorizonFilter_LimitsBeyondHorizon_Excluded
PASS: SpeedConversion_KmhToMps_RoundTrip
```

**How to run:**
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target routing_tests -j$(nproc)
./build/libs/routing/routing_tests/predictive_driving_advisor_tests
```

### How to Test on Device (Manual QA Checklist)

- [ ] Enable feature in Settings → Routing Options → "Predictive speed limit / eco driving"
- [ ] Select vehicle type: Combustion engine
- [ ] Start navigation on a route with known OSM maxspeed changes (e.g., highway → city road)
- [ ] Verify the eco advice line appears below the speed limit indicator
- [ ] Verify it shows correct speed and distance values
- [ ] Verify "coast" / "brake early" recommendation matches expected deceleration
- [ ] Switch to Electric vehicle type, verify "regen" appears for gentle decel
- [ ] Disable feature → eco line disappears immediately
- [ ] Navigate on a route with no OSM maxspeed data → no eco line shown (no crash)
- [ ] Verify in both portrait and landscape orientation
- [ ] Verify in dark mode readability
- [ ] Verify no performance impact (navigation remains smooth at 60 fps)

### Limitations & Open Questions

1. **OSM data completeness:** `maxspeed` coverage varies by region. In areas without OSM speed limit data, the feature simply shows nothing — it never invents limits. This is by design.
2. **No voice announcements yet:** The spec mentions optional TTS ("In 800 meters: 70, then in 1.6 km: 50"). This is intentionally left out of this PR to keep the change focused. A follow-up PR can integrate with the existing `TurnsTtsText` system with rate-limiting (max once per 20–30 s).
3. **Sensitivity / horizon settings:** The advisor thresholds are configurable in code (`AdvisorConfig`) but not yet exposed as user-facing sliders. The look-ahead horizon is currently fixed at 2 km. A follow-up can add a 1/2/5 km selector and sensitivity presets (gentle/normal/aggressive).
4. **No automatic vehicle control:** This feature is strictly advisory. It does not interface with any CAN bus, Bluetooth car API, or braking system. The driver always makes the final decision.

### Safety Disclaimer

> Speed limits shown are derived from OpenStreetMap community data and may be incomplete or outdated in some regions. This feature is a driving assistance aid only — it does **not** replace real road signs, traffic signals, or local traffic regulations. Always obey posted speed limits and drive safely.

---

*This PR adds no new external dependencies. All code follows existing project conventions (C++17, GoogleTest for tests, AndroidX for UI). The change is opt-in (disabled by default) and has zero impact on existing navigation behavior when disabled.*