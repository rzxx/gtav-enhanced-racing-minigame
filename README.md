# StreetRacing — impromptu street races for GTA V Enhanced

Drive up behind someone, honk, and race them to a random point on the map.
No checkpoints, no lobbies — first to the yellow marker wins.

## Status: v2 racing architecture

- **Trigger:** honk at an NPC driver ahead of you / in your camera view (scores angle + aim + distance, so it picks who you meant)
- **Finish:** random road point 1.2–2.8 km ahead (configurable), snapped to street, shown as yellow blip + GPS route + 3D cylinder
- **Start:** instant rolling start, rival launches the moment you honk
- **AI:** full pipeline `route -> corridor -> perception -> tactics -> trajectories -> speed -> actuator (stock DriveTo servo)`; see Architecture below
- **HUD:** ticker messages + 1 Hz subtitle with both distances, who leads, tactical mode and target speed
- **Cancel:** `G` key (configurable), plus auto-cancel on death/wreck/timeout

## Requirements

- GTA V Enhanced + ScriptHookV + ScriptHookVDotNet Enhanced (v3 API)
- .NET 8 SDK (builds `net48` via `Microsoft.NETFramework.ReferenceAssemblies`, no VS needed)

## Build + install

```powershell
# builds and copies StreetRacing.dll + default ini into the game
.\tools\install.ps1
# custom game path:
.\tools\install.ps1 -GtaDir "D:\path\to\Grand Theft Auto V Enhanced"
```

Then in game press **Insert** to reload scripts (or restart the game).

## How to race

1. Get in a car, pull up behind / alongside an NPC driver.
2. Tap the horn (`E`).
3. Rival launches immediately — chase the yellow GPS route to the marker.

## Config (`scripts\StreetRacing.ini`)

| Key | Default | What |
|---|---|---|
| Min/MaxDistance | 1200 / 2800 | finish band, random per race |
| FinishRadius | 30 | win radius, meters |
| MaxChallengeRange | 35 | honk search radius |
| AiCruiseSpeed | 47 | m/s (~170 km/h); planner caps by curvature/traffic, capability caps by car |
| DrivingStyle / DrivingStyleRaw | Reckless / 0 | low-level actuator mode only (intelligence is in the planner, not flags) |
| DriverProfile | Balanced | Cautious / Balanced / Aggressive — lookahead, reaction, margins, grip use, risk |
| RiskTolerance | -1 | -1 = preset, else 0..1 overrides profile risk |
| LookaheadTimeS / SafetyMarginM / GripFactor | 0 | 0 = preset; overrides for the profile |
| RefreshIntervalMs / StuckTimeoutMs | 2000 / 4000 | actuator refresh + stuck repath |
| RaceTimeoutMs / CooldownMs | 600000 / 8000 | give-up timer, rest between races |
| CancelKey | G | cancel active race |
| DebugViz | 0 | 1 = in-game overlay: route/corridor/candidates/predictions/braking |
| Actuator | Direct | Direct (intended: executes joint path/speed every tick) or GtaDriver (baseline/diagnostic only) |

## Architecture (v3 — joint maneuver)

```
race route -> drivable corridor -> persistent perception -> tactics
    -> JOINT maneuver (path + speed together) -> Direct -> DriveV
```

- **Route (`Route/RaceRoute.cs`):** real connected GPS route first (`GET_GPS_BLIP_ROUTE_FOUND` / `GET_POS_ALONG_GPS_TYPE_ROUTE`, type 1 first, distance- then index-interpretation, densely sampled ~5 m so interpolation never cuts junctions into bad tangents), with connected street-walk fallback and straight last resort. `Source` (`GpsDist/GpsIdx/FallbackWalk/StraightFallback`) is logged; a `TryUpgradeToGps` pass adopts GPS within ~12 s if the blip route wasn't ready at Start (forward-compatible reproject only, dot > 0.5, else the upgrade is rejected). Localization is continuity-aware: predicted progress from motion, search around the expected station, score by distance + heading (dot > 0.5 required, dot <= 0 never healthy) + continuity; >12 m jumps while the car barely moves are rejected (`LocJump` + lost) instead of teleporting progress. Loss is tightened (`HeadingIncompatible` > 65 deg / 400 ms, `WrongDirection` > 70 deg, `WentBackwards` > 20 m, recovery re-acquire < 45 deg). `ValidateStart` gates the start line (close forward projection, <45 deg, continuous forward 30 m, no sharp branch) — bad destinations are rerolled/rejected, never raced. Recovery is a heading-compatible merge search (`TryGetRecoveryMerge`: <45 deg route, <65 deg bearing, reachable), not a blind 40 m point.
- **Corridor (`Road/RoadCorridor.cs`):** FIXED `GET_ROAD_BOUNDARY_USING_HEADING` (one output, not two — probes left/right via ±90° with width/midpoint validation + on-road cross-check). Sampled profile along the horizon (slices every 10 m to lookahead+60 m, `HalfWidthAt/MinHalfWidthAhead/SliceAt`), not one `HalfWidth`. Sweep fallback, then conservative default.
- **Perception (`Sense/Perception.cs`):** PERSISTENT tracking by handle with short expiry + coast/hysteresis (no clear/rebuild TTC-sorted top-24). Always retains route/path-relevant actors, the nearby safety bubble and the rival; stable relevance ordering (TTC is one signal, not the retention order). Route-frame actors (`RouteS/RouteLateral/SpeedAlong/ClosingAlong/RouteTtc`) + `OffRoadway` flag so sidewalk peds/props never constrain paths they cannot intersect. `TryGetLeadOnRoute` for Follow. Explicit `Reset()` per race (tracks/scan/retention cleared).
- **Capability (`Planning/VehicleCapability.cs`):** spin/yaw rejection via slip + yaw gates; confidence 0..1 (rises stable, collapses on slide/spin); only stable physical samples adapt brake/lat/top. Impacts/teleports never train it. `Seed()` fully resets per race (incl. observed peaks).
- **Maneuver (`Planning/TrajectoryPlanner.cs` + `SpeedPlanner.cs`):** JOINT 7-candidate plan, now POSE-aware. Every candidate is a cubic Hermite `d(0)=current lateral, d'(0)=-tan(headErr), d(S)=desired, d'(S)=0` on 5 m stations, so all paths begin FORWARD from the nose and spread laterally; a 90 deg merge yields large `maxKappa` + `pose-incompatible` instead of `maxKappa~0.01`. Per candidate: curvature speed profile → station arrival times → predict actors at those times → swept-envelope test along THAT path → constrain speed only for actors conflicting with THAT path → backwards braking pass (+ forward accel feasibility) → score the complete maneuver (safety/clearance, progress/mean-speed, smoothness/curvature+decel, tactical/inside intent, road margin, hysteresis). Winner is path+speed together. Hard invariant: `|headErr| > 50 deg` never yields a cruise maneuver — the brain crawls (`PoseHold` 0 m/s / `PoseMerge` 5 m/s) or holds via a pose-aware recovery connector (`RecoveryMerge` 8 m/s, `maxKappa > 0.25` rejected as U-turn). `Reset()` per race (`PlanId` restarts at 1).
- **Tactics (`Tactics/RaceTactics.cs`):** same modes/gates, route-aware, skips `OffRoadway` sidewalk clutter in `SideClear/Blocked/Clearance`. Forces `Recovery` on `|headErr| > 60 deg`. Explicit `Reset()` per race (no more starting in `Crashed`).
- **Actuator (`Control/`):** `IVehicleActuator` seam now passes the full maneuver (`SetManeuver`: sampled `Path` + `StationS` + `SpeedProfile`). `DirectActuator` = intended controller: local speed-dependent lookahead on the SELECTED path + pure-pursuit steering + PI on LOCAL planned speed, every script tick (~20 Hz) independently of the ~10 Hz plan cadence. `GtaDriverActuator` = baseline/diagnostic only (servo-tracks aim+speed, rate-limited re-issue). Both report extended `PathFollowingError` (cross-track/heading/local-speed + steer/throttle/brake) every tick. Controller state fully reset per race (new instance + zeroed integrators).
- **Viz (`Debug/RaceDebugViz.cs`):** route/corridor/candidates/chosen/predictions/aim/braking markers (`DebugViz=1`), plus ego nose vector (white) vs route tangent (magenta) — on a straight start all gray/cyan candidate lines must leave the nose forward; sideways lines mean the race must not start.
- **Skill (`Core/DriverProfile.cs`):** unchanged numbers (no tuning this pass).
- **Impacts (`Sense/ImpactClassifier.cs`):** unchanged.

## AI diagnostics (telemetry)

Each race writes `scripts\StreetRacing_race_<id>.csv` (10 Hz) plus `_events.csv`.
Disable with `TelemetryEnabled=0`. Send both files after test races to tune further.

Samples: legacy 42 cols unchanged, then `chIdx,chMeanV,chMinV,constrHandle,constrKind,constrS,minPredClear,planId,steerDeg,thr01,brk01,localVTgt` — chosen candidate + its speed profile summary, which actor constrained which station, predicted clearance, planner id, controller errors/outputs — then pose-foundation cols `egoHead_deg,routeHead_deg,firstTangErr_deg,locExpected_m,locJump_m` (heading compatibility + continuity audit).

Events: `START / ROUTE / START_POSE / LOC / ROUTE_AHEAD / RECOVERY_MERGE / ROUTE_INVALID / ROUTE_VALID / GPS_ROUTE / ACTUATOR / TACTIC / ROUTE_LOST / ROUTE_FOUND / IMPACT / TELEPORT / HARD_BRAKE / CTRL / PATH_ERR / PLAN` (`PLAN` now includes `egoHead/routeHead/headErr/firstTangErr/maxKappa/seg/s/expS/locDetail`; `START_POSE` + `LOC` carry the full pose-foundation snapshot; `RECOVERY_MERGE` names the selected merge station or why none exists).

Decisive test (straight road first, before any traffic/racecraft tuning): honk at a same-direction rival on a straight road with `DebugViz=1`. Expect: `START_POSE` shows `headErr` < ~15 deg; every gray/cyan candidate line leaves the nose FORWARD and spreads laterally; `firstTangErr` < ~10 deg, `maxKappa` < ~0.005, `v_tgt` near cruise, `prog_m` advances monotonically (~speed × t) over the first 3 s with no 18–30 m jumps (`locJump_m` ≈ motion per tick). If the debug lines go sideways or `headErr` ≈ 90 deg on sample 1, the start gate must reject the race (`ARM_REJECT`) — do not tune steering, profiles, TTC, overtaking or peds until this holds consistently.

Retired: `offroad_m` (invalid street-node distance), 40 m frontal-only sensing, `dec`-only brake detection, alignment-only wrong-way detection, long-range `DriveTo(finish)`, global corridor-wide obstacle speed, creep heuristic, TTC-sorted top-24 perception rebuild.

## Tuning the AI

- Race pace comes from `AiCruiseSpeed` + `DriverProfile` + `RiskTolerance`, not from driving-style flags. Start with `Balanced`, then `Aggressive` / `RiskTolerance=0.85` for wilder rivals or `Cautious` for cleaner ones.
- `DrivingStyle` only changes the low-level actuator (e.g. `Psycho` adds oncoming-lane use, `Disciplined` is a lane-keeping diagnostic). If the planner is right, `Reckless` is enough.
- If rivals still feel capped at high speed, the remaining suspect is
  `vehicleaihandlinginfo.meta` (what Eddlm's Faster AI Drivers mod edits): the game caps
  non-racing AI below a car's true top speed. Optional experiment, needs OpenIV.

## Roadmap (post-MVP ideas)

- Betting + rival purse, skill scaled to car class
- Standing countdown start at red lights
- Multiple simultaneous rivals
- Cops/wanted heat during races
- Traffic density tweaks while racing
- Saved/edited routes, optional waypoint-as-finish mode

## Troubleshooting

- Script won't load: check `ScriptHookVDotNet.log` in the game folder for `StreetRacing` errors.
- No challenge on honk: you must be the driver, target within 35 m and roughly ahead / looked-at.
- Rival dumb at one spot: send the `_events.csv` — look for `ROUTE_LOST` / `RECOVERY` / `CTRL` lines around that time; stuck recovery re-paths after 4 s crawling.
