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
| Actuator | GtaDriver | GtaDriver (experiment) or Direct (steering/throttle/brake) |

## Architecture (v2 — trajectory foundations)

```
race route -> drivable corridor -> perception/prediction -> tactics
    -> candidate trajectories -> speed profile -> actuator -> DriveV
```

- **Route (`Route/RaceRoute.cs`):** real connected GPS route first (`GET_GPS_BLIP_ROUTE_FOUND` / `GET_POS_ALONG_GPS_TYPE_ROUTE`, validated geometrically across route types 1/0/2, distance- then index-interpretation, resampled ~18 m), with connected street-walk fallback and straight last resort. `Source` (`GpsDist/GpsIdx/FallbackWalk/StraightFallback`) is logged; a `TryUpgradeToGps` pass adopts GPS within ~12 s if the blip route wasn't ready at Start. Progress/loss/recovery as before (same-direction bias, near-point recovery, never the 2 km finish).
- **Corridor (`Road/RoadCorridor.cs`):** FIXED `GET_ROAD_BOUNDARY_USING_HEADING` (one output, not two — probes left/right via ±90° with width/midpoint validation + on-road cross-check). Sampled profile along the horizon (slices every 10 m to lookahead+60 m, `HalfWidthAt/MinHalfWidthAhead/SliceAt`), not one `HalfWidth`. Sweep fallback, then conservative default.
- **Perception (`Sense/Perception.cs`):** route-frame actors (`RouteS/RouteLateral/SpeedAlong/ClosingAlong/RouteTtc`) alongside ego-frame; all planning uses the route frame. FIXED `ClosestTtcIndex` (resolved after sort, was invalid). `TryGetLeadOnRoute` for speed following.
- **Capability (`Planning/VehicleCapability.cs`):** spin/yaw rejection via slip + yaw gates; confidence 0..1 (rises stable, collapses on slide/spin); only stable physical samples adapt brake/lat/top. Impacts/teleports never train it.
- **Trajectories (`Planning/TrajectoryPlanner.cs`):** 7 smooth sampled paths (smoothstep start→target lateral, stations every 10 m) through corridor slices. Swept scoring: per-station boundaries, path curvature (`MaxKappa`, lateral-g demand), predicted-actor distance to the polyline over transit time, tactical/inside bias. Blocked best yields to first viable unless committed.
- **Speed (`Planning/SpeedPlanner.cs`):** curvature profile every 10 m to lookahead+80 m with backwards braking pass (`v[i]=min(vAllow[i],sqrt(v[i+1]²+2·a·ds))`), so future corners constrain now. FIXED `CornerCaution` (divide: >1 slower; numbers unchanged). Obstacle speeds from projected `SpeedAlong`, not `Speed*0.7`. `BrakingPointS` + full profile exposed for viz.
- **Tactics (`Tactics/RaceTactics.cs`):** same modes/gates, now route-aware (`SideClear/Blocked/Clearance` use route lateral/dist; narrow-road uses profile min width).
- **Actuator (`Control/`):** `IVehicleActuator` seam, planner unchanged. `GtaDriverActuator` = EXPERIMENT (short-horizon servo, rate-limited re-issue; hands point/speed to GTA pathfinding, does not guarantee trajectory). `DirectActuator` = real pure-pursuit + PI longitudinal (`SteeringAngle/Throttle/BrakePower`), selectable via `Actuator=Direct`. Both report `PathFollowingError` (desired vs actual) every plan tick — the decisive measurement.
- **Viz (`Debug/RaceDebugViz.cs`):** route/corridor/candidates/chosen/predictions/aim/braking markers (`DebugViz=1`).
- **Skill (`Core/DriverProfile.cs`):** unchanged numbers (no tuning this pass).
- **Impacts (`Sense/ImpactClassifier.cs`):** unchanged.

## AI diagnostics (telemetry)

Each race writes `scripts\StreetRacing_race_<id>.csv` (10 Hz) plus `_events.csv`.
Disable with `TelemetryEnabled=0`. Send both files after test races to tune further.

Samples: `t_ms,...,finishGap,routeSrc,minHalfW,pathErrLat,pathErrHead,speedErr,distToPath,brakePtS,capConf,actuator,chosenReject,minMargin,maxKappa` (first 31 cols unchanged)

Events: `START / ROUTE / GPS_ROUTE / ACTUATOR / TACTIC / ROUTE_LOST / ROUTE_FOUND / IMPACT / TELEPORT / HARD_BRAKE / CTRL / PATH_ERR`

Decisive test: run 2–3 races with `DebugViz=1`, then read `distToPath/speedErr` + `PATH_ERR` events. If GTA's servo holds <4 m / <4 m/s on twisty roads, it follows; if it repeatedly cuts/swings/caps (expectation: it will not hold precise trajectories under DriveV), set `Actuator=Direct` and re-test — planner output is identical, only the servo changes.

Retired: `offroad_m` (invalid street-node distance), 40 m frontal-only sensing, `dec`-only brake detection, alignment-only wrong-way detection, long-range `DriveTo(finish)`.

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
