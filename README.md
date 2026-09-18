# StreetRacing — impromptu street races for GTA V Enhanced

Drive up behind someone, honk, and race them to a random point on the map.
No checkpoints, no lobbies — first to the yellow marker wins.

## Status: COLLAPSED minimal driver (no racecraft until foundations pass)

Complexity was retired before competence: the 7-candidate + tactics +
Crashed stack oscillated laterally, braked for its own curvature, false-
triggered IMPACT with zero damage, deadlocked recovery at 0 m/s, and
inferred reverse inside the actuator. Do not re-add states, candidates, or
guards until the dumb driver below passes controlled tests.

- **Trigger:** honk at an NPC driver ahead of you / in your camera view (scores angle + aim + distance, so it picks who you meant)
- **Finish:** random road point 1.2–2.8 km ahead (configurable), snapped to street, shown as yellow blip + GPS route + 3D cylinder
- **Start:** instant rolling start, rival launches the moment you honk (start-pose gate rejects sideways routes instead of recovering from bad setup)
- **AI (default `DriverMode=Simple`):** `GPS route -> stable localization -> ONE pose-feasible center trajectory (PoseConnector +tan) -> curvature-capped fixed/moderate speed -> Direct`. No candidates, no opponent tactics, no overtaking, no collision avoidance, no `Crashed`, no dynamic recovery FSM, no civilian behavior (unless `EnablePassing=1` for the single-blocker test).
- **Diag (`DriverMode=DirectDiag`):** Phase-1 hardware probe — no perception/route/planner. Straight-road `throttle(6s) -> coast(2s) -> brake -> steerL/R` with `Throttle+ThrottlePower/brake/gear/RPM/speed/accel` logging. Prove Direct moves a DriveV car before any AI work.
- **Legacy (`DriverMode=Legacy`):** old full stack preserved for comparison only (7 candidates + tactics). Not the milestone path.
- **HUD:** ticker messages + 1 Hz subtitle with both distances, who leads, intent/mode and target speed
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
| DebugViz | 0 | 1 = in-game overlay: route/corridor/single-path/aim |
| Actuator | Direct | Direct (executes maneuver every tick) or GtaDriver (emergency rejoin only) |
| DriverMode | Simple | Simple (dumb follower, default) / DirectDiag (hardware probe) / Legacy (old stack, comparison only) |
| SimpleCruise | 18 | Simple follower cruise cap m/s; effective = min(AiCruiseSpeed, SimpleCruise) |
| EnablePassing | 0 | 1 = single-blocker FOLLOW/PASS test (Phase 5); 0 = pure route following |
| UseGtaRejoin | 0 | 1 = emergency low-speed GTA DriveTo rejoin fallback; never normal driving |
| DiagCruise | 18 | DirectDiag probe target speed m/s |

## Architecture (COLLAPSED — minimal competent driver)

```
Simple (default): GPS route -> stable localization -> ONE center path
    (PoseConnector +tan) -> curvature-capped fixed speed -> Direct -> DriveV
Diag:             fixed throttle/coast/brake/steer stages -> Direct -> DriveV
Legacy:           old joint 7-candidate stack (DriverMode=Legacy only)
```

- **Pose connector (`Core/PoseConnector.cs`, single source):** cubic Hermite `d(0)=current lateral, d'(0)=+tan(headErr), d(S)=0, d'(S)=0`. Sign proof in file: route-north/ego+10°-right gives `d' = -sin10/cos10 = tan(-10)`. Verified: ego 94°/route 124° reconstructs 94° (err 0); old `-tan` gave 154° (err +60°). `VerifyToward()` guards every Simple plan (first tangent must rotate toward the nose); `SelfTest()` returns `OK` and is logged at race start.
- **Route (`Route/RaceRoute.cs`, preserved):** real connected GPS route first, fallback walk, straight last resort; continuity-aware localization (expected station, heading dot, 12 m jump guard); `ValidateStart` gates sideways starts; `TryGetRecoveryMerge` finds heading-compatible future merges. Unchanged geometry, new owner (SimpleBrain).
- **Corridor (`Road/RoadCorridor.cs`, preserved):** sampled half-width profile for margin audit + pass-width check. No decisions in Simple mode beyond `MinHalfWidthAhead` for the gated pass test.
- **Capability (`Planning/VehicleCapability.cs`, preserved):** handling-seeded + stable-sample adaptation. Impacts/teleports never train it.
- **Simple follower (`Race/SimpleBrain.cs`, DEFAULT):** one center path per plan tick (no lateral alternatives, so no whole-road oscillation); curvature-only speed (`sqrt(aLat/k)` + braking/accel passes, no obstacle planner); stable `AlongS`; `INTENT` events (not `PLAN` flicker). Speed defaults to `min(AiCruiseSpeed, SimpleCruise)` ≈ 15–20 m/s.
- **Intent (`Race/ManeuverIntent.cs`, Phase 4 foundation):** `KEEP_LINE / FOLLOW / PASS_LEFT / PASS_RIGHT / RECOVER` with 1.5 s dwell hysteresis. Trajectory is generated INSIDE the intent. Simple defaults to `KEEP_LINE`; `RECOVER` only via the primitive below; `FOLLOW/PASS` only when `EnablePassing=1`.
- **Recovery (`Race/RecoveryPrimitive.cs`, Phase 3):** explicit `Stop -> Reverse (controlled 8 m, Reverse=true) -> Forward crawl -> Rejoin (heading-compatible connector)`. Entry needs corroboration (IsLost / headErr>50 / collision+decel / damage+decel / 4 s no-progress) — never decel alone. Never holds 0 indefinitely: no-merge crawls at ≤4 m/s so pose changes. `ManeuverCommand.Reverse` is the ONLY reverse authority. GTA `DriveTo` is emergency-rejoin only (`UseGtaRejoin=1`).
- **Actuator (`Control/DirectActuator.cs`):** pure-pursuit + PI on the single path. Commands BOTH `Throttle` and `ThrottlePower` (DriveV hardware needs both on some cars). Reverse ONLY when `cmd.Reverse` is set. Legacy secret `headErr>130°` auto-reverse is deleted.
- **Impacts (`Sense/ImpactClassifier.cs`, fixed):** decel alone (even <-12) is NEVER Impact — it is Braking. Impact needs `damage>=4` or `HasCollided` plus strong decel. False zero-damage IMPACTs no longer feed recovery.
- **Tactics (`Tactics/RaceTactics.cs`, retired from default):** `Crashed` never assigned (kept in enum for compat). Legacy brain only.
- **Legacy (`Race/RaceBrain.cs`, `Planning/TrajectoryPlanner.cs`, `Sense/Perception.cs`):** preserved for `DriverMode=Legacy` comparison. Pose sign fixed there too; legacy no-merge hold changed from 0 m/s deadlock to 3 m/s crawl.
- **Viz (`Debug/RaceDebugViz.cs`, preserved):** draws the single Simple path (cyan) + route/corridor/nose-vs-tangent. On a straight start the single line must leave the nose forward.
- **Skill (`Core/DriverProfile.cs`):** unchanged numbers (no tuning until foundations pass).

## AI diagnostics (telemetry)

Each race writes `scripts\StreetRacing_race_<id>.csv` (10 Hz) plus `_events.csv`.
Disable with `TelemetryEnabled=0`. Send both files after test races to tune further.

Samples: legacy 42 cols unchanged, then `chIdx,chMeanV,chMinV,constrHandle,constrKind,constrS,minPredClear,planId,steerDeg,thr01,brk01,localVTgt` — chosen candidate + its speed profile summary, which actor constrained which station, predicted clearance, planner id, controller errors/outputs — then pose-foundation cols `egoHead_deg,routeHead_deg,firstTangErr_deg,locExpected_m,locJump_m` (heading compatibility + continuity audit).

Events (Simple): `START / ROUTE (poseCheck=OK) / START_POSE / GPS_ROUTE / ACTUATOR / INTENT (intent/why/held + lat/v/lim + egoHead/routeHead/headErr/firstTang/maxKappa/s) / RECOVER_ENTER / RECOVER_EXIT / PASS_COMMIT (only if EnablePassing=1) / POSE_CONNECTOR_FAIL (must never fire) / IMPACT (corroborated only) / TELEPORT`. Legacy adds `PLAN / TACTIC / ROUTE_LOST / RECOVERY_MERGE`.

Decisive tests in order (do not tune profiles/heuristics until each passes):

1. **Direct hardware (`DriverMode=DirectDiag`, straight road):** `DIAG_STAGE` Throttle→Coast→Brake→SteerL→SteerR→Done with `DIAG` rows showing `cmdThr=1/actThr/thrPow` rising together and `spd` climbing from 0 toward `DiagCruise` (15–20), then falling under brake, then lateral response to ±12° steer. If `thr=1/brk=0` yet `spd=0`, inspect `thrPow/gear/rpm/eng` columns — fix hardware layer first.
2. **Dumb follower (`DriverMode=Simple`, `EnablePassing=0`, straight road, `DebugViz=1`):** `START_POSE headErr` < ~15°; the SINGLE cyan line leaves the nose forward; `INTENT` stays `KEEP_LINE`; `firstTangErr` < ~10°, `maxKappa` < ~0.005, `v_tgt` near `SimpleCruise`, `prog_m` advances monotonically (~speed × t) with `locJump_m` ≈ motion per tick. Lateral target must NOT jump across the road (single endLat=0 by construction).
3. **City route:** 1–2 km ordinary route at ~15–20 m/s without leaving road, stopping for no reason, oscillating, or entering `RECOVER`. Only then enable `EnablePassing=1` for the one-stopped-civilian pass test, then rebuild racecraft.

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
