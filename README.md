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

## Architecture (v2)

```
race route -> drivable corridor -> perception/prediction -> tactics
    -> candidate trajectories -> speed profile -> actuator -> DriveV
```

- **Route (`Route/RaceRoute.cs`):** dense centerline polyline with continuous progress (`AlongS`), lookahead points, curvature ahead, and loss detection (`AwayFromRoute / WrongDirection / WentBackwards / Circling / NoProgress`). Nearest search prefers same-direction segments so carriageway splits don't snap across; recovery aims at a near route point, never the 2 km finish.
- **Corridor (`Road/RoadCorridor.cs`):** usable road width + boundaries via `GET_ROAD_BOUNDARY_USING_HEADING`, falling back to an `IS_POINT_ON_ROAD` lateral sweep, then a conservative default. `OffCorridor = max(0, |lateral| - halfWidth)` replaces the retired `offroad_m`, which measured distance to a *future* street node and was invalid as a departure signal.
- **Perception (`Sense/Perception.cs`):** 10 Hz surround scan with range from stopping distance + margin (80–170 m, not a fixed 40 m cone). Every vehicle / rival / ped / prop-obstacle gets relative velocity, closing speed and TTC; constant-velocity prediction feeds trajectory scoring.
- **Capability (`Planning/VehicleCapability.cs`):** braking / lateral-g / top-speed estimates seeded from handling data (`BrakeForce`, `TractionCurveMax`) and adapted from plausible tyre samples only — impacts/teleports never train it. No hardcoded vehicle classes.
- **Trajectories (`Planning/TrajectoryPlanner.cs`):** 5 lateral candidates across the full usable width, scored by clearance to predicted actors, lane-change cost, corner-inside bias and tactical bias. Best wins; rejected alternatives are logged.
- **Speed (`Planning/SpeedPlanner.cs`):** `min(cruise, curvature limit, obstacle limit, tactical limit)` from measured grip/brakes, with braking-distance feasibility at 40/80/120 m and TTC guards.
- **Tactics (`Tactics/RaceTactics.cs`):** `Cruise / Follow / AttackSetup / OvertakeLeft / OvertakeRight / Commit / Abort / SideBySide / Defend / CornerPrep / Recovery / Crashed` with explicit commit/abort gates (predicted-clear lane + TTC, gap collapse or corner aborts). Overtakes may cross lanes; traffic/ped awareness is preserved (they score as blocks, the style flag just doesn't queue).
- **Actuator (`Control/`):** `IVehicleActuator` seam. `GtaDriverActuator` uses the stock driver as a receding-horizon servo to the planned aim point (80–150 m), re-issuing only on plan change / stuck — no long-range `DriveTo(finish)` losses. `DirectActuatorStub` documents the future steering/throttle/brake swap.
- **Skill (`Core/DriverProfile.cs`):** lookahead time, reaction interval, safety margin/time, grip factor, risk tolerance, corner caution. No random steering noise.
- **Impacts (`Sense/ImpactClassifier.cs`):** `dec < -12` impossible for tyres; those samples are `Impact`/`Teleport` (position jump, health drop, collision flag), never `HARD_BRAKE`.

## AI diagnostics (telemetry)

Each race writes `scripts\StreetRacing_race_<id>.csv` (10 Hz) plus `_events.csv`.
Disable with `TelemetryEnabled=0`. Send both files after test races to tune further.

Samples: `t_ms,style,tactical,prog_m,prog_pct,look_m,lat_m,halfW_m,offCorr_m,headErr_deg,curv,aimLat,chScore,rejLat,rejScore,v_tgt,v_act,v_lim,brakeNeed,aBrake,aLat,nActors,nearD,nearTTC,nearClose,cmdCruise,cmdStyle,routeLost,impact,reissue,finishGap`

Events: `START / ROUTE / TACTIC / ROUTE_LOST / ROUTE_FOUND / IMPACT / TELEPORT / HARD_BRAKE / CTRL`

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
