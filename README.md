# StreetRacing — impromptu street races for GTA V Enhanced

Drive up behind someone, honk, and race them to a random point on the map.
No checkpoints, no lobbies — first to the yellow marker wins.

## Status: MVP

- **Trigger:** honk at an NPC driver ahead of you / in your camera view (scores angle + aim + distance, so it picks who you meant)
- **Finish:** random road point 1.2–2.8 km ahead (configurable), snapped to street, shown as yellow blip + GPS route + 3D cylinder
- **Start:** instant rolling start, rival launches the moment you honk
- **AI:** rushed driving style (ignores lights, overtakes), maxed ability/aggression, auto re-path every 6 s + stuck recovery
- **HUD:** ticker messages + 1 Hz subtitle with both distances and who leads
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
| AiCruiseSpeed | 47 | m/s (~170 km/h); game clamps to car |
| RetaskIntervalMs / StuckTimeoutMs | 6000 / 4000 | AI re-path + stuck recovery |
| RaceTimeoutMs / CooldownMs | 600000 / 8000 | give-up timer, rest between races |
| CancelKey | G | cancel active race |

## Tuning the AI (the fun part)

Found by dumping `VehicleDrivingFlags` from SHVDN metadata + FiveM native docs:

- **v1 bug (big one):** we called the wrong `DriveTo` overload — `(speed, flags, radius)`
  instead of `(radius, speed, style)` — so the rival was capped at **15 m/s (~54 km/h)**.
  Fixed; `AiCruiseSpeed` now actually applies.
- **v1 style bug:** `Rushed (1074528293)` contains the `StopForVehicles` flag, so the AI
  queued behind traffic and only passed when the road opened up. Default is now
  `Reckless (1074528292)` = Rushed minus that flag, so it swerves/passes instead.
- We also set `SET_DRIVER_RACING_MODIFIER 1.0` (game scripts use 0.2/0.5/1.0 for race drivers)
  and refresh cruise speed + style per tick via `SET_DRIVE_TASK_CRUISE_SPEED` /
  `SET_DRIVE_TASK_DRIVING_STYLE` instead of re-issuing the task (repath = brake stutter).
- A/B test styles live via `DrivingStyle` in the ini (`Calm/Rushed/Reckless/Psycho`)
  or any raw int via `DrivingStyleRaw`, then Insert to reload. No recompile.
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
- Rival dumb at one spot: stuck recovery re-paths after 4 s crawling; report the location if repeatable.
