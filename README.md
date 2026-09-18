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

- Rival too slow: raise `AiCruiseSpeed`. Too crashy: lower it, raise `RetaskIntervalMs`.
- Driving style is `1074528293` (rushed, ignore lights, aggressive overtake) in `OpponentDriver.cs`. Normal-traffic style `786603` is there for comparison if you want a "lawful" mode later.
- Skill-by-car, rubber-banding, and nitro are intentionally left out of MVP — see roadmap.

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
