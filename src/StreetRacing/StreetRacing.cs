using System;
using System.Drawing;
using System.Windows.Forms;
using GTA;
using GTA.Math;
using GTA.Native;
using GTA.UI;

namespace StreetRacing
{
    public class StreetRacing : Script
    {
        private readonly StreetRacingConfig cfg;
        private RaceState state = RaceState.Idle;
        private readonly OpponentDriver ai = new OpponentDriver();

        private Vehicle oppVehicle;
        private Ped oppDriver;
        private Vector3 finish = Vector3.Zero;
        private Blip finishBlip;
        private Checkpoint finishCp;

        private int raceStartTime;
        private int lastHudTime;
        private int cooldownUntil;
        private int lastHonkAttempt;
        private bool hornWasDown;
        private bool wasLeading;
        private int activeStyle;

        private Telemetry telemetry;
        private int lastSenseTime;
        private int lastSampleTime;
        private RoadSense sense;
        private bool offroadIn;
        private int offroadEnterT;
        private int wrongwaySince;
        private bool wrongwayIn;
        private int underSince;
        private float lastSpeedB;
        private int lastSpeedT;
        private int lastBrakeEvent;
        private float lastHealth;

        public StreetRacing()
        {
            cfg = StreetRacingConfig.Load();
            Interval = 50;
            Tick += OnTick;
            KeyDown += OnKeyDown;
            Aborted += OnAborted;
            Notification.PostTicker("StreetRacing loaded: honk at a driver ahead to race.", false, false);
        }

        private void OnKeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == cfg.CancelKey && state == RaceState.Racing)
            {
                EndRace("Race cancelled.");
            }
        }

        private void OnTick(object sender, EventArgs e)
        {
            switch (state)
            {
                case RaceState.Idle:
                    TickIdle();
                    break;
                case RaceState.Racing:
                    TickRacing();
                    break;
                case RaceState.Cooldown:
                    if (Game.GameTime >= cooldownUntil)
                    {
                        state = RaceState.Idle;
                    }
                    break;
            }
        }

        private void TickIdle()
        {
            bool down = false;
            try
            {
                down = Game.IsControlPressed(GTA.Control.VehicleHorn);
            }
            catch
            {
            }
            bool rising = down && !hornWasDown;
            hornWasDown = down;
            if (!rising || Game.GameTime - lastHonkAttempt < cfg.HonkDebounceMs)
            {
                return;
            }
            lastHonkAttempt = Game.GameTime;

            if (!OpponentPicker.TryPick(cfg.MaxChallengeRange, out var vehicle, out var driver))
            {
                return; // silent: honking in empty traffic should do nothing
            }

            var player = Game.Player.Character;
            var origin = player.CurrentVehicle.Position;
            var heading = player.CurrentVehicle.ForwardVector;
            if (!FinishPicker.TryPick(origin, heading, cfg.MinDistance, cfg.MaxDistance, out var spot))
            {
                Notification.PostTicker("No road ahead for a finish line. Try facing open road.", false, false);
                return;
            }

            oppVehicle = vehicle;
            oppDriver = driver;
            finish = spot;

            try
            {
                finishBlip?.Delete();
                finishBlip = World.CreateBlip(finish);
                finishBlip.Sprite = BlipSprite.Standard;
                finishBlip.Color = BlipColor.Yellow;
                finishBlip.IsShortRange = false;
                finishBlip.ShowRoute = true;
                finishBlip.Name = "Race Finish";
            }
            catch
            {
            }

            try
            {
                finishCp?.Delete();
                finishCp = World.CreateCheckpoint(
                    CheckpointIcon.CylinderCheckerboard,
                    finish,
                    finish + new Vector3(0f, 0f, 10f),
                    cfg.FinishRadius,
                    Color.FromArgb(220, 255, 210, 0));
            }
            catch
            {
            }

            ai.Start(oppDriver, oppVehicle, finish, cfg.AiCruiseSpeed, cfg.ResolveDrivingStyle(), cfg.RefreshIntervalMs, cfg.StuckTimeoutMs);
            raceStartTime = Game.GameTime;
            lastHudTime = 0;
            wasLeading = true;
            activeStyle = cfg.ResolveDrivingStyle();
            lastSenseTime = 0;
            lastSampleTime = 0;
            sense = new RoadSense();
            offroadIn = false;
            wrongwaySince = 0;
            wrongwayIn = false;
            underSince = 0;
            lastSpeedB = oppVehicle.Speed;
            lastSpeedT = raceStartTime;
            lastBrakeEvent = 0;
            lastHealth = oppVehicle.Health;
            try
            {
                telemetry?.Close();
                telemetry = cfg.TelemetryEnabled ? new Telemetry(raceStartTime, activeStyle) : null;
            }
            catch
            {
                telemetry = null;
            }
            state = RaceState.Racing;
            Notification.PostTicker("Challenge accepted! First to the ~y~yellow marker~s~ wins.", false, false);
        }

        private void TickRacing()
        {
            var player = Game.Player.Character;
            if (player == null || !player.Exists() || player.IsDead)
            {
                EndRace("You died. Race over.");
                return;
            }
            if (!ai.Valid())
            {
                EndRace("Rival is out (wrecked / gone). Race over.");
                return;
            }
            if (Game.GameTime - raceStartTime > cfg.RaceTimeoutMs)
            {
                EndRace("Race timed out. Nobody got there.");
                return;
            }

            ai.OnTick();

            var youAt = player.IsInVehicle() ? player.CurrentVehicle.Position : player.Position;
            float dYou = FinishPicker.FlatDistance(youAt, finish);
            float dOpp = FinishPicker.FlatDistance(oppVehicle.Position, finish);

            int now = Game.GameTime;
            if (now - lastSenseTime >= 500)
            {
                lastSenseTime = now;
                try
                {
                    sense = RoadSense.Sample(oppVehicle);
                }
                catch
                {
                }
            }
            if (telemetry != null && now - lastSampleTime >= 100)
            {
                lastSampleTime = now;
                float rSpeed = oppVehicle.Speed;
                float ySpeed = player.IsInVehicle() ? player.CurrentVehicle.Speed : 0f;
                telemetry.Sample(now - raceStartTime, activeStyle, rSpeed, ySpeed,
                    dOpp, dYou, sense.OffRoad, sense.AlignDeg, sense.Traffic, sense.Frontal, cfg.AiCruiseSpeed);
                Detect(now - raceStartTime, rSpeed);
            }

            if (dYou < cfg.FinishRadius || dOpp < cfg.FinishRadius)
            {
                EndRace(dYou <= dOpp
                    ? $"~g~You win!~s~ {Math.Max(0, (int)dOpp)}m ahead of your rival."
                    : $"~r~You lose.~s~ Rival beat you by {Math.Max(0, (int)dYou)}m.");
                return;
            }

            if (Game.GameTime - lastHudTime > 1000)
            {
                lastHudTime = Game.GameTime;
                try
                {
                    // The game can drop a blip route when you stray far off-path;
                    // re-asserting rebuilds it instead of leaving you guideless.
                    if (finishBlip != null)
                    {
                        finishBlip.ShowRoute = true;
                    }
                }
                catch
                {
                }
                bool leading = dYou <= dOpp;
                string msg = leading
                    ? $"~y~RACE~s~  You: {(int)dYou}m  Rival: {(int)dOpp}m  ~g~you lead"
                    : $"~y~RACE~s~  You: {(int)dYou}m  Rival: {(int)dOpp}m  ~r~rival leads";
                if (leading != wasLeading)
                {
                    wasLeading = leading;
                }
                try
                {
                    GTA.UI.Screen.ShowSubtitle(msg);
                }
                catch
                {
                }
            }
        }

        private void Detect(int t, float rSpeed)
        {
            // Off-road episodes with hysteresis (7 m enter, 4 m exit).
            if (!offroadIn && sense.OffRoad > 7f)
            {
                offroadIn = true;
                offroadEnterT = t;
                telemetry.Event(t, "OFFROAD_ENTER", $"speed={rSpeed:F0}");
            }
            else if (offroadIn && sense.OffRoad < 4f)
            {
                offroadIn = false;
                telemetry.Event(t, "OFFROAD_EXIT", $"dur={(t - offroadEnterT) / 1000}");
            }

            // Wrong-way: facing >100 deg off the road direction, sustained 2 s.
            if (sense.AlignDeg > 100f && rSpeed > 8f)
            {
                if (wrongwaySince == 0)
                {
                    wrongwaySince = t;
                }
                else if (!wrongwayIn && t - wrongwaySince > 2000)
                {
                    wrongwayIn = true;
                    telemetry.Event(t, "WRONGWAY_ENTER", $"align={sense.AlignDeg:F0}");
                }
            }
            else
            {
                if (wrongwayIn)
                {
                    telemetry.Event(t, "WRONGWAY_EXIT", $"dur={(t - wrongwaySince) / 1000}");
                }
                wrongwayIn = false;
                wrongwaySince = 0;
            }

            // Under-drive: on road, 60 m+ clear ahead, but under 45% of cruise.
            if (sense.OffRoad < 5f && sense.Frontal > 60f && rSpeed < cfg.AiCruiseSpeed * 0.45f)
            {
                if (underSince == 0)
                {
                    underSince = t;
                }
                else if (t - underSince > 3000)
                {
                    underSince = t; // re-fire every 3 s while it persists
                    telemetry.Event(t, "UNDERDRIVE", $"speed={rSpeed:F0};frontal={sense.Frontal:F0}");
                }
            }
            else
            {
                underSince = 0;
            }

            // Panic braking: decel worse than -7 m/s^2.
            int dt = t - lastSpeedT;
            if (dt >= 100)
            {
                float accel = (rSpeed - lastSpeedB) / (dt / 1000f);
                if (accel < -7f && t - lastBrakeEvent > 3000)
                {
                    lastBrakeEvent = t;
                    telemetry.Event(t, "HARD_BRAKE", $"speed={rSpeed:F0};dec={accel:F0}");
                }
                lastSpeedB = rSpeed;
                lastSpeedT = t;
            }

            // Crash: health drop over one sample.
            float h = oppVehicle.Health;
            if (lastHealth - h > 8f)
            {
                telemetry.Event(t, "CRASH", $"dmg={lastHealth - h:F0};speed={rSpeed:F0};offroad={sense.OffRoad:F0}");
            }
            lastHealth = h;
        }

        private void EndRace(string message)
        {
            try
            {
                ai.Stop();
            }
            catch
            {
            }
            try
            {
                finishBlip?.Delete();
                finishCp?.Delete();
            }
            catch
            {
            }
            finishBlip = null;
            finishCp = null;
            try
            {
                telemetry?.Close();
            }
            catch
            {
            }
            telemetry = null;
            Notification.PostTicker(message, false, false);
            cooldownUntil = Game.GameTime + cfg.CooldownMs;
            state = RaceState.Cooldown;
        }

        private void OnAborted(object sender, EventArgs e)
        {
            try
            {
                ai.Stop();
            }
            catch
            {
            }
            try
            {
                finishBlip?.Delete();
                finishCp?.Delete();
                telemetry?.Close();
            }
            catch
            {
            }
            telemetry = null;
        }
    }
}
