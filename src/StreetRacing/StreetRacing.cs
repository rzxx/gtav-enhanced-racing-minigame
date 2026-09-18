using System;
using System.Drawing;
using System.Windows.Forms;
using GTA;
using GTA.Math;
using GTA.UI;
using StreetRacing.Race;

namespace StreetRacing
{
    /// Race lifecycle (challenge -> racing -> cooldown). Driving intelligence
    /// lives in RaceBrain (route -> corridor -> perception -> tactics ->
    /// trajectory -> speed -> actuator); this class only owns the trigger,
    /// finish marker, win/lose checks and HUD.
    public class StreetRacing : Script
    {
        private readonly StreetRacingConfig cfg;
        private RaceState state = RaceState.Idle;
        private readonly RaceBrain brain = new RaceBrain();

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
        private DriverProfile activeProfile;

        private RaceTelemetry telemetry;

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

            activeStyle = cfg.ResolveDrivingStyle();
            activeProfile = cfg.ResolveDriverProfile();
            raceStartTime = Game.GameTime;
            lastHudTime = 0;
            wasLeading = true;
            try
            {
                telemetry?.Close();
                telemetry = cfg.TelemetryEnabled ? new RaceTelemetry(raceStartTime, activeStyle, activeProfile.Name) : null;
            }
            catch
            {
                telemetry = null;
            }
            try
            {
                brain.Start(oppDriver, oppVehicle, finish, cfg.AiCruiseSpeed, activeStyle,
                    activeProfile, telemetry, cfg.RefreshIntervalMs, cfg.StuckTimeoutMs);
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(0, "BRAIN_START_FAIL", ex.Message); } catch { }
                EndRace("Rival failed to start. Race over.");
                return;
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
            if (!brain.Valid())
            {
                EndRace("Rival is out (wrecked / gone). Race over.");
                return;
            }
            if (Game.GameTime - raceStartTime > cfg.RaceTimeoutMs)
            {
                EndRace("Race timed out. Nobody got there.");
                return;
            }

            try
            {
                brain.OnTick();
            }
            catch
            {
                // One bad planning tick must not kill the race; the actuator
                // keeps executing its last plan.
            }

            var youAt = player.IsInVehicle() ? player.CurrentVehicle.Position : player.Position;
            float dYou = FinishPicker.FlatDistance(youAt, finish);
            float dOpp = FinishPicker.FlatDistance(oppVehicle.Position, finish);

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
                if (leading != wasLeading)
                {
                    wasLeading = leading;
                }
                string lead = leading ? "~g~you lead" : "~r~rival leads";
                string msg;
                try
                {
                    msg = $"~y~RACE~s~  You: {(int)dYou}m  Rival: {(int)dOpp}m  {lead} ~s~[{brain.TacticalName} {brain.TargetSpeed:F0}]";
                }
                catch
                {
                    msg = leading
                        ? $"~y~RACE~s~  You: {(int)dYou}m  Rival: {(int)dOpp}m  ~g~you lead"
                        : $"~y~RACE~s~  You: {(int)dYou}m  Rival: {(int)dOpp}m  ~r~rival leads";
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

        private void EndRace(string message)
        {
            try
            {
                brain.Stop();
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
                brain.Stop();
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
