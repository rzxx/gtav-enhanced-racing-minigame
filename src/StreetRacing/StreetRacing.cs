using System;
using System.Drawing;
using System.Windows.Forms;
using GTA;
using GTA.Math;
using GTA.Native;
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

        // Route-setup invariant: prefer a usable GPS route BEFORE releasing
        // the rival. The GPS blip route takes a frame or two to compute; Start
        // on the same tick as ShowRoute=true almost always Build()s a
        // FallbackWalk, then upgrades underneath the planner 1s later
        // (progress/heading jump). Arming holds the challenge up to 1.5s for
        // GET_GPS_BLIP_ROUTE_FOUND so Build() starts on GPS when possible.
        private bool hasPending;
        private Vehicle pendingOppVehicle;
        private Ped pendingOppDriver;
        private Vector3 pendingFinish = Vector3.Zero;
        private int pendingSince;
        private int pendingStyle;
        private DriverProfile pendingProfile;
        private int pendingAttempts;

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
            // --- Arming: waiting for the GPS route before releasing the rival.
            if (hasPending)
            {
                try
                {
                    if (pendingOppVehicle == null || !pendingOppVehicle.Exists()
                        || pendingOppDriver == null || !pendingOppDriver.Exists() || pendingOppDriver.IsDead)
                    {
                        CancelPending();
                        return;
                    }
                    var pc0 = Game.Player.Character;
                    if (pc0 == null || !pc0.Exists() || !pc0.IsInVehicle())
                    {
                        CancelPending();
                        return;
                    }
                }
                catch { CancelPending(); return; }
                bool gpsReady = false;
                try { gpsReady = Function.Call<bool>(Hash.GET_GPS_BLIP_ROUTE_FOUND); }
                catch { gpsReady = false; }
                bool timedOut = Game.GameTime - pendingSince > 1500;
                if (gpsReady || timedOut)
                {
                    // LOCAL START validity gate: a globally plausible GPS
                    // polyline (starts within 220 m, ends near finish) can
                    // still be ~90 deg sideways from the rival (wrong branch /
                    // carriageway split). Verify near the rival there is a
                    // close forward-compatible projection and the first tens
                    // of meters run forward before releasing. On failure,
                    // reroll the destination (new finish + new blip route)
                    // instead of trying to recover from a bad setup.
                    string startWhy = "";
                    bool startOk = false;
                    try { startOk = ValidateRouteForRival(pendingOppVehicle, pendingFinish, out startWhy); }
                    catch { startOk = false; startWhy = "validate-exc"; }
                    if (!startOk)
                    {
                        pendingAttempts++;
                        if (pendingAttempts >= 3)
                        {
                            try { telemetry?.Event(0, "ARM_REJECT", $"no-sane-forward-route attempts={pendingAttempts};{startWhy}"); } catch { }
                            Notification.PostTicker("No sane forward route for a race here. Try facing open road.", false, false);
                            CancelPending();
                            pendingAttempts = 0;
                            return;
                        }
                        // Reroll: pick a fresh finish, re-point the blip, and
                        // re-arm for its GPS route.
                        try
                        {
                            var pcR = Game.Player.Character;
                            Vector3 oR = pcR.IsInVehicle() ? pcR.CurrentVehicle.Position : pcR.Position;
                            Vector3 hR = pcR.IsInVehicle() ? pcR.CurrentVehicle.ForwardVector : new Vector3(0f, 1f, 0f);
                            if (FinishPicker.TryPick(oR, hR, cfg.MinDistance, cfg.MaxDistance, out var spot2))
                            {
                                pendingFinish = spot2;
                                pendingSince = Game.GameTime;
                                try { finishBlip?.Delete(); } catch { }
                                try
                                {
                                    finishBlip = World.CreateBlip(pendingFinish);
                                    finishBlip.Sprite = BlipSprite.Standard;
                                    finishBlip.Color = BlipColor.Yellow;
                                    finishBlip.IsShortRange = false;
                                    finishBlip.ShowRoute = true;
                                    finishBlip.Name = "Race Finish";
                                }
                                catch { }
                                try { finishCp?.Delete(); } catch { }
                                try
                                {
                                    finishCp = World.CreateCheckpoint(
                                        CheckpointIcon.CylinderCheckerboard,
                                        pendingFinish,
                                        pendingFinish + new Vector3(0f, 0f, 10f),
                                        cfg.FinishRadius,
                                        Color.FromArgb(220, 255, 210, 0));
                                }
                                catch { }
                                return;
                            }
                        }
                        catch { }
                        Notification.PostTicker("No sane forward route for a race here. Try facing open road.", false, false);
                        CancelPending();
                        pendingAttempts = 0;
                        return;
                    }
                    // Release with whatever route exists (GPS preferred); the
                    // Brain logs src + upgrades cleanly if this was a fallback.
                    oppVehicle = pendingOppVehicle;
                    oppDriver = pendingOppDriver;
                    finish = pendingFinish;
                    activeStyle = pendingStyle;
                    activeProfile = pendingProfile;
                    hasPending = false;
                    pendingOppVehicle = null;
                    pendingOppDriver = null;
                    pendingAttempts = 0;
                    StartRaceNow(timedOut && !gpsReady ? "fallback-timeout" : "gps-ready");
                }
                return;
            }

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

            pendingOppVehicle = vehicle;
            pendingOppDriver = driver;
            pendingFinish = spot;
            pendingStyle = cfg.ResolveDrivingStyle();
            pendingProfile = cfg.ResolveDriverProfile();
            pendingSince = Game.GameTime;
            pendingAttempts = 0;
            hasPending = true;

            try
            {
                finishBlip?.Delete();
                finishBlip = World.CreateBlip(pendingFinish);
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
                    pendingFinish,
                    pendingFinish + new Vector3(0f, 0f, 10f),
                    cfg.FinishRadius,
                    Color.FromArgb(220, 255, 210, 0));
            }
            catch
            {
            }
            // Do NOT Start yet: arming branch above releases once the GPS
            // route exists (or 1.5s timeout), so Build() starts on GPS.
        }

        private void StartRaceNow(string armReason)
        {
            activeStyle = pendingStyle;
            // activeProfile already set from pending in the arming release;
            // keep fields consistent if StartRaceNow is called directly.
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
                try { telemetry?.Event(0, "ARM", $"{armReason}"); } catch { }
                brain.Start(oppDriver, oppVehicle, finish, cfg.AiCruiseSpeed, activeStyle,
                    activeProfile, telemetry, cfg.RefreshIntervalMs, cfg.StuckTimeoutMs,
                    cfg.UseDirectActuator(), cfg.DebugViz);
                // Final start-line gate: the temp-route check above used the
                // arming-time pose; the rival may have crept. If the BUILT
                // route is sideways from the actual start pose, do not race
                // it — reject instead of recovering from a bad setup.
                try
                {
                    string sr;
                    if (!brain.IsStartPoseValid(out sr))
                    {
                        try { telemetry?.Event(0, "ARM_REJECT", $"built-route-invalid;{sr}"); } catch { }
                        try { telemetry?.Close(); } catch { }
                        telemetry = null;
                        try { brain.Stop(); } catch { }
                        try { finishBlip?.Delete(); } catch { }
                        try { finishCp?.Delete(); } catch { }
                        finishBlip = null;
                        finishCp = null;
                        cooldownUntil = Game.GameTime + cfg.CooldownMs;
                        state = RaceState.Cooldown;
                        Notification.PostTicker("No sane forward route for a race here. Try facing open road.", false, false);
                        return;
                    }
                }
                catch { }
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

        private bool ValidateRouteForRival(Vehicle rivalVeh, Vector3 dest, out string reason)
        {
            reason = "no-vehicle";
            try
            {
                if (rivalVeh == null || !rivalVeh.Exists()) return false;
                Vector3 rp = rivalVeh.Position;
                float rh = 0f;
                try { rh = rivalVeh.Heading; } catch { rh = 0f; }
                var probe = new RaceRoute();
                probe.Build(rp, dest);
                // Update once so AlongS/HeadingError reflect the rival pose.
                try
                {
                    float half = 7f;
                    probe.Update(rp, rh, 0f, Game.GameTime, half);
                }
                catch { }
                bool ok = probe.ValidateStart(rp, rh, out reason);
                try
                {
                    reason = $"src={probe.Source};pts={probe.Points.Count};len={probe.TotalLength:F0};s={probe.AlongS:F0};headErr={probe.HeadingErrorDeg:F0};dist={probe.DistToRoute:F1};" + reason;
                }
                catch { }
                return ok;
            }
            catch (Exception ex)
            {
                try { reason = "exc:" + ex.Message; } catch { }
                return false;
            }
        }

        private void CancelPending()
        {
            hasPending = false;
            pendingOppVehicle = null;
            pendingOppDriver = null;
            pendingAttempts = 0;
            try { finishBlip?.Delete(); } catch { }
            try { finishCp?.Delete(); } catch { }
            finishBlip = null;
            finishCp = null;
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
                    msg = $"~y~RACE~s~  You: {(int)dYou}m  Rival: {(int)dOpp}m  {lead} ~s~[{brain.TacticalName} {brain.TargetSpeed:F0} {brain.RouteSource} {brain.ActuatorName}]";
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
            hasPending = false;
            pendingOppVehicle = null;
            pendingOppDriver = null;
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
