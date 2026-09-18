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
    /// Race lifecycle (challenge -> arming -> racing -> cooldown). Driving
    /// intelligence lives in the brains; this class only owns the trigger,
    /// GPS acquisition, finish marker, win/lose checks and HUD.
    ///
    /// Startup contract (this pass, Simple GPS-only):
    ///   HONK -> pick opponent + finish -> create blip/GPS route -> ARMING
    ///   -> repeatedly extract GPS geometry (RETRY on not-ready)
    ///   -> ONE valid RouteSnapshot -> validate SAME snapshot locally
    ///   -> if valid: start SimpleBrain FROM that snapshot (no 2nd sampling)
    ///   -> if locally invalid: reroll finish (new blip, new acquisition)
    ///   -> if GPS never arrives: explicit cancel (never FallbackWalk).
    /// A failed sample means "not ready yet", never "route invalid".
    public class StreetRacing : Script
    {
        private readonly StreetRacingConfig cfg;
        private RaceState state = RaceState.Idle;
        // Collapsed brains: Simple (default dumb follower), Diag (Phase-1
        // hardware probe), Legacy (old full stack, comparison only).
        private readonly RaceBrain legacyBrain = new RaceBrain();
        private readonly SimpleBrain simpleBrain = new SimpleBrain();
        private readonly DirectDiagBrain diagBrain = new DirectDiagBrain();

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

        // --- Explicit arming state (replaces Idle+hasPending booleans).
        // Same opponent/finish/blip are preserved across acquisition retries.
        // The blip is NEVER deleted because one extraction attempt failed.
        private Vehicle armingOppVehicle;
        private Ped armingOppDriver;
        private Vector3 armingFinish = Vector3.Zero;
        private int armingStyle;
        private DriverProfile armingProfile;
        private int armingBeginMs;
        private int armingSinceMs;
        private int armingLastSampleMs;
        private int armingSampleNotBeforeMs;
        private int armingSampleAttempts;
        private int armingRerolls;
        private int armingLastReassertMs;
        private RouteSnapshot acceptedSnapshot;

        // Acquisition tuning: Simple is GPS-only with a patient window.
        // Timeout means "GPS still isn't available" (cancel), NEVER permission
        // to continue on FallbackWalk/StraightFallback.
        private const int SimpleSampleIntervalMs = 150;
        // GTA rebuilds the active GPS route asynchronously after a new routed
        // blip is created. Never sample on the next script tick: telemetry
        // showed stale/mismatched routes being consumed ~55 ms after rerolls.
        private const int SimpleInitialRouteSettleMs = 250;
        private const int SimpleRerollRouteSettleMs = 350;
        private const int SimpleAcquireTimeoutMs = 8000;
        private const int LegacyAcquireTimeoutMs = 1500;
        private const int MaxArmingRerolls = 3;

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
                if (cfg.UseDiagDriver() || diagBrain.Running)
                {
                    EndDiag("Diagnostic cancelled.");
                }
                else
                {
                    EndRace("Race cancelled.");
                }
            }
            else if (e.KeyCode == cfg.CancelKey && state == RaceState.Arming)
            {
                CancelArming("player-cancel");
            }
        }

        private void OnTick(object sender, EventArgs e)
        {
            switch (state)
            {
                case RaceState.Idle:
                    TickIdle();
                    break;
                case RaceState.Arming:
                    TickArming();
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
            // DirectDiag isolation: its own minimal lifecycle with no route,
            // no FinishPicker, no GPS, no planner, no recovery, no start
            // validation. Never fall through to the normal arming pipeline
            // below while diag is selected.
            if (cfg.UseDiagDriver())
            {
                TickIdleDiag();
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

            // Finish ahead of the RIVAL (not the player): the AI must drive
            // it, and validation is rival-centric. Player and rival are close
            // and same-direction (picker gates dot>0.5), so this is also
            // roughly ahead of the player. Using player pose here caused
            // systematic START_VALID failures when headings differed.
            Vector3 origin;
            Vector3 heading;
            try
            {
                origin = vehicle.Position;
                heading = vehicle.ForwardVector;
                if (RaceMath.FlatLength(new Vector3(heading.X, heading.Y, 0f)) < 0.05f)
                {
                    var player0 = Game.Player.Character;
                    origin = player0.CurrentVehicle.Position;
                    heading = player0.CurrentVehicle.ForwardVector;
                }
            }
            catch
            {
                var player = Game.Player.Character;
                origin = player.CurrentVehicle.Position;
                heading = player.CurrentVehicle.ForwardVector;
            }
            if (!FinishPicker.TryPick(origin, heading, cfg.MinDistance, cfg.MaxDistance, out var spot))
            {
                Notification.PostTicker("No road ahead for a finish line. Try facing open road.", false, false);
                return;
            }

            BeginArming(vehicle, driver, spot);
        }

        private void BeginArming(Vehicle vehicle, Ped driver, Vector3 spot)
        {
            int now = Game.GameTime;
            armingOppVehicle = vehicle;
            armingOppDriver = driver;
            armingFinish = spot;
            try { armingStyle = cfg.ResolveDrivingStyle(); } catch { armingStyle = 0; }
            try { armingProfile = cfg.ResolveDriverProfile(); } catch { armingProfile = null; }
            armingBeginMs = now;
            armingSinceMs = now;
            armingLastSampleMs = now;
            armingSampleNotBeforeMs = now + SimpleInitialRouteSettleMs;
            armingSampleAttempts = 0;
            armingRerolls = 0;
            armingLastReassertMs = now;
            acceptedSnapshot = null;

            CreateFinishMarkers(armingFinish);
            try { Notification.PostTicker("Challenge found. Calculating GPS route...", false, false); } catch { }

            // Lightweight arming telemetry immediately: startup failures must
            // be observable even when no race ever starts.
            try
            {
                try { telemetry?.Close(); } catch { }
                telemetry = cfg.TelemetryEnabled ? new RaceTelemetry(armingBeginMs, armingStyle, armingProfile != null ? armingProfile.Name : "?") : null;
            }
            catch (Exception ex)
            {
                try { telemetry = null; } catch { }
                try { GTA.UI.Notification.PostTicker("Telemetry failed: " + ex.Message, false, false); } catch { }
            }
            try
            {
                int oppHandle = -1;
                float oppHead = 0f;
                Vector3 oppPos = Vector3.Zero;
                try { if (vehicle != null && vehicle.Exists()) { oppHandle = vehicle.Handle; oppPos = vehicle.Position; oppHead = vehicle.Heading; } } catch { }
                int drvHandle = -1;
                try { if (driver != null && driver.Exists()) drvHandle = driver.Handle; } catch { }
                Vector3 plPos = Vector3.Zero;
                float plHead = 0f;
                float plOppDist = -1f;
                float plOppHeadDiff = 999f;
                try
                {
                    var pc = Game.Player.Character;
                    if (pc != null && pc.Exists() && pc.IsInVehicle())
                    {
                        var pv = pc.CurrentVehicle;
                        plPos = pv.Position;
                        plHead = pv.Heading;
                        plOppDist = RaceMath.FlatDistance(plPos, oppPos);
                        plOppHeadDiff = RaceMath.HeadingDiffDeg(oppHead, plHead);
                    }
                }
                catch { }
                telemetry?.Event(0, "ARM_BEGIN", $"mode={cfg.DriverMode};oppVeh={oppHandle};oppDrv={drvHandle};oppPos=({oppPos.X:F0},{oppPos.Y:F0});oppHead={oppHead:F0};plPos=({plPos.X:F0},{plPos.Y:F0});plHead={plHead:F0};plOppDist={plOppDist:F0};plOppHeadDiff={plOppHeadDiff:F0};finish=({spot.X:F0},{spot.Y:F0});style={armingStyle};profile={(armingProfile != null ? armingProfile.Name : "?")}");
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(0, "ARM_EXC", "begin:" + ex.Message); } catch { }
            }
            state = RaceState.Arming;
        }

        private void CreateFinishMarkers(Vector3 spot)
        {
            try
            {
                try { finishBlip?.Delete(); } catch { }
                finishBlip = World.CreateBlip(spot);
                finishBlip.Sprite = BlipSprite.Standard;
                finishBlip.Color = BlipColor.Yellow;
                finishBlip.IsShortRange = false;
                finishBlip.ShowRoute = true;
                finishBlip.Name = "Race Finish";
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_EXC", "blip:" + ex.Message); } catch { }
            }

            try
            {
                try { finishCp?.Delete(); } catch { }
                finishCp = World.CreateCheckpoint(
                    CheckpointIcon.CylinderCheckerboard,
                    spot,
                    spot + new Vector3(0f, 0f, 10f),
                    cfg.FinishRadius,
                    Color.FromArgb(220, 255, 210, 0));
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_EXC", "cp:" + ex.Message); } catch { }
            }
        }

        private int ElapsedArmingMs()
        {
            try { return Game.GameTime - armingBeginMs; } catch { return 0; }
        }

        private void TickArming()
        {
            // Diag never arms; bail out cleanly if the mode flipped mid-arm.
            if (cfg.UseDiagDriver())
            {
                CancelArming("mode-switched-to-diag");
                return;
            }
            // Validate participants still exist (explicit reasons, no silent catch).
            try
            {
                if (armingOppVehicle == null || !armingOppVehicle.Exists()
                    || armingOppDriver == null || !armingOppDriver.Exists() || armingOppDriver.IsDead)
                {
                    CancelArming("opponent-gone");
                    return;
                }
                var pc0 = Game.Player.Character;
                if (pc0 == null || !pc0.Exists() || !pc0.IsInVehicle())
                {
                    CancelArming("player-not-in-vehicle");
                    return;
                }
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_EXC", "validate-participants:" + ex.Message); } catch { }
                CancelArming("validate-exc");
                return;
            }

            if (cfg.UseSimpleDriver())
                TickArmingSimple();
            else
                TickArmingLegacy();
        }

        // --- Simple GPS-only arming: async acquisition, persistent snapshot.
        private void TickArmingSimple()
        {
            int now = Game.GameTime;
            // Keep the yellow route visible while arming (re-assert, never recreate).
            try
            {
                if (finishBlip != null && now - armingLastReassertMs > 1000)
                {
                    armingLastReassertMs = now;
                    finishBlip.ShowRoute = true;
                }
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_EXC", "reassert:" + ex.Message); } catch { }
            }

            // Acquisition timeout: GPS still isn't available -> fail cleanly.
            // NEVER fall through to FallbackWalk/StraightFallback.
            if (now - armingSinceMs > SimpleAcquireTimeoutMs)
            {
                string why = $"gps-timeout attempts={armingSampleAttempts} rerolls={armingRerolls} elapsed={(now - armingSinceMs) / 1000f:F0}s finish=({armingFinish.X:F0},{armingFinish.Y:F0})";
                CancelArming(why);
                return;
            }

            if (now < armingSampleNotBeforeMs)
                return;
            if (now - armingLastSampleMs < SimpleSampleIntervalMs)
                return;
            armingLastSampleMs = now;

            Vector3 rivalPos;
            float rivalHeading;
            try
            {
                rivalPos = armingOppVehicle.Position;
                rivalHeading = armingOppVehicle.Heading;
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_EXC", "rival-pose:" + ex.Message); } catch { }
                return; // retry next tick, same opponent/finish/blip
            }

            RouteSnapshot snap = null;
            string acqDetail = "";
            bool acquired = false;
            try
            {
                acquired = RaceRoute.TryAcquireGpsSnapshot(rivalPos, armingFinish, out snap, out acqDetail);
            }
            catch (Exception ex)
            {
                acquired = false;
                snap = null;
                acqDetail = "acquire-exc:" + ex.Message;
            }

            int t = ElapsedArmingMs();
            if (!acquired || snap == null)
            {
                // Readiness failure: NOT ready yet, NOT invalid. Keep same
                // opponent/finish/blip and retry. Never reroll here.
                armingSampleAttempts++;
                try { telemetry?.Event(t, "GPS_WAIT", $"attempt={armingSampleAttempts};elapsed={(now - armingSinceMs) / 1000f:F1}s;{acqDetail}"); } catch { }
                return;
            }

            // ONE successful snapshot: log it, then validate THE SAME snapshot.
            armingSampleAttempts++;
            try { telemetry?.Event(t, "GPS_SNAPSHOT_OK", $"attempt={armingSampleAttempts};src={snap.Source};pts={snap.Points.Count};len={snap.TotalLength:F0};{acqDetail}"); } catch { }

            string vReason = "";
            bool vOk = false;
            try { vOk = ValidateSnapshotForRival(snap, rivalPos, rivalHeading, out vReason); }
            catch (Exception ex)
            {
                vOk = false;
                vReason = "validate-exc:" + ex.Message;
            }
            try { telemetry?.Event(ElapsedArmingMs(), "START_VALID", $"{(vOk ? "valid" : "INVALID")};{vReason}"); } catch { }

            if (!vOk)
            {
                // Genuinely invalid for this rival/start: reroll destination.
                armingRerolls++;
                if (armingRerolls > MaxArmingRerolls)
                {
                    try { telemetry?.Event(ElapsedArmingMs(), "ARM_REJECT", $"no-sane-forward-route rerolls={armingRerolls};{vReason}"); } catch { }
                    CancelArming($"no-sane-forward-route rerolls={armingRerolls};{vReason}");
                    return;
                }
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_REROLL", $"reason={vReason};reroll={armingRerolls}/{MaxArmingRerolls}"); } catch { }
                RerollArmingFinish(vReason);
                return;
            }

            // Accepted: persist snapshot, hand off WITHOUT second sampling.
            acceptedSnapshot = snap;
            try { telemetry?.Event(ElapsedArmingMs(), "ARM_READY", $"src={snap.Source};pts={snap.Points.Count};len={snap.TotalLength:F0};{vReason}"); } catch { }
            StartRaceNowFromSnapshot(acceptedSnapshot, "gps-ready");
        }

        private bool ValidateSnapshotForRival(RouteSnapshot snap, Vector3 rivalPos, float rivalHeading, out string reason)
        {
            reason = "no-snapshot";
            try
            {
                if (snap == null || !snap.IsGps)
                {
                    reason = $"non-gps src={(snap != null ? snap.Source : "null")};never-fallback";
                    return false;
                }
                var probe = new RaceRoute();
                probe.ImportSnapshot(snap);
                try { probe.Update(rivalPos, rivalHeading, 0f, Game.GameTime, 7f); } catch { }
                bool ok = probe.ValidateStart(rivalPos, rivalHeading, out reason);
                try
                {
                    reason = $"src={probe.Source};pts={probe.Points.Count};len={probe.TotalLength:F0};s={probe.AlongS:F0};egoHead={rivalHeading:F0};routeHead={probe.RouteHeadingDeg:F0};headErr={probe.HeadingErrorDeg:F0};dist={probe.DistToRoute:F1};" + reason;
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

        private void RerollArmingFinish(string why)
        {
            int now = Game.GameTime;
            try
            {
                // Reroll ahead of the RIVAL (same fix as initial pick): the
                // new route must validate rival-centrically.
                Vector3 oR;
                Vector3 hR;
                try
                {
                    oR = armingOppVehicle.Position;
                    hR = armingOppVehicle.ForwardVector;
                    if (RaceMath.FlatLength(new Vector3(hR.X, hR.Y, 0f)) < 0.05f)
                        throw new Exception("rival-fwd-degenerate");
                }
                catch
                {
                    var pcR = Game.Player.Character;
                    oR = pcR.IsInVehicle() ? pcR.CurrentVehicle.Position : pcR.Position;
                    hR = pcR.IsInVehicle() ? pcR.CurrentVehicle.ForwardVector : new Vector3(0f, 1f, 0f);
                }
                if (FinishPicker.TryPick(oR, hR, cfg.MinDistance, cfg.MaxDistance, out var spot2))
                {
                    armingFinish = spot2;
                    armingSinceMs = now;
                    armingLastSampleMs = now;
                    armingSampleNotBeforeMs = now + SimpleRerollRouteSettleMs;
                    armingSampleAttempts = 0;
                    acceptedSnapshot = null;
                    armingLastReassertMs = now;
                    CreateFinishMarkers(armingFinish);
                    try { Notification.PostTicker("Recalculating GPS route...", false, false); } catch { }
                    try { telemetry?.Event(ElapsedArmingMs(), "ARM_REROLL_NEWFINISH", $"finish=({spot2.X:F0},{spot2.Y:F0});settleMs={SimpleRerollRouteSettleMs};prevWhy={why}"); } catch { }
                    return;
                }
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_EXC", "reroll:" + ex.Message); } catch { }
            }
            CancelArming("reroll-pick-failed;" + why);
        }

        private void CancelArming(string reason)
        {
            int t = 0;
            try { t = ElapsedArmingMs(); } catch { }
            try { telemetry?.Event(t, "ARM_CANCEL", reason); } catch { }
            try { telemetry?.Close(); } catch { }
            telemetry = null;
            armingOppVehicle = null;
            armingOppDriver = null;
            acceptedSnapshot = null;
            armingSampleAttempts = 0;
            armingSampleNotBeforeMs = 0;
            armingRerolls = 0;
            try { finishBlip?.Delete(); } catch { }
            try { finishCp?.Delete(); } catch { }
            finishBlip = null;
            finishCp = null;
            // Reason-specific visible message (not always "No GPS route"):
            // gps-timeout/acquire failure vs validation rejection are
            // different problems with different remedies.
            string msg = "No GPS route here yet. Try facing open road.";
            try
            {
                string r = (reason ?? "").ToLowerInvariant();
                if (r.Contains("no-sane-forward-route") || r.Contains("reroll"))
                    msg = "No sane forward route here. Reposition facing open road and honk again.";
                else if (r.Contains("opponent-gone"))
                    msg = "Rival drove off. Honk again to challenge.";
                else if (r.Contains("player-not-in-vehicle"))
                    msg = "Challenge cancelled.";
                else if (r.Contains("player-cancel"))
                    msg = "Challenge cancelled.";
            }
            catch { }
            try { Notification.PostTicker(msg, false, false); } catch { }
            // Short cooldown for arming failures (not a full race cooldown):
            // lets the player retry immediately instead of silent honks.
            try { cooldownUntil = Game.GameTime + 2000; } catch { }
            state = RaceState.Cooldown;
        }

        private void StartRaceNowFromSnapshot(RouteSnapshot snap, string armReason)
        {
            // Normal-race Simple path only. Arming owns acquisition+acceptance;
            // this method assumes the snapshot is accepted (no 2nd sampling,
            // no fallback, no re-validation through new GPS queries).
            if (cfg.UseDiagDriver())
                return;
            if (snap == null || !snap.IsGps)
            {
                CancelArming("accepted-snapshot-invalid src=" + (snap != null ? snap.Source : "null"));
                return;
            }
            oppVehicle = armingOppVehicle;
            oppDriver = armingOppDriver;
            finish = armingFinish;
            activeStyle = armingStyle;
            activeProfile = armingProfile;
            raceStartTime = Game.GameTime;
            lastHudTime = 0;
            wasLeading = true;
            // Reuse arming telemetry (do NOT close/recreate: startup + race
            // share one continuous timeline from ARM_BEGIN).
            int tReady = ElapsedArmingMs();
            try { telemetry?.Event(tReady, "ARM", $"{armReason};mode={cfg.DriverMode};snapshots=1;noSecondGps=1"); } catch { }
            try
            {
                if (cfg.UseSimpleDriver())
                {
                    simpleBrain.StartFromSnapshot(oppDriver, oppVehicle, snap, cfg.AiCruiseSpeed, activeStyle,
                        activeProfile, telemetry, cfg.RefreshIntervalMs, cfg.StuckTimeoutMs,
                        cfg.DebugViz, cfg.SimpleCruise, cfg.EnablePassing, cfg.UseGtaRejoin, armingBeginMs);
                }
                else
                {
                    // Should never happen (Simple path only), but keep Legacy
                    // coherent if config flipped mid-arming.
                    legacyBrain.Start(oppDriver, oppVehicle, finish, cfg.AiCruiseSpeed, activeStyle,
                        activeProfile, telemetry, cfg.RefreshIntervalMs, cfg.StuckTimeoutMs,
                        cfg.UseDirectActuator(), cfg.DebugViz);
                }
                // Defensive assert (in-memory string only, no GPS natives):
                // accepted geometry must still be GPS after import. This is
                // NOT a reroll cycle: on impossible mismatch, cancel loudly.
                try
                {
                    if (cfg.UseSimpleDriver())
                    {
                        string src = "";
                        try { src = simpleBrain.RouteSource ?? "?"; } catch { }
                        bool isGps = false;
                        try { isGps = src.StartsWith("Gps"); } catch { }
                        if (!isGps)
                        {
                            try { telemetry?.Event(ElapsedArmingMs(), "BRAIN_SOURCE_MISMATCH", $"src={src};should-never-happen;noSecondGps=1"); } catch { }
                            try { StopAllBrains(); } catch { }
                            CancelArming($"brain-source-mismatch src={src}");
                            return;
                        }
                    }
                }
                catch (Exception ex)
                {
                    try { telemetry?.Event(ElapsedArmingMs(), "ARM_EXC", "source-assert:" + ex.Message); } catch { }
                }
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "BRAIN_START_FAIL", ex.Message); } catch { }
                EndRace("Rival failed to start. Race over.");
                return;
            }
            // Clear arming (keep blip/cp + telemetry for the race).
            armingOppVehicle = null;
            armingOppDriver = null;
            acceptedSnapshot = null;
            armingSampleAttempts = 0;
            armingSampleNotBeforeMs = 0;
            armingRerolls = 0;
            try { telemetry?.Event(ElapsedArmingMs(), "RACE_START", $"src={snap.Source};pts={snap.Points.Count};len={snap.TotalLength:F0}"); } catch { }
            state = RaceState.Racing;
            Notification.PostTicker("Challenge accepted! First to the ~y~yellow marker~s~ wins.", false, false);
        }

        // --- Legacy arming: preserves the old 1.5s fallback semantics.
        // Simple never reaches here. Legacy allows FallbackWalk after timeout.
        private void TickArmingLegacy()
        {
            int now = Game.GameTime;
            try
            {
                if (finishBlip != null && now - armingLastReassertMs > 1000)
                {
                    armingLastReassertMs = now;
                    finishBlip.ShowRoute = true;
                }
            }
            catch { }
            bool gpsReady = false;
            try { gpsReady = Function.Call<bool>(Hash.GET_GPS_BLIP_ROUTE_FOUND); }
            catch { gpsReady = false; }
            bool timedOut = now - armingSinceMs > LegacyAcquireTimeoutMs;
            if (!(gpsReady || timedOut))
                return;

            string startWhy = "";
            bool startOk = false;
            try { startOk = ValidateRouteForRival(armingOppVehicle, armingFinish, out startWhy); }
            catch (Exception ex) { startOk = false; startWhy = "validate-exc:" + ex.Message; }
            try { telemetry?.Event(ElapsedArmingMs(), "START_VALID", $"{(startOk ? "valid" : "INVALID")};{startWhy}"); } catch { }
            if (!startOk)
            {
                armingRerolls++;
                if (armingRerolls > MaxArmingRerolls)
                {
                    try { telemetry?.Event(ElapsedArmingMs(), "ARM_REJECT", $"no-sane-forward-route rerolls={armingRerolls};{startWhy}"); } catch { }
                    CancelArming($"no-sane-forward-route rerolls={armingRerolls};{startWhy}");
                    return;
                }
                try { telemetry?.Event(ElapsedArmingMs(), "ARM_REROLL", $"reason={startWhy};reroll={armingRerolls}/{MaxArmingRerolls}"); } catch { }
                RerollArmingFinish(startWhy);
                return;
            }
            oppVehicle = armingOppVehicle;
            oppDriver = armingOppDriver;
            finish = armingFinish;
            activeStyle = armingStyle;
            activeProfile = armingProfile;
            armingOppVehicle = null;
            armingOppDriver = null;
            acceptedSnapshot = null;
            StartRaceNowLegacy(timedOut && !gpsReady ? "fallback-timeout" : "gps-ready");
        }

        private void StartRaceNowLegacy(string armReason)
        {
            if (cfg.UseDiagDriver())
                return;
            raceStartTime = Game.GameTime;
            lastHudTime = 0;
            wasLeading = true;
            // Legacy reuses arming telemetry for continuity (no close/recreate).
            try { telemetry?.Event(ElapsedArmingMs(), "ARM", $"{armReason};mode={cfg.DriverMode}"); } catch { }
            try
            {
                legacyBrain.Start(oppDriver, oppVehicle, finish, cfg.AiCruiseSpeed, activeStyle,
                    activeProfile, telemetry, cfg.RefreshIntervalMs, cfg.StuckTimeoutMs,
                    cfg.UseDirectActuator(), cfg.DebugViz);
                try
                {
                    string sr;
                    bool ok = legacyBrain.IsStartPoseValid(out sr);
                    if (!ok)
                    {
                        try { telemetry?.Event(ElapsedArmingMs(), "ARM_REJECT", $"built-route-invalid;{sr}"); } catch { }
                        EndRace("No sane forward route for a race here. Try facing open road.");
                        return;
                    }
                }
                catch { }
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(ElapsedArmingMs(), "BRAIN_START_FAIL", ex.Message); } catch { }
                EndRace("Rival failed to start. Race over.");
                return;
            }
            try { telemetry?.Event(ElapsedArmingMs(), "RACE_START", $"legacy;{armReason}"); } catch { }
            state = RaceState.Racing;
            Notification.PostTicker("Challenge accepted! First to the ~y~yellow marker~s~ wins.", false, false);
        }

        // DirectDiag minimal lifecycle: honk -> immediate start, no route.
        // Deliberately bypasses FinishPicker, finish blip/checkpoint,
        // arming, GPS wait, ValidateRouteForRival and rerolls.
        private void TickIdleDiag()
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

            // Immediately assign the selected opponent and start the probe.
            // No finish, no blip, no checkpoint, no arming.
            oppVehicle = vehicle;
            oppDriver = driver;
            finish = Vector3.Zero;
            activeStyle = cfg.ResolveDrivingStyle();
            activeProfile = cfg.ResolveDriverProfile();
            StartDiagNow();
        }

        private void StartDiagNow()
        {
            // Defensive: diag never owns a finish marker or GPS route, even
            // for one frame. Clear any stale state from a previous mode.
            try { finishBlip?.Delete(); } catch { }
            try { finishCp?.Delete(); } catch { }
            finishBlip = null;
            finishCp = null;
            finish = Vector3.Zero;
            armingOppVehicle = null;
            armingOppDriver = null;
            acceptedSnapshot = null;
            armingSampleAttempts = 0;
            armingSampleNotBeforeMs = 0;
            armingRerolls = 0;
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
                try { telemetry?.Event(0, "ARM", $"diag-direct;mode={cfg.DriverMode}"); } catch { }
                diagBrain.Start(oppDriver, oppVehicle, telemetry, cfg.DiagCruise);
            }
            catch (Exception ex)
            {
                try { telemetry?.Event(0, "BRAIN_START_FAIL", ex.Message); } catch { }
                EndDiag("Diagnostic failed to start.");
                return;
            }
            state = RaceState.Racing;
            Notification.PostTicker("Diagnostic started: throttle/coast/brake/steer probe.", false, false);
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
                try
                {
                    reason = $"src={probe.Source};pts={probe.Points.Count};len={probe.TotalLength:F0};s={probe.AlongS:F0};headErr={probe.HeadingErrorDeg:F0};dist={probe.DistToRoute:F1};" + reason;
                }
                catch { }
                // Legacy allows fallback; Simple never reaches here (strict path above).
                try
                {
                    probe.Update(rp, rh, 0f, Game.GameTime, 7f);
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

        private void TickRacing()
        {
            // DirectDiag isolation: no finish-distance checks, no GPS/blip
            // route reassertion, no route HUD, no finish dependency, no
            // race-route validation while the probe is active.
            if (cfg.UseDiagDriver() || diagBrain.Running)
            {
                TickDiagRacing();
                return;
            }
            var player = Game.Player.Character;
            if (player == null || !player.Exists() || player.IsDead)
            {
                EndRace("You died. Race over.");
                return;
            }
            if (!ActiveValid())
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
                ActiveOnTick();
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
                    msg = $"~y~RACE~s~  You: {(int)dYou}m  Rival: {(int)dOpp}m  {lead} ~s~[{ActiveTactical()} {ActiveTargetSpeed():F0} {ActiveRouteSource()} {ActiveActuator()}]";
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

        // DirectDiag active tick: minimal lifecycle with no route state.
        // No finish reference, no blip/GPS touch, no route HUD.
        private void TickDiagRacing()
        {
            var player = Game.Player.Character;
            if (player == null || !player.Exists() || player.IsDead)
            {
                EndDiag("Diagnostic over: you died.");
                return;
            }
            if (!diagBrain.Valid())
            {
                EndDiag("Diagnostic over: rival out (wrecked / gone).");
                return;
            }
            if (Game.GameTime - raceStartTime > cfg.RaceTimeoutMs)
            {
                EndDiag("Diagnostic timed out.");
                return;
            }

            try
            {
                diagBrain.OnTick();
            }
            catch
            {
            }

            // Explicit completion: the probe latches Finished after the Done
            // stage holds; end automatically instead of idling as a dead race.
            try
            {
                if (diagBrain.Finished)
                {
                    EndDiag("Diagnostic complete: throttle/coast/brake/steer done. See telemetry.");
                    return;
                }
            }
            catch { }

            if (Game.GameTime - lastHudTime > 1000)
            {
                lastHudTime = Game.GameTime;
                string msg;
                try
                {
                    msg = $"~y~DIAG~s~ {diagBrain.TacticalName} tgt {diagBrain.TargetSpeed:F0} m/s act {diagBrain.ActualSpeed:F0} m/s";
                }
                catch
                {
                    msg = "~y~DIAG~s~ running";
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

        private void EndDiag(string message)
        {
            try { telemetry?.Event(Game.GameTime - raceStartTime, "DIAG_END", message); } catch { }
            try
            {
                StopAllBrains();
            }
            catch
            {
            }
            // Defensive: diag never creates these, but clear stale markers
            // if a previous normal race left any behind.
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
            finish = Vector3.Zero;
            oppVehicle = null;
            oppDriver = null;
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

        private void EndRace(string message)
        {
            try
            {
                StopAllBrains();
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
            armingOppVehicle = null;
            armingOppDriver = null;
            acceptedSnapshot = null;
            try
            {
                StopAllBrains();
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

        // --- Collapsed brain dispatch (Simple default, Diag probe, Legacy).
        private void StopAllBrains()
        {
            try { legacyBrain.Stop(); } catch { }
            try { simpleBrain.Stop(); } catch { }
            try { diagBrain.Stop(); } catch { }
        }

        private bool ActiveValid()
        {
            try
            {
                if (cfg.UseDiagDriver()) return diagBrain.Valid();
                if (cfg.UseSimpleDriver()) return simpleBrain.Valid();
                return legacyBrain.Valid();
            }
            catch { return false; }
        }

        private void ActiveOnTick()
        {
            if (cfg.UseDiagDriver()) diagBrain.OnTick();
            else if (cfg.UseSimpleDriver()) simpleBrain.OnTick();
            else legacyBrain.OnTick();
        }

        private string ActiveTactical()
        {
            try
            {
                if (cfg.UseDiagDriver()) return diagBrain.TacticalName;
                if (cfg.UseSimpleDriver()) return simpleBrain.TacticalName;
                return legacyBrain.TacticalName;
            }
            catch { return "?"; }
        }

        private float ActiveTargetSpeed()
        {
            try
            {
                if (cfg.UseDiagDriver()) return diagBrain.TargetSpeed;
                if (cfg.UseSimpleDriver()) return simpleBrain.TargetSpeed;
                return legacyBrain.TargetSpeed;
            }
            catch { return 0f; }
        }

        private string ActiveRouteSource()
        {
            try
            {
                if (cfg.UseDiagDriver()) return diagBrain.RouteSource;
                if (cfg.UseSimpleDriver()) return simpleBrain.RouteSource;
                return legacyBrain.RouteSource;
            }
            catch { return "?"; }
        }

        private string ActiveActuator()
        {
            try
            {
                if (cfg.UseDiagDriver()) return diagBrain.ActuatorName;
                if (cfg.UseSimpleDriver()) return simpleBrain.ActuatorName;
                return legacyBrain.ActuatorName;
            }
            catch { return "?"; }
        }
    }
}
