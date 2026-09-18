using System;
using GTA;
using GTA.Math;

namespace StreetRacing.Race
{
    /// Phase-1 hardware/control validation (Direct layer, isolated).
    ///
    /// Deliberately has: no perception, no tactics, no obstacle planner, no
    /// recovery state machine, no candidate selection, no route.
    ///
    /// Staged script on a straight road (rival vehicle from standstill):
    ///   Stage Throttle: steer=0, full throttle, verify acceleration.
    ///   Stage Coast:    steer=0, zero throttle/brake, verify coast.
    ///   Stage Brake:    steer=0, controlled braking to stop.
    ///   Stage Steer:    fixed steering commands (+/-) at low speed.
    ///
    /// Logs every 100 ms: commanded vs actual throttle/throttle-power, brake,
    /// handbrake, gear/RPM if accessible, speed and acceleration. Do not
    /// proceed to route following until this proves Direct can physically
    /// accelerate, coast, brake and steer a DriveV vehicle.
    internal sealed class DirectDiagBrain
    {
        public enum Stage { Throttle, Coast, Brake, SteerLeft, SteerRight, Done }

        private Ped driver;
        private Vehicle vehicle;
        private RaceTelemetry telemetry;
        private int t0;
        private float diagCruise = 18f;

        private Stage stage = Stage.Throttle;
        private int stageSince;
        private float lastSpeed;
        private Vector3 lastPos = Vector3.Zero;
        private bool hasKin;
        private int lastKinT;
        private int lastLogMs = -100000;
        private int lastTeleMs;

        public bool Running { get; private set; }
        // Explicit completion latch: set once the Done stage has held long
        // enough to log the final result. The lifecycle polls this to end
        // the diagnostic automatically instead of idling as a dead race.
        public bool Finished { get; private set; }
        public string TacticalName => "DIAG_" + stage;
        public float TargetSpeed { get; private set; }
        public string SpeedLimit => "Diag:" + stage;
        public float ActualSpeed { get; private set; }
        public float FinishGap => 0f;
        public float Progress01 => 0f;
        public bool RouteLost => false;
        public float LookaheadM => 0f;
        public string RouteSource => "Diag/NoRoute";
        public string ActuatorName => "DirectDiag";

        public void Start(Ped driver, Vehicle vehicle, RaceTelemetry telemetry, float diagCruise)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.telemetry = telemetry;
            this.diagCruise = diagCruise > 1f ? diagCruise : 18f;
            t0 = Game.GameTime;
            stage = Stage.Throttle;
            stageSince = t0;
            hasKin = false;
            lastSpeed = 0f;
            lastKinT = t0;
            lastLogMs = -100000;
            lastTeleMs = 0;
            TargetSpeed = this.diagCruise;
            ActualSpeed = 0f;
            Running = true;
            Finished = false;
            try
            {
                try { driver.IsPersistent = true; } catch { }
                try { vehicle.IsPersistent = true; } catch { }
                try { driver.Task.ClearAll(); } catch { }
                try
                {
                    if (!vehicle.IsEngineRunning) vehicle.IsEngineRunning = true;
                }
                catch { }
                try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                telemetry?.Event(0, "DIAG_START", $"cruise={this.diagCruise:F0};stages=Throttle(6s)->Coast(2s)->Brake(stop)->SteerL(3s)->SteerR(3s)->Done");
                LogHw(0, "init");
            }
            catch { }
        }

        public bool Valid()
        {
            try { return Running && vehicle != null && vehicle.Exists() && driver != null && driver.Exists() && !driver.IsDead; }
            catch { return false; }
        }

        public void Stop()
        {
            Running = false;
            try
            {
                if (vehicle != null && vehicle.Exists())
                {
                    try { vehicle.Throttle = 0f; } catch { }
                    try { vehicle.ThrottlePower = 0f; } catch { }
                    try { vehicle.BrakePower = 1f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                    vehicle.IsPersistent = false;
                }
            }
            catch { }
            try { telemetry?.Event(Game.GameTime - t0, "DIAG_STOP", $"stage={stage}"); } catch { }
        }

        public void OnTick()
        {
            if (!Running) return;
            int now = Game.GameTime;
            int t = now - t0;
            float egoSpeed = 0f;
            try { egoSpeed = vehicle.Speed; } catch { return; }
            ActualSpeed = egoSpeed;
            if (!hasKin)
            {
                hasKin = true;
                lastSpeed = egoSpeed;
                try { lastPos = vehicle.Position; } catch { }
                lastKinT = now;
                return;
            }

            float heldS = (now - stageSince) / 1000f;
            // Stage transitions (time- or speed-gated).
            switch (stage)
            {
                case Stage.Throttle:
                    TargetSpeed = diagCruise;
                    Apply(1f, 1f, 0f, 0f, false);
                    if (heldS > 6f || egoSpeed >= diagCruise)
                        Advance(Stage.Coast, now, t, $"thr-done spd={egoSpeed:F1} held={heldS:F1}s");
                    break;
                case Stage.Coast:
                    TargetSpeed = egoSpeed;
                    Apply(0f, 0f, 0f, 0f, false);
                    if (heldS > 2f)
                        Advance(Stage.Brake, now, t, $"coast-done spd={egoSpeed:F1}");
                    break;
                case Stage.Brake:
                    TargetSpeed = 0f;
                    Apply(0f, 0f, 0.8f, 0f, false);
                    if (egoSpeed < 0.5f && heldS > 1f)
                        Advance(Stage.SteerLeft, now, t, "stopped");
                    else if (heldS > 8f)
                        Advance(Stage.SteerLeft, now, t, $"brake-timeout spd={egoSpeed:F1}");
                    break;
                case Stage.SteerLeft:
                    TargetSpeed = 5f;
                    // Fixed steering command to verify lateral authority.
                    Apply(0.4f, 0.4f, 0f, 12f, false);
                    if (heldS > 3f)
                        Advance(Stage.SteerRight, now, t, "steerL-done");
                    break;
                case Stage.SteerRight:
                    TargetSpeed = 5f;
                    Apply(0.4f, 0.4f, 0f, -12f, false);
                    if (heldS > 3f)
                        Advance(Stage.Done, now, t, "steerR-done");
                    break;
                case Stage.Done:
                    TargetSpeed = 0f;
                    Apply(0f, 0f, 1f, 0f, false);
                    // Explicit completion: hold the stopped state briefly so
                    // the final telemetry lands, then latch Finished. Control
                    // outputs above are unchanged; this only signals the
                    // lifecycle to end the diagnostic automatically.
                    if (heldS > 2f && !Finished)
                    {
                        Finished = true;
                        try { telemetry?.Event(t, "DIAG_DONE", $"all-stages-complete;spd={egoSpeed:F1}"); } catch { }
                        try { LogHw(t, "done"); } catch { }
                    }
                    break;
            }

            if (now - lastLogMs >= 100)
            {
                lastLogMs = now;
                LogHw(t, stage.ToString());
            }
            if (now - lastTeleMs >= 100)
            {
                lastTeleMs = now;
                // Keep the shared CSV alive during diag (route cols zeroed).
                try
                {
                    telemetry?.Sample(t, 0, "DIAG_" + stage,
                        0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f,
                        TargetSpeed, egoSpeed, "Diag", 0f, 0f, 0f, 0,
                        999f, 999f, 0f, 0f, 0, 0, "NONE", 0, 0f,
                        "Diag", 0f, 0f, 0f, 0f, 999f, -1f, 0f, "DirectDiag", "",
                        99f, 0f, -1, 0f, 0f, -1, "", -1f, 999f, 0,
                        0f, 0f, 0f, TargetSpeed, 0f, 0f, 0f, 0f, 0f);
                }
                catch { }
            }

            lastSpeed = egoSpeed;
            lastKinT = now;
            try { lastPos = vehicle.Position; } catch { }
        }

        private void Advance(Stage next, int now, int t, string why)
        {
            try { telemetry?.Event(t, "DIAG_STAGE", $"{stage}->{next};{why}"); } catch { }
            stage = next;
            stageSince = now;
            try { LogHw(t, $"enter-{next}"); } catch { }
        }

        private void Apply(float thr, float thrPow, float brk, float steerDeg, bool handbrake)
        {
            try
            {
                try { vehicle.SteeringAngle = steerDeg; } catch { }
                try { vehicle.SteeringScale = 1f; } catch { }
                try { vehicle.Throttle = thr; } catch { }
                try { vehicle.ThrottlePower = thrPow; } catch { }
                try { vehicle.BrakePower = brk; } catch { }
                try { vehicle.IsHandbrakeForcedOn = handbrake; } catch { }
            }
            catch { }
        }

        private void LogHw(int t, string ctx)
        {
            try
            {
                float cmdThr = 0f;
                float actThr = 0f;
                float actPow = 0f;
                float actBrk = 0f;
                string gear = "?";
                string rpm = "?";
                string eng = "?";
                string hb = "?";
                float spd = 0f;
                try { spd = vehicle.Speed; } catch { }
                try { actThr = vehicle.Throttle; } catch { }
                try { actPow = vehicle.ThrottlePower; } catch { }
                try { actBrk = vehicle.BrakePower; } catch { }
                try { gear = vehicle.CurrentGear.ToString(); } catch { }
                try { rpm = vehicle.CurrentRPM.ToString("F2"); } catch { }
                try { eng = vehicle.IsEngineRunning ? "on" : "off"; } catch { }
                // Commanded values by stage (what we asked for this tick).
                switch (stage)
                {
                    case Stage.Throttle: cmdThr = 1f; break;
                    case Stage.SteerLeft:
                    case Stage.SteerRight: cmdThr = 0.4f; break;
                    default: cmdThr = 0f; break;
                }
                float acc = 0f;
                try
                {
                    float dt = (Game.GameTime - lastKinT) / 1000f;
                    if (dt > 0.01f && dt < 0.6f) acc = (spd - lastSpeed) / dt;
                }
                catch { }
                telemetry?.Event(t, "DIAG",
                    $"ctx={ctx};stage={stage};cmdThr={cmdThr:F2};actThr={actThr:F2};thrPow={actPow:F2};"
                    + $"brk={actBrk:F2};hb={hb};gear={gear};rpm={rpm};eng={eng};spd={spd:F1};acc={acc:F1}");
            }
            catch { }
        }
    }
}
