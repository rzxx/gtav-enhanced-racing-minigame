using System;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing.Control
{
    /// Real direct controller: pure-pursuit steering + longitudinal PID that
    /// writes steering/throttle/brake every tick. The planner architecture
    /// above it is UNCHANGED — it consumes the same (aim point, target speed)
    /// the GTA servo gets, but executes the trajectory itself instead of
    /// asking GTA pathfinding to replan it.
    ///
    /// Status: implemented, selectable via StreetRacing.ini Actuator=Direct,
    /// default remains GtaDriver(experiment) until controlled tests answer:
    /// "Can GTA's driver accurately follow our chosen path and speed under
    /// DriveV?" If path-following error stays large under GtaDriver, flip the
    /// default here. Direct control under DriveV physics still needs in-game
    /// gain validation (steering authority falls with speed; throttle/brake
    /// mapping varies by car) — start with Balanced + low cruise and read the
    /// PathFollowingError telemetry before raising pace.
    internal sealed class DirectActuator : IVehicleActuator
    {
        private Ped driver;
        private Vehicle vehicle;
        private int stuckMs;
        private int refreshMs;

        public Vector3 CurrentAim { get; private set; } = Vector3.Zero;
        public float CurrentCruise { get; private set; }
        public int CurrentStyle { get; private set; }
        public bool HasPlan { get; private set; }
        public int ReissueCount { get; private set; }
        public string LastReason { get; private set; } = "";
        public string ActuatorName => "Direct";
        public PathFollowingError LastError { get; private set; } = new PathFollowingError();

        // Longitudinal state (simple PI + anti-windup via clamp).
        private float speedInt;

        public void Attach(Ped driver, Vehicle vehicle, float cruise, int style, int refreshMs, int stuckMs)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.CurrentCruise = cruise;
            this.CurrentStyle = style;
            this.refreshMs = refreshMs;
            this.stuckMs = stuckMs;

            driver.IsPersistent = true;
            vehicle.IsPersistent = true;

            Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, true);
            Function.Call(Hash.SET_PED_KEEP_TASK, driver, true);
            Function.Call(Hash.SET_DRIVER_ABILITY, driver, 1.0f);
            Function.Call(Hash.SET_DRIVER_AGGRESSIVENESS, driver, 1.0f);
            Function.Call(Hash.SET_DRIVER_RACING_MODIFIER, driver, 1.0f);

            try { driver.Task.ClearAll(); } catch { }
            HasPlan = false;
            ReissueCount = 0;
            speedInt = 0f;
        }

        public void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason)
        {
            CurrentAim = aimPoint;
            CurrentCruise = Math.Max(0f, targetSpeed);
            CurrentStyle = style;
            LastReason = reason ?? "";
            HasPlan = true;
        }

        public void Clear()
        {
            HasPlan = false;
            try
            {
                if (vehicle != null && vehicle.Exists())
                {
                    vehicle.Throttle = 0f;
                    try { vehicle.BrakePower = 0f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                }
            }
            catch { }
        }

        public bool Valid()
        {
            return driver != null && driver.Exists() && !driver.IsDead
                && vehicle != null && vehicle.Exists() && !vehicle.IsDead;
        }

        public void Stop()
        {
            try
            {
                if (vehicle != null && vehicle.Exists())
                {
                    try { vehicle.Throttle = 0f; } catch { }
                    try { vehicle.BrakePower = 1f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                    vehicle.IsPersistent = false;
                }
                if (driver != null && driver.Exists())
                {
                    try { driver.Task.ClearAll(); } catch { }
                    Function.Call(Hash.SET_PED_KEEP_TASK, driver, false);
                    Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, false);
                    driver.IsPersistent = false;
                }
            }
            catch { }
        }

        public bool OnTick(bool forceReissue, string tacticalName)
        {
            if (!HasPlan || !Valid()) return false;
            Vector3 egoPos;
            float egoHeading;
            float egoSpeed;
            try
            {
                egoPos = vehicle.Position;
                egoHeading = vehicle.Heading;
                egoSpeed = vehicle.Speed;
            }
            catch { return false; }

            // --- Lateral: pure pursuit to the aim point.
            var to = new Vector3(CurrentAim.X - egoPos.X, CurrentAim.Y - egoPos.Y, 0f);
            float distToAim = RaceMath.FlatLength(to);
            float desiredHeading = distToAim > 1f
                ? RaceMath.HeadingFromVector(RaceMath.FlatNormalize(to))
                : egoHeading;
            // Signed error: + = need to turn left.
            float headErr = RaceMath.HeadingDiffDeg(desiredHeading, egoHeading);

            // Speed-scheduled gain: full authority at crawl, calmer at pace.
            // SteeringAngle units in SHVDN are degrees at the wheels; clamp hard.
            float steerDeg = RaceMath.Clamp(headErr * 1.2f, -32f, 32f);
            if (egoSpeed > 25f) steerDeg = RaceMath.Clamp(headErr * 0.7f, -18f, 18f);
            else if (egoSpeed > 15f) steerDeg = RaceMath.Clamp(headErr * 0.9f, -24f, 24f);

            // Reverse logic: aim behind us at crawl -> back up with inverted steer.
            bool reversing = Math.Abs(headErr) > 130f && egoSpeed < 4f && distToAim > 4f;
            if (reversing) steerDeg = -steerDeg;

            try { vehicle.SteeringAngle = steerDeg; } catch { }
            try { vehicle.SteeringScale = 1f; } catch { }

            // --- Longitudinal: PI on speed error, mapped to throttle/brake.
            float speedErr = CurrentCruise - egoSpeed;
            float dt = 0.05f; // 20 Hz script tick
            speedInt = RaceMath.Clamp(speedInt + speedErr * dt, -6f, 6f);
            float u = speedErr * 0.35f + speedInt * 0.12f; // + = need throttle

            // Crawl recovery: stopped far from aim with a plan -> launch.
            bool stalled = egoSpeed < 1.5f && distToAim > 8f && CurrentCruise > 3f;

            try
            {
                if (reversing)
                {
                    vehicle.Throttle = -0.6f;
                    try { vehicle.BrakePower = 0f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                }
                else if (u >= 0f)
                {
                    float thr = RaceMath.Clamp(u, stalled ? 0.8f : 0f, 1f);
                    if (stalled && thr < 0.8f) thr = 0.8f;
                    vehicle.Throttle = thr;
                    try { vehicle.BrakePower = 0f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                    // Brakes below ~35 m/s full-throttle launch control-ish: keep simple.
                }
                else
                {
                    vehicle.Throttle = 0f;
                    float brk = RaceMath.Clamp(-u, 0.15f, 1f);
                    // Hard stop when the plan wants ~0 and we still roll.
                    if (CurrentCruise < 0.5f && egoSpeed > 1f) brk = 1f;
                    try { vehicle.BrakePower = brk; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = CurrentCruise < 0.5f && egoSpeed < 3f && distToAim < 6f; }
                    catch { }
                }
            }
            catch { }

            ReissueCount++; // counts control applications (vs GTA repaths)
            return true;
        }

        public void UpdatePathError(Vector3 egoPos, float egoHeadingDeg, float egoSpeed,
            RaceRoute route, TrajectoryCandidate chosen, bool hasChosen, float targetSpeed)
        {
            var e = new PathFollowingError { Valid = false };
            try
            {
                if (route == null || !route.Built) { LastError = e; return; }
                float desiredHeading = route.HeadingAtS(route.AlongS + 8f);
                float distToPath = 999f;
                if (hasChosen && chosen.Path != null && chosen.Path.Count >= 2)
                {
                    var p0 = chosen.Path[0];
                    var p1 = chosen.Path[Math.Min(2, chosen.Path.Count - 1)];
                    var d = new Vector3(p1.X - p0.X, p1.Y - p0.Y, 0f);
                    if (RaceMath.FlatLength(d) > 0.5f)
                        desiredHeading = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(d));
                    float best = float.MaxValue;
                    for (int i = 0; i < chosen.Path.Count - 1; i++)
                    {
                        var pr = RaceMath.ProjectOnSegment(egoPos, chosen.Path[i], chosen.Path[i + 1]);
                        if (pr.Dist < best) best = pr.Dist;
                    }
                    distToPath = best;
                }
                e.Valid = true;
                e.LateralErrM = route.Lateral;
                e.HeadingErrDeg = -RaceMath.HeadingDiffDeg(desiredHeading, egoHeadingDeg);
                e.SpeedErrMps = targetSpeed - egoSpeed;
                e.DistToPathM = distToPath;
            }
            catch { e.Valid = false; }
            LastError = e;
        }
    }

    /// Retired seam placeholder. Use DirectActuator (Actuator=Direct) instead.
    [System.Obsolete("Use Control.DirectActuator.")]
    internal sealed class DirectActuatorStub : IVehicleActuator
    {
        public Vector3 CurrentAim { get; private set; } = Vector3.Zero;
        public float CurrentCruise { get; private set; }
        public int CurrentStyle { get; private set; }
        public bool HasPlan { get; private set; }
        public int ReissueCount => 0;
        public string ActuatorName => "DirectStub(retired)";
        public string LastReason => "";
        public PathFollowingError LastError => new PathFollowingError();
        public void Attach(GTA.Ped driver, GTA.Vehicle vehicle, float cruise, int style, int refreshMs, int stuckMs) { }
        public void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason)
        {
            CurrentAim = aimPoint;
            CurrentCruise = targetSpeed;
            CurrentStyle = style;
            HasPlan = true;
            throw new NotImplementedException("DirectActuatorStub is retired; use Control.DirectActuator.");
        }
        public void Clear() { HasPlan = false; }
        public bool Valid() => false;
        public void Stop() { }
        public bool OnTick(bool forceReissue, string tacticalName) => false;
        public void UpdatePathError(Vector3 egoPos, float egoHeadingDeg, float egoSpeed, RaceRoute route, TrajectoryCandidate chosen, bool hasChosen, float targetSpeed) { }
    }
}
