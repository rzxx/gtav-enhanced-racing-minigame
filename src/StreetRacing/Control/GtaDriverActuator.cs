using System;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing.Control
{
    /// BASELINE / DIAGNOSTIC ONLY — stock GTA driver used as a servo.
    ///
    /// The joint planner + Direct controller is the intended path: it
    /// executes the selected sampled path/speed itself. This actuator still
    /// hands (point, speed) to GTA's own pathfinding, which replans its own
    /// lane-level path with its own curvature/speed model under DriveV and
    /// therefore DOES NOT guarantee execution of our maneuver. Keep it only
    /// to measure that gap (see PathFollowingError) — never as the decision
    /// maker. Persistent perception + joint planning already removed the
    /// flicker that used to force constant DriveTo repaths; re-issues stay
    /// rate-limited (aim moved / speed changed / mode changed / stuck).
    internal sealed class GtaDriverActuator : IVehicleActuator
    {
        private Ped driver;
        private Vehicle vehicle;
        private int stuckMs;

        public Vector3 CurrentAim { get; private set; } = Vector3.Zero;
        public float CurrentCruise { get; private set; }
        public int CurrentStyle { get; private set; }
        public bool HasPlan { get; private set; }
        public int ReissueCount { get; private set; }
        public string LastReason { get; private set; } = "";
        public string ActuatorName => "GtaDriver(baseline)";
        public PathFollowingError LastError { get; private set; } = new PathFollowingError();
        public bool LastTickReissued { get; private set; }

        private int lastRefreshTime;
        private int refreshMs;
        private Vector3 lastPos = Vector3.Zero;
        private int lastMoveTime;

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

            lastPos = vehicle.Position;
            lastMoveTime = Game.GameTime;
            HasPlan = false;
            ReissueCount = 0;
        }

        public void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason)
        {
            CurrentAim = aimPoint;
            CurrentCruise = targetSpeed;
            CurrentStyle = style;
            LastReason = reason ?? "";
            HasPlan = true;
        }

        public void SetManeuver(ManeuverCommand cmd)
        {
            // Baseline ignores the sampled path: it servo-tracks the distant
            // aim + cruise only (diagnostic against the SELECTED maneuver via
            // UpdatePathError's chosen path).
            try
            {
                CurrentAim = cmd.AimPoint;
                CurrentCruise = Math.Max(0f, cmd.TargetSpeed);
                CurrentStyle = cmd.Style;
                LastReason = cmd.Reason ?? "";
                HasPlan = true;
            }
            catch { }
        }

        public void Clear() { HasPlan = false; }

        /// Returns true when a full DriveTo re-issue happened this tick.
        public bool OnTick(bool forceReissue, string tacticalName)
        {
            LastTickReissued = false;
            if (!HasPlan || !Valid()) return false;
            int now = Game.GameTime;

            if (now - lastRefreshTime > refreshMs)
            {
                try
                {
                    Function.Call(Hash.SET_DRIVE_TASK_CRUISE_SPEED, driver, CurrentCruise);
                    Function.Call(Hash.SET_DRIVE_TASK_DRIVING_STYLE, driver, CurrentStyle);
                }
                catch { }
                lastRefreshTime = now;
            }

            bool needIssue = forceReissue || ReissueCount == 0;
            try
            {
                if (!needIssue)
                {
                    // Re-issue only on meaningful plan change (hysteresis).
                    float aimMoved = RaceMath.FlatDistance(vehicle.Position, CurrentAim) < 0.5f
                        ? 999f : AimMovedSinceIssue();
                    if (aimMoved > 18f) needIssue = true;
                }
            }
            catch { }

            // Stuck: crawling and barely moved -> fresh path to the same aim.
            bool crawling = false;
            try { crawling = vehicle.Speed < 1.5f; } catch { }
            try
            {
                if (FinishPicker.FlatDistance(vehicle.Position, lastPos) > 4f)
                {
                    lastPos = vehicle.Position;
                    lastMoveTime = now;
                }
                else if (crawling && now - lastMoveTime > stuckMs)
                {
                    needIssue = true;
                    lastMoveTime = now;
                    LastReason = "stuck:" + tacticalName;
                }
            }
            catch { }

            if (needIssue)
            {
                IssueTask();
                LastTickReissued = true;
                return true;
            }
            return false;
        }

        public void UpdatePathError(Vector3 egoPos, float egoHeadingDeg, float egoSpeed,
            RaceRoute route, TrajectoryCandidate chosen, bool hasChosen, float targetSpeed)
        {
            var e = new PathFollowingError { Valid = false };
            try
            {
                if (route == null || !route.Built) { LastError = e; return; }
                float desiredLat = 0f;
                float desiredHeading = route.HeadingAtS(route.AlongS + 8f);
                float distToPath = 999f;
                if (hasChosen && chosen.Path != null && chosen.Path.Count >= 2)
                {
                    desiredLat = chosen.LateralM * 0.15f; // approx near-field desire
                    // Desired heading from the first path segment (what we asked).
                    var p0 = chosen.Path[0];
                    var p1 = chosen.Path[Math.Min(2, chosen.Path.Count - 1)];
                    var d = new Vector3(p1.X - p0.X, p1.Y - p0.Y, 0f);
                    if (RaceMath.FlatLength(d) > 0.5f)
                        desiredHeading = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(d));
                    // Min distance to the polyline (true tracking error).
                    float best = float.MaxValue;
                    for (int i = 0; i < chosen.Path.Count - 1; i++)
                    {
                        var pr = RaceMath.ProjectOnSegment(egoPos, chosen.Path[i], chosen.Path[i + 1]);
                        if (pr.Dist < best) best = pr.Dist;
                    }
                    distToPath = best;
                    // Near-field lateral desire: interpolate first stations.
                    desiredLat = route.Lateral; // fallback
                    try
                    {
                        // Lateral of ego relative to path start frame ≈ route lateral
                        // minus path's initial lateral (which starts at old ego lat).
                        desiredLat = 0f;
                    }
                    catch { }
                }
                e.Valid = true;
                e.LateralErrM = route.Lateral - desiredLat;
                // When a chosen path exists, lateral error vs the path polyline
                // is better expressed as signed cross-track: use dist with sign
                // from route lateral for now (both share the route frame).
                e.HeadingErrDeg = RaceMath.HeadingDiffDeg(egoHeadingDeg, desiredHeading);
                // Note: HeadingDiff(target,current) convention is (target-current);
                // we want (actual-desired), so negate the helper's order.
                e.HeadingErrDeg = -e.HeadingErrDeg;
                e.SpeedErrMps = targetSpeed - egoSpeed;
                e.DistToPathM = distToPath;
            }
            catch { e.Valid = false; }
            LastError = e;
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
                if (driver != null && driver.Exists())
                {
                    driver.Task.ClearAll();
                    Function.Call(Hash.SET_PED_KEEP_TASK, driver, false);
                    Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, false);
                    driver.IsPersistent = false;
                }
                if (vehicle != null && vehicle.Exists())
                    vehicle.IsPersistent = false;
            }
            catch { }
        }

        private float lastIssueAimX;
        private float lastIssueAimY;
        private float lastIssueCruise;
        private int lastIssueStyle;
        private bool hasIssued;

        private float AimMovedSinceIssue()
        {
            if (!hasIssued) return 999f;
            float dx = CurrentAim.X - lastIssueAimX;
            float dy = CurrentAim.Y - lastIssueAimY;
            float moved = (float)Math.Sqrt(dx * dx + dy * dy);
            if (Math.Abs(CurrentCruise - lastIssueCruise) > 4f) return 999f;
            if (CurrentStyle != lastIssueStyle) return 999f;
            return moved;
        }

        private void IssueTask()
        {
            try
            {
                driver.Task.DriveTo(vehicle, CurrentAim, CurrentCruise, (VehicleDrivingFlags)CurrentStyle, 12f);
            }
            catch
            {
                try { driver.Task.DriveTo(vehicle, CurrentAim, CurrentCruise, (VehicleDrivingFlags)CurrentStyle, 12f); }
                catch { }
            }
            lastIssueAimX = CurrentAim.X;
            lastIssueAimY = CurrentAim.Y;
            lastIssueCruise = CurrentCruise;
            lastIssueStyle = CurrentStyle;
            hasIssued = true;
            lastRefreshTime = Game.GameTime;
            ReissueCount++;
        }
    }
}
