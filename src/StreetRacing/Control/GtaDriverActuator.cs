using System;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing.Control
{
    /// Stock GTA driver used as a trajectory/speed servo: we command a
    /// SHORT-horizon aim point on the planned trajectory (80–150 m), not the
    /// 2 km finish. That keeps the game pathfinder on the correct carriageway
    /// and makes unreachable-finish losses structurally impossible — the
    /// failure mode that killed the old long-range DriveTo.
    ///
    /// Re-issues are rate-limited (aim moved / speed changed / mode changed /
    /// stuck): per-tick we only refresh cruise + style, which does not stutter.
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

    /// Placeholder for a future direct steering/throttle/brake controller
    /// (SET_VEHICLE_STEER / throttle natives under DriveV). Kept so the
    /// planner -> actuator seam is proven swappable; not wired by default.
    internal sealed class DirectActuatorStub : IVehicleActuator
    {
        public Vector3 CurrentAim { get; private set; } = Vector3.Zero;
        public float CurrentCruise { get; private set; }
        public int CurrentStyle { get; private set; }
        public bool HasPlan { get; private set; }
        public int ReissueCount => 0;
        public void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason)
        {
            CurrentAim = aimPoint;
            CurrentCruise = targetSpeed;
            CurrentStyle = style;
            HasPlan = true;
            throw new NotImplementedException("Direct actuator is a seam placeholder; use GtaDriverActuator.");
        }
        public void Clear() { HasPlan = false; }
    }
}
