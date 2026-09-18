using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Owns the opponent AI: issues a rushed long-range drive task,
    /// re-paths on an interval and when stuck.
    internal sealed class OpponentDriver
    {
        // Rushed + ignore lights + aggressive overtake (public driving-style guides).
        private const int RushedStyle = 1074528293;

        private Ped driver;
        private Vehicle vehicle;
        private Vector3 target;
        private float cruiseSpeed;
        private int retaskMs;
        private int stuckMs;

        private Vector3 lastPos = Vector3.Zero;
        private int lastMoveTime;
        private int lastTaskTime;
        public bool Running { get; private set; }

        public void Start(Ped driver, Vehicle vehicle, Vector3 target, float cruiseSpeed, int retaskMs, int stuckMs)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.target = target;
            this.cruiseSpeed = cruiseSpeed;
            this.retaskMs = retaskMs;
            this.stuckMs = stuckMs;

            driver.IsPersistent = true;
            vehicle.IsPersistent = true;

            // Race, don't flee when bumped; max out skill knobs.
            Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, true);
            Function.Call(Hash.SET_PED_KEEP_TASK, driver, true);
            Function.Call(Hash.SET_DRIVER_ABILITY, driver, 1.0f);
            Function.Call(Hash.SET_DRIVER_AGGRESSIVENESS, driver, 1.0f);

            IssueTask();
            lastPos = vehicle.Position;
            lastMoveTime = Game.GameTime;
            Running = true;
        }

        public void OnTick()
        {
            if (!Running || !Valid())
            {
                return;
            }

            int now = Game.GameTime;
            if (now - lastTaskTime > retaskMs)
            {
                IssueTask();
            }

            // Stuck? (crawling and barely moved) -> force a fresh path.
            bool crawling = vehicle.Speed < 1.5f;
            if (FinishPicker.FlatDistance(vehicle.Position, lastPos) > 4f)
            {
                lastPos = vehicle.Position;
                lastMoveTime = now;
            }
            else if (crawling && now - lastMoveTime > stuckMs)
            {
                IssueTask();
                lastMoveTime = now;
            }
        }

        public void Stop()
        {
            Running = false;
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
                {
                    vehicle.IsPersistent = false;
                }
            }
            catch
            {
            }
        }

        public bool Valid()
        {
            return driver != null && driver.Exists() && !driver.IsDead
                && vehicle != null && vehicle.Exists() && !vehicle.IsDead;
        }

        private void IssueTask()
        {
            driver.Task.DriveTo(vehicle, target, 15f, (VehicleDrivingFlags)RushedStyle, cruiseSpeed);
            lastTaskTime = Game.GameTime;
        }
    }
}
