using System;
using GTA;
using GTA.Math;
using StreetRacing.Control;
using StreetRacing.Tactics;

namespace StreetRacing.Race
{
    /// Orchestrator for the new street-racing AI:
    ///   route -> corridor -> perception/prediction -> tactics ->
    ///   trajectories -> speed profile -> actuator (stock DriveTo servo).
    ///
    /// Tick cadence (script runs at ~20 Hz / 50 ms):
    ///   every tick : ego kinematics + impact classification + capability
    ///   10 Hz      : perception, route, tactics, trajectory, speed, telemetry
    ///   5 Hz       : corridor width probes (native-heavy)
    internal sealed class RaceBrain
    {
        private Ped driver;
        private Vehicle vehicle;
        private Vector3 finish = Vector3.Zero;
        private float cruiseSetting;
        private int style;
        private DriverProfile profile;
        private RaceTelemetry telemetry;
        private int t0;

        private readonly RaceRoute route = new RaceRoute();
        private readonly RoadCorridor corridor = new RoadCorridor();
        private readonly Perception perception = new Perception();
        private readonly VehicleCapability capability = new VehicleCapability();
        private readonly TrajectoryPlanner traj = new TrajectoryPlanner();
        private readonly SpeedPlanner speedPlan = new SpeedPlanner();
        private readonly RaceTactics tactics = new RaceTactics();
        private readonly GtaDriverActuator actuator = new GtaDriverActuator();

        private int refreshMs = 2000;
        private int stuckMs = 4000;

        // Ego kinematics history for accel / impact classification.
        private float lastSpeed;
        private Vector3 lastPos = Vector3.Zero;
        private float lastHeading;
        private int lastKinT;
        private float lastHealth = -1f;
        private bool hasKin;
        private SampleKind lastImpact = SampleKind.Normal;
        private int lastImpactEventMs = -100000;
        private float lastAccelLong;
        private float lastLatAccel;

        // Scheduling.
        private int lastPercMs;
        private int lastCorrMs;
        private int lastPlanMs;
        private int lastTeleMs;

        // Plan snapshot for telemetry / HUD.
        public string TacticalName => tactics.Mode.ToString();
        public float TargetSpeed { get; private set; }
        public string SpeedLimit { get; private set; } = "Cruise";
        public float ActualSpeed { get; private set; }
        public float FinishGap { get; private set; }
        public float Progress01 => route.Progress01;
        public bool RouteLost => route.IsLost;
        public float LookaheadM { get; private set; } = 80f;
        public TrajectoryCandidate Chosen => traj.HasChosen ? traj.Chosen : new TrajectoryCandidate();
        public bool Running { get; private set; }

        private TacticalMode lastLoggedTactic = (TacticalMode)(-1);
        private bool lastLoggedLost;
        private string lastLoggedLossReason = "";
        private int lastBrakeEventMs = -100000;

        public void Start(Ped driver, Vehicle vehicle, Vector3 finish, float cruise,
            int style, DriverProfile profile, RaceTelemetry telemetry,
            int refreshMs, int stuckMs)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.finish = finish;
            this.cruiseSetting = cruise;
            this.style = style;
            this.profile = profile ?? DriverProfile.FromName("balanced");
            this.telemetry = telemetry;
            this.refreshMs = refreshMs;
            this.stuckMs = stuckMs;
            t0 = Game.GameTime;

            Vector3 origin;
            try { origin = vehicle.Position; } catch { origin = Game.Player.Character.Position; }
            route.Build(origin, finish);
            capability.Seed(vehicle);
            corridor.Update(route, origin, t0);
            route.Update(origin, SafeHeading(vehicle), 0f, t0, corridor.HalfWidth);

            actuator.Attach(driver, vehicle, cruise, style, refreshMs, stuckMs);
            tactics.SinceMs = t0;

            hasKin = false;
            lastHealth = -1f;
            Running = true;

            try
            {
                telemetry?.Event(0, "ROUTE", $"pts={route.Points.Count};len={route.TotalLength:F0};prof={this.profile.Name};risk={this.profile.RiskTolerance:F2}");
            }
            catch { }
        }

        public bool Valid()
        {
            try
            {
                return Running && actuator.Valid();
            }
            catch { return false; }
        }

        public void Stop()
        {
            Running = false;
            try { actuator.Stop(); } catch { }
        }

        public void OnTick()
        {
            if (!Running) return;
            int now = Game.GameTime;
            int t = now - t0;

            Vector3 egoPos;
            Vector3 egoVel;
            Vector3 egoFwd;
            float egoSpeed;
            float egoHeading;
            try
            {
                if (vehicle == null || !vehicle.Exists()) return;
                egoPos = vehicle.Position;
                egoVel = vehicle.Velocity;
                egoSpeed = vehicle.Speed;
                egoFwd = RaceMath.FlatNormalize(new Vector3(vehicle.ForwardVector.X, vehicle.ForwardVector.Y, 0f));
                egoHeading = vehicle.Heading;
            }
            catch { return; }
            ActualSpeed = egoSpeed;
            FinishGap = RaceMath.FlatDistance(egoPos, finish);

            // --- Every-tick kinematics + impact classification (cheap).
            UpdateKinematics(now, egoPos, egoSpeed, egoHeading);

            if (!hasKin)
            {
                hasKin = true;
                lastSpeed = egoSpeed;
                lastPos = egoPos;
                lastHeading = egoHeading;
                lastKinT = now;
                try { lastHealth = vehicle.HealthFloat; } catch { lastHealth = -1f; }
                return; // need one more sample for accel
            }

            // --- 10 Hz: perception.
            if (now - lastPercMs >= profile.ReactionIntervalMs)
            {
                lastPercMs = now;
                float reactionS = profile.ReactionIntervalMs / 1000f;
                Vehicle playerVeh = SafePlayerVehicle();
                Ped playerPed = null;
                try { playerPed = Game.Player.Character; } catch { }
                try
                {
                    perception.Update(vehicle, playerVeh, playerPed, egoSpeed,
                        capability.UsableBrake(profile.GripFactor), reactionS, now, profile.ReactionIntervalMs);
                }
                catch { }
            }

            // --- Route update at plan rate (needs corridor width for thresholds).
            bool doPlan = now - lastPlanMs >= profile.ReactionIntervalMs;
            if (doPlan)
            {
                lastPlanMs = now;
                try { route.Update(egoPos, egoHeading, egoSpeed, now, corridor.HalfWidth); }
                catch { }
            }

            // --- 5 Hz: corridor probes (native-heavy: boundary + sweeps).
            if (now - lastCorrMs >= 200)
            {
                lastCorrMs = now;
                try { corridor.Update(route, egoPos, now); } catch { }
            }

            if (doPlan)
            {
                // Rival (player) state for tactics.
                Vector3 rivalPos = egoPos;
                Vector3 rivalVel = new Vector3();
                float rivalDist = 9999f;
                bool rivalAhead = false;
                try
                {
                    var pc = Game.Player.Character;
                    Vehicle pv = null;
                    try { pv = pc.IsInVehicle() ? pc.CurrentVehicle : null; } catch { }
                    if (pv != null && pv.Exists() && pv != vehicle)
                    {
                        rivalPos = pv.Position;
                        try { rivalVel = pv.Velocity; } catch { }
                    }
                    else if (pc != null && pc.Exists())
                    {
                        rivalPos = pc.Position;
                        try { rivalVel = pc.Velocity; } catch { }
                    }
                    rivalDist = RaceMath.FlatDistance(egoPos, rivalPos);
                    var toR = new Vector3(rivalPos.X - egoPos.X, rivalPos.Y - egoPos.Y, 0f);
                    rivalAhead = RaceMath.FlatDot(toR, egoFwd) > 0f;
                }
                catch { }

                bool justImpacted = (now - lastImpactEventMs) < 1200;

                LookaheadM = profile.LookaheadForSpeed(egoSpeed);

                // Curve limit preview for tactic gating (uses current capability).
                float aLatPreview = capability.UsableLat(profile.GripFactor);
                float k80 = route.CurvatureAhead(80f);
                float curvePreview = k80 > 1e-5f ? (float)Math.Sqrt(aLatPreview / k80) : cruiseSetting;

                try
                {
                    tactics.Update(route, corridor, perception, egoPos, egoFwd, egoSpeed,
                        rivalPos, rivalVel, rivalDist, rivalAhead, curvePreview, cruiseSetting,
                        now, profile, justImpacted);
                }
                catch { }

                TrajectoryCandidate chosen = new TrajectoryCandidate();
                float target = cruiseSetting;
                bool recoveredTarget = false;
                try
                {
                    if (route.IsLost || tactics.Mode == TacticalMode.Recovery || tactics.Mode == TacticalMode.Crashed)
                    {
                        Vector3 rec = route.RecoveryTarget();
                        chosen = new TrajectoryCandidate
                        {
                            LateralM = 0f,
                            LookaheadM = 40f,
                            AimPoint = rec,
                            Score = 0f,
                            ClearanceM = 999f,
                            CurveCost = 0f,
                            TacticalBias = 0f,
                            RejectReason = route.LossReason,
                        };
                        traj.LastCandidates.Clear();
                        traj.LastCandidates.Add(chosen);
                        traj.Chosen = chosen;
                        traj.HasChosen = true;
                        target = Math.Min(cruiseSetting, 11f);
                        speedPlan.TargetSpeed = target;
                        speedPlan.Limiting = "Recovery";
                        speedPlan.CurveLimit = target;
                        speedPlan.ObstacleLimit = target;
                        speedPlan.RequiredDecel = 0f;
                        speedPlan.BrakingNeed = 0f;
                        recoveredTarget = true;
                    }
                    else
                    {
                        chosen = traj.Plan(route, corridor, perception, tactics, profile, egoPos, egoFwd, egoSpeed, LookaheadM);
                        target = speedPlan.Plan(cruiseSetting, egoSpeed, route, corridor, perception,
                            capability, profile, tactics.Mode, LookaheadM);
                    }
                }
                catch { }

                TargetSpeed = target;
                SpeedLimit = speedPlan.Limiting ?? "Cruise";

                // --- Actuator: short-horizon servo, not long-range DriveTo.
                bool forceReissue = tactics.ChangedThisTick || recoveredTarget;
                if (lastLoggedTactic != tactics.Mode) forceReissue = true;
                if (route.IsLost != lastLoggedLost) forceReissue = true;
                try
                {
                    Vector3 aim = traj.HasChosen ? traj.Chosen.AimPoint : route.LookaheadPoint(LookaheadM);
                    // Keep the aim on/near drivable surface: if the planned aim
                    // somehow leaves the road, pull it back toward the corridor
                    // center instead of asking GTA to path off-road.
                    aim = ClampAimToCorridor(aim, route);
                    actuator.SetPlan(aim, Math.Max(0f, target), style, tactics.Mode.ToString());
                    bool reissued = actuator.OnTick(forceReissue, tactics.Mode.ToString());
                    if (reissued)
                    {
                        try
                        {
                            telemetry?.Event(t, "CTRL", $"aim={aim.X:F0},{aim.Y:F0};v={target:F0};mode={tactics.Mode};why={actuator.LastReason}");
                        }
                        catch { }
                    }
                }
                catch { }

                EmitTransitionEvents(t, now);
            }

            // --- 10 Hz telemetry samples.
            if (now - lastTeleMs >= 100)
            {
                lastTeleMs = now;
                WriteSample(t, now, egoPos, egoSpeed);
            }

            lastSpeed = egoSpeed;
            lastPos = egoPos;
            lastHeading = egoHeading;
            lastKinT = now;
        }

        private void UpdateKinematics(int now, Vector3 egoPos, float egoSpeed, float egoHeading)
        {
            if (!hasKin) return;
            float dtS = (now - lastKinT) / 1000f;
            if (dtS <= 0f || dtS > 0.6f)
            {
                lastImpact = SampleKind.Normal;
                return;
            }
            float accel = (egoSpeed - lastSpeed) / dtS;
            lastAccelLong = accel;
            float dhDeg = RaceMath.HeadingDiffDeg(egoHeading, lastHeading);
            float yawRate = dhDeg * (float)Math.PI / 180f / dtS;
            lastLatAccel = egoSpeed * yawRate;

            float displacement = RaceMath.FlatDistance(egoPos, lastPos);
            float expected = (Math.Abs(egoSpeed) + Math.Abs(lastSpeed)) * 0.5f * dtS;
            float health = -1f;
            float healthDrop = 0f;
            try
            {
                health = vehicle.HealthFloat;
                if (lastHealth > 0f) healthDrop = lastHealth - health;
            }
            catch { }
            bool collided = false;
            ImpactClassifier.TryReadCollision(vehicle, out collided);

            SampleKind kind = ImpactClassifier.Classify(accel, dtS, displacement, expected, healthDrop, collided, egoSpeed);
            lastImpact = kind;
            if ((kind == SampleKind.Impact || kind == SampleKind.Teleport) && now - lastImpactEventMs > 1500)
            {
                lastImpactEventMs = now;
                try
                {
                    if (kind == SampleKind.Impact)
                        telemetry?.Event(now - t0, "IMPACT", $"dec={accel:F0};spd={egoSpeed:F0};dmg={healthDrop:F0};offCorr={corridor.OffCorridor(route.Lateral):F0}");
                    else
                        telemetry?.Event(now - t0, "TELEPORT", $"moved={displacement:F0};exp={expected:F0};spd={egoSpeed:F0}");
                }
                catch { }
            }
            // Genuine hard braking (never an impact): log sparsely.
            if (kind == SampleKind.Braking && now - lastBrakeEventMs > 3000)
            {
                lastBrakeEventMs = now;
                try { telemetry?.Event(now - t0, "HARD_BRAKE", $"spd={egoSpeed:F0};dec={accel:F0}"); } catch { }
            }

            // Capability learns only from plausible tyre samples.
            if (kind == SampleKind.Normal || kind == SampleKind.Braking)
            {
                try { capability.Observe(accel, lastLatAccel, egoSpeed, dtS); } catch { }
            }
            if (health >= 0f) lastHealth = health;
        }

        private void EmitTransitionEvents(int t, int now)
        {
            try
            {
                if (lastLoggedTactic != tactics.Mode)
                {
                    if ((int)lastLoggedTactic != -1)
                        telemetry?.Event(t, "TACTIC", $"{lastLoggedTactic}->{tactics.Mode};why={tactics.Reason};lat={tactics.DesiredLateral:F2}");
                    lastLoggedTactic = tactics.Mode;
                }
                if (route.IsLost && !lastLoggedLost)
                    telemetry?.Event(t, "ROUTE_LOST", $"{route.LossReason};prog={route.AlongS:F0};dist={route.DistToRoute:F0}");
                else if (!route.IsLost && lastLoggedLost)
                    telemetry?.Event(t, "ROUTE_FOUND", $"prog={route.AlongS:F0}");
                lastLoggedLost = route.IsLost;
                lastLoggedLossReason = route.LossReason ?? "";
            }
            catch { }
        }

        private void WriteSample(int t, int now, Vector3 egoPos, float egoSpeed)
        {
            if (telemetry == null) return;
            try
            {
                float offCorr = corridor.OffCorridor(route.Lateral);
                float curv = route.CurvatureAhead(80f);
                float aimLat = traj.HasChosen ? traj.Chosen.LateralM : 0f;
                float chScore = traj.HasChosen ? traj.Chosen.Score : 0f;
                float rejLat = 0f;
                float rejScore = 0f;
                if (traj.LastCandidates.Count > 1)
                {
                    rejLat = traj.LastCandidates[1].LateralM;
                    rejScore = traj.LastCandidates[1].Score;
                }
                float nearD = 999f;
                float nearTtc = 999f;
                float nearClose = 0f;
                if (perception.TryGetClosestThreat(out var th))
                {
                    nearD = th.Dist;
                    nearTtc = th.Ttc;
                    nearClose = th.ClosingSpeed;
                }
                else if (perception.AheadCount > 0)
                {
                    nearD = perception.NearestAheadDist;
                    nearTtc = perception.NearestAheadTtc;
                    nearClose = perception.NearestAheadClosing;
                }
                telemetry.Sample(t, style, tactics.Mode.ToString(),
                    route.AlongS, route.Progress01, LookaheadM,
                    route.Lateral, corridor.HalfWidth, offCorr, route.HeadingErrorDeg, curv,
                    aimLat, chScore, rejLat, rejScore,
                    TargetSpeed, egoSpeed, SpeedLimit ?? "Cruise", speedPlan.BrakingNeed,
                    capability.ABrakeMax, capability.ALatMax, perception.Count,
                    nearD, nearTtc, nearClose,
                    actuator.CurrentCruise, actuator.CurrentStyle,
                    route.IsLost ? 1 : 0, lastImpact.ToString(),
                    actuator.ReissueCount, FinishGap);
            }
            catch { }
        }

        private Vector3 ClampAimToCorridor(Vector3 aim, RaceRoute rt)
        {
            try
            {
                float half = corridor.HalfWidth;
                // Express aim in the ahead-slice frame; pull back if outside.
                Vector3 center = rt.LookaheadPoint(LookaheadM);
                float h = rt.HeadingAhead(LookaheadM);
                var dir = RaceMath.VectorFromHeading(h);
                var to = new Vector3(aim.X - center.X, aim.Y - center.Y, 0f);
                float lat = RaceMath.FlatCross(dir, to);
                float lon = RaceMath.FlatDot(to, dir);
                if (Math.Abs(lat) > half + 2f)
                {
                    float cl = RaceMath.Clamp(lat, -(half + 1f), half + 1f);
                    var left = new Vector3(-dir.Y, dir.X, 0f);
                    return new Vector3(center.X + dir.X * lon + left.X * cl,
                        center.Y + dir.Y * lon + left.Y * cl, aim.Z);
                }
            }
            catch { }
            return aim;
        }

        private static float SafeHeading(Vehicle v)
        {
            try { return v.Heading; } catch { return 0f; }
        }

        private Vehicle SafePlayerVehicle()
        {
            try
            {
                var pc = Game.Player.Character;
                if (pc != null && pc.Exists() && pc.IsInVehicle())
                    return pc.CurrentVehicle;
            }
            catch { }
            return null;
        }
    }
}
