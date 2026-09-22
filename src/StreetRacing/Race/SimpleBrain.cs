using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;
using GTA.Native;
using StreetRacing.Control;
using StreetRacing.Debug;

namespace StreetRacing.Race
{
    /// Milestone simple driver: competent basic GPS route following, nothing more.
    ///
    /// Pipeline (this pass only):
    ///   valid GPS route
    ///   -> stable route localization (RaceRoute continuity-aware Update)
    ///   -> optional ONE-TIME pose join (PoseConnector, only when start is
    ///      offset/misaligned; latched, never re-entered)
    ///   -> persistent GPS centerline reference (route-anchored window, NOT
    ///      rebuilt through ego)
    ///   -> curvature-based desired road speed (cruise-capped, braking-feasible)
    ///   -> path-specific actor prediction constrains speed for genuine conflicts
    ///   -> persistent acceleration-limited commanded speed
    ///      (MoveTowards(prevCommanded, desired, limit*dt), NEVER actual+1)
    ///   -> Direct steering/throttle/brake.
    ///
    /// SpatialPlannerV1 is ENABLED after the initial join:
    ///   a local 2.5D road-surface world model fuses road supports + moving actor footprints;
    ///   beam search expands short curvature primitives directly in world XY on connected road surfaces;
    ///   GPS is only the global route-progress objective, not the trajectory
    ///   coordinate system.
    ///
    /// Still disabled:
    ///   semantic lane graph / oncoming-lane classification / player tactics /
    ///   GTA DriveTo fallback / FallbackWalk / StraightFallback.
    ///
    /// RecoveryPrimitive is enabled only for structural loss or persistent
    /// planner infeasibility without an active traffic constraint. It owns an
    /// explicit Stop -> Reverse -> Forward -> Rejoin sequence; normal traffic
    /// waiting never enters recovery.
    ///
    /// Speed separation (the recursive bug this fixes):
    ///   desiredRoadSpeed = road allows (cruise + curvature + braking distance).
    ///   commandedSpeed  = persistent ramp toward desired (accel/decel limits).
    ///   actualSpeed     = vehicle.Speed (measured, NEVER feeds desired).
    ///   localTarget     = SpeedAtS(profile, sEgo) ~= commandedSpeed.
    /// Telemetry exposes all four separately. SpeedLimit identifies
    /// Traffic:<kind>#handle, Curvature, AccelRamp, or Cruise.
    ///
    /// Path separation (the ego-anchored bug this fixes):
    ///   TRACK path geometry is anchored to the ROUTE (PointAtS(AlongS + s)),
    ///   so cross-track error stays meaningful (2 m left reads ~+2 m).
    ///   The old KEEP_LINE rebuilt a Hermite through ego every 100 ms, which
    ///   zeroed the error by construction and hid drift. PoseConnector is used
    ///   ONLY for the initial JOIN when start pose is offset/misaligned; once
    ///   aligned we latch to normal planning. Structural loss is handled by an
    ///   explicit recovery state rather than silently falling back to a rail.
    ///
    /// Steering sign (empirical, DirectDiag):
    ///   DirectDiag SteerLeft commands SteeringAngle = +12 deg and the NPC
    ///   visibly turned LEFT, so positive GTA steering = left.
    ///   crossTrack > 0 means ego is LEFT of the desired path direction.
    ///   Returning from the left therefore needs RIGHT = negative steering,
    ///   i.e. steer -= crossTrack * gain. This is verified by
    ///   DirectActuator.SteeringSignSelfTest() at Start (logged).
    internal sealed class SimpleBrain
    {
        private Ped driver;
        private Vehicle vehicle;
        private Vector3 finish = Vector3.Zero;
        private float cruiseSetting = 18f;
        private float simpleCruiseCap = 18f;
        private int style;
        private DriverProfile profile;
        private RaceTelemetry telemetry;
        private int t0;

        private readonly RaceRoute route = new RaceRoute();
        private readonly RoadCorridor corridor = new RoadCorridor();
        private readonly VehicleCapability capability = new VehicleCapability();
        // Spatial planning uses a persistent perception world, but trajectory
        // geometry itself is searched in world XY on connected road surfaces rather than route offsets.
        private readonly Perception perception = new Perception();
        private readonly LocalWorldModel localWorld = new LocalWorldModel();
        // Observer-only physical geometry map. It is visualized/telemetered but
        // intentionally not consumed by SpatialPlannerV1 until we trust what it sees.
        private readonly PhysicalSurfaceMap physicalSurface = new PhysicalSurfaceMap();
        private readonly SpatialPlannerV1 spatialPlanner = new SpatialPlannerV1();
        private readonly RecoveryPrimitive recovery = new RecoveryPrimitive();
        private readonly TrajectoryPlanner trajViz = new TrajectoryPlanner();
        private readonly SpeedPlanner speedViz = new SpeedPlanner();
        private readonly DrivingReference drivingReference = new DrivingReference();
        private readonly RaceDebugViz viz = new RaceDebugViz();
        private IVehicleActuator actuator;

        private int lastPlanMs;
        private int lastCorrMs;
        private int lastTeleMs;
        private int lastGpsRetryMs;
        private int lastGpsMissingLogMs = -100000;
        private int lastLostLogMs = -100000;
        private bool lastLoggedLost;
        private int routeLostSinceMs = -1;
        private int referenceInvalidSinceMs = -1;
        private int plannerInvalidSinceMs = -1;
        private int recoveryEnteredMs = -1;
        private RecoveryPrimitive.Stage lastRecoveryStage = RecoveryPrimitive.Stage.Idle;
        private int stallSinceMs = -1;
        private int lastStallEventMs = -100000;
        private bool stallWasActive;

        private float lastSpeed;
        private float lastSignedLong;
        private Vector3 lastPos = Vector3.Zero;
        private bool hasKin;
        private int lastKinT;
        private float lastKinHeading;
        private float lastHealth = -1f;
        private float lastAccelLong;
        private float lastSlipDeg;
        private float lastYawRate;

        private TrajectoryCandidate current = new TrajectoryCandidate();
        private bool hasCurrent;
        private int maneuverPlanId;
        private string lastIntentLog = "";
        private int lastIntentLogMs = -100000;
        private string lastStabilityMode = "";
        private int lastStabilityEventMs = -100000;
        private string lastVizError = "";
        private int lastVizErrorMs = -100000;
        private int lastRoadModelEventMs = -100000;
        private bool roadModelWasLow;
        private int lastPhysicalSurfaceEventMs = -100000;
        private int lastPhysicalSurfaceErrorMs = -100000;
        private float lastRefRawKappa;
        private float lastRefKappa;
        private float lastRefHeadStep;
        private int lastRefRoadClamp;
        private string lastRefDetail = "";
        private DrivingReference.Result lastRoadReference;
        private float egoHalfLength = 2.3f;
        private float egoHalfWidth = 1.0f;
        private Vector3 lastEgoFwd = new Vector3(0f, 1f, 0f);
        private float lastEgoHeading;

        // --- Milestone persistent state (never reconstructed from actualSpeed).
        private float desiredRoadSpeed = 18f;
        private float commandedSpeed;
        private bool commandedInit;
        private int prevPlanMs = -1;

        // --- One-time startup join latch. Normal spatial planning owns the
        // route after this; explicit recovery may temporarily take over.
        private bool joined;
        private string joinState = "Init";

        public bool Running { get; private set; }
        public string TacticalName => joinState;
        public float TargetSpeed { get; private set; }
        public string SpeedLimit { get; private set; } = "Cruise";
        public float ActualSpeed { get; private set; }
        public float FinishGap { get; private set; }
        public float Progress01 => route.Progress01;
        public bool RouteLost => route.IsLost;
        public float LookaheadM { get; private set; } = 70f;
        public string RouteSource => route.Source;
        public string ActuatorName => actuator != null ? actuator.ActuatorName : "?";
        public float DesiredRoadSpeed => desiredRoadSpeed;
        public float CommandedSpeed => commandedSpeed;
        public string JoinState => joinState;
        public bool TestFailed { get; private set; }
        public string TestFailureReason { get; private set; } = "";

        // Join thresholds: when is PoseConnector actually needed?
        private const float JoinLatThreshM = 2.0f;
        private const float JoinHeadThreshDeg = 15f;
        private const float JoinedLatM = 1.5f;
        private const float JoinedHeadDeg = 10f;
        private const string SpatialBuildTag = "physical-traversability-observer-v3";

        public void Start(Ped driver, Vehicle vehicle, Vector3 finish, float cruise,
            int style, DriverProfile profile, RaceTelemetry telemetry,
            int refreshMs, int stuckMs, bool debugViz,
            float simpleCruiseCap, bool enablePassing, bool useGtaRejoin)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.finish = finish;
            this.cruiseSetting = cruise;
            this.simpleCruiseCap = simpleCruiseCap > 1f ? simpleCruiseCap : 18f;
            this.style = style;
            this.profile = profile ?? DriverProfile.FromName("balanced");
            this.telemetry = telemetry;
            // Milestone: passing / GTA rejoin / recovery are hard-disabled,
            // even if the ini enables them. Log the override explicitly.
            t0 = Game.GameTime;

            try { route.Reset(); } catch { }
            try { corridor.Reset(); } catch { }
            try { drivingReference.Reset(); } catch { }
            try { perception.Reset(); } catch { }
            try { localWorld.Reset(); } catch { }
            try { physicalSurface.Reset(); } catch { }
            try { spatialPlanner.Reset(); } catch { }
            try { recovery.Reset(t0); } catch { }
            try { trajViz.Reset(); } catch { }
            try { speedViz.Reset(); } catch { }

            Vector3 origin;
            try { origin = vehicle.Position; } catch { origin = Game.Player.Character.Position; }
            float originHeading = SafeHeading(vehicle);
            float originSpeed = 0f;
            try { originSpeed = vehicle.Speed; } catch { }
            route.Build(origin, finish);
            capability.Seed(vehicle);
            try
            {
                Vector3 min;
                Vector3 max;
                vehicle.Model.GetDimensions(out min, out max);
                float len = Math.Abs(max.Y - min.Y);
                float wid = Math.Abs(max.X - min.X);
                if (len > 1f && len < 20f) egoHalfLength = len * 0.5f;
                if (wid > 0.8f && wid < 8f) egoHalfWidth = wid * 0.5f;
            }
            catch { egoHalfLength = 2.3f; egoHalfWidth = 1.0f; }
            LookaheadM = this.profile.LookaheadForSpeed(0f);
            try { corridor.Update(route, origin, LookaheadM, t0); } catch { }
            try { route.Update(origin, originHeading, 0f, t0, corridor.HalfWidth); } catch { }

            actuator = new DirectActuator();
            actuator.Attach(driver, vehicle, EffectiveCruise(), style, refreshMs, stuckMs);
            try
            {
                var direct = actuator as DirectActuator;
                telemetry?.Event(Math.Max(0, Game.GameTime - t0), "OWNERSHIP",
                    direct != null ? direct.TakeoverDetail : "direct-cast-failed");
            }
            catch { }
            viz.Enabled = debugViz;
            try { telemetry?.Event(Math.Max(0, Game.GameTime - t0), "DEBUG_VIZ", $"enabled={(debugViz ? 1 : 0)}"); } catch { }
            try { telemetry?.Event(Math.Max(0, Game.GameTime - t0), "SPATIAL_BUILD", SpatialBuildTag); } catch { }

            hasKin = false;
            lastSpeed = originSpeed;
            lastSignedLong = originSpeed;
            lastPos = origin;
            lastKinT = t0;
            lastKinHeading = originHeading;
            lastHealth = -1f;
            lastAccelLong = 0f;
            lastSlipDeg = 0f;
            lastYawRate = 0f;
            lastPlanMs = 0;
            lastCorrMs = 0;
            lastTeleMs = 0;
            lastGpsRetryMs = 0;
            lastGpsMissingLogMs = -100000;
            lastLostLogMs = -100000;
            lastLoggedLost = false;
            routeLostSinceMs = -1;
            referenceInvalidSinceMs = -1;
            plannerInvalidSinceMs = -1;
            recoveryEnteredMs = -1;
            lastRecoveryStage = RecoveryPrimitive.Stage.Idle;
            stallSinceMs = -1;
            lastStallEventMs = -100000;
            stallWasActive = false;
            TestFailed = false;
            TestFailureReason = "";
            lastRoadModelEventMs = -100000;
            roadModelWasLow = false;
            lastPhysicalSurfaceEventMs = -100000;
            lastPhysicalSurfaceErrorMs = -100000;
            lastRefRawKappa = 0f;
            lastRefKappa = 0f;
            lastRefHeadStep = 0f;
            lastRefRoadClamp = 0;
            lastRefDetail = "";
            lastRoadReference = null;
            TargetSpeed = 0f;
            SpeedLimit = "Cruise";
            ActualSpeed = originSpeed;
            FinishGap = RaceMath.FlatDistance(origin, finish);
            maneuverPlanId = 0;
            hasCurrent = false;
            lastIntentLog = "";
            lastIntentLogMs = -100000;
            lastStabilityMode = "";
            lastStabilityEventMs = -100000;
            lastVizError = "";
            lastVizErrorMs = -100000;
            lastEgoFwd = RaceMath.VectorFromHeading(originHeading);
            lastEgoHeading = originHeading;

            // Persistent speed state: start from actual motion so the first
            // command is continuous, then ramp toward desired. NEVER from
            // actual+constant on later ticks (that was the collapse bug).
            desiredRoadSpeed = EffectiveCruise();
            commandedSpeed = RaceMath.Clamp(originSpeed, 0f, EffectiveCruise());
            commandedInit = true;
            prevPlanMs = t0;

            // Join latch: if start is already aligned, go straight to TRACK.
            // Otherwise JOIN once until aligned, then latch to TRACK forever.
            bool needJoin = Math.Abs(route.Lateral) > JoinLatThreshM
                || Math.Abs(route.HeadingErrorDeg) > JoinHeadThreshDeg;
            joined = !needJoin;
            joinState = IsGpsSource() ? (joined ? "Track" : "Join") : "GpsWait";

            Running = true;

            try
            {
                string poseChk = PoseConnector.SelfTest();
                string steerChk = DirectActuator.SteeringSignSelfTest();
                string headingChk = HeadingConventionCheck(vehicle);
                telemetry?.Event(0, "ROUTE", $"src={route.Source};pts={route.Points.Count};len={route.TotalLength:F0};simpleCruise={EffectiveCruise():F0};poseCheck={poseChk};steerCheck={steerChk};headingCheck={headingChk};gpsOnly=1;recovery=ForwardOnly;controller=V2;spatialPlanner=V1+Gates");
                telemetry?.Event(0, "ACTUATOR", $"Direct;LayeredLocalWorld + SpatialPlannerV1+Gates + persistent cmd speed;iniPassing={enablePassing};gtaRejoin=OFF(override ini={useGtaRejoin});recovery=ForwardOnly;controller=V2");
                string vStart = "";
                try { vStart = route.ValidateStart(origin, originHeading, out string vr) ? $"valid;{vr}" : $"INVALID;{vr}"; }
                catch { vStart = "validate-exc"; }
                telemetry?.Event(0, "START_POSE", $"egoHead={originHeading:F0};routeHead={route.RouteHeadingDeg:F0};headErr={route.HeadingErrorDeg:F0};lat={route.Lateral:F1};dist={route.DistToRoute:F1};s={route.AlongS:F0};startValid={vStart};join={joinState};needJoin={needJoin}");
                if (!IsGpsSource())
                    telemetry?.Event(0, "GPS_WAIT", $"non-gps at start src={route.Source};holding until GPS (never driving on fallback)");
                if (poseChk != "OK")
                    telemetry?.Event(0, "POSE_CONNECTOR_FAIL", $"selftest={poseChk}");
                if (steerChk != "OK")
                    telemetry?.Event(0, "STEER_SIGN_FAIL", $"selftest={steerChk}");
                if (!headingChk.StartsWith("OK"))
                    telemetry?.Event(0, "HEADING_CONVENTION_FAIL", headingChk);
            }
            catch { }
        }

        public bool IsStartPoseValid(out string reason)
        {
            reason = "unknown";
            try
            {
                if (vehicle == null || !vehicle.Exists()) { reason = "no-vehicle"; return false; }
                return route.ValidateStart(vehicle.Position, SafeHeading(vehicle), out reason);
            }
            catch (Exception ex) { try { reason = "exc:" + ex.Message; } catch { } return false; }
        }

        /// Arming-handoff start: uses ONE already-accepted GPS snapshot.
        /// NEVER calls GTA GPS natives (no Build, no TryUpgrade, no sampling).
        /// Localization (route.Update) against the rival's current pose is the
        /// only per-start computation. Driving / speed / join logic below is
        /// identical to Start; only the geometry source differs.
        public void StartFromSnapshot(Ped driver, Vehicle vehicle, RouteSnapshot snapshot, float cruise,
            int style, DriverProfile profile, RaceTelemetry telemetry,
            int refreshMs, int stuckMs, bool debugViz,
            float simpleCruiseCap, bool enablePassing, bool useGtaRejoin, int armingBeginMs)
        {
            if (snapshot == null || snapshot.Points == null || snapshot.Points.Count < 2)
            {
                try { telemetry?.Event(0, "SNAPSHOT_INVALID", "null-or-too-few-points"); } catch { }
                throw new InvalidOperationException("accepted snapshot missing");
            }
            bool isGps = false;
            try { isGps = snapshot.Source != null && snapshot.Source.StartsWith("Gps"); } catch { }
            if (!isGps)
            {
                try { telemetry?.Event(0, "SNAPSHOT_INVALID", $"non-gps src={snapshot.Source};never-fallback"); } catch { }
                throw new InvalidOperationException("non-gps snapshot rejected src=" + snapshot.Source);
            }
            this.driver = driver;
            this.vehicle = vehicle;
            this.finish = snapshot.Finish;
            this.cruiseSetting = cruise;
            this.simpleCruiseCap = simpleCruiseCap > 1f ? simpleCruiseCap : 18f;
            this.style = style;
            this.profile = profile ?? DriverProfile.FromName("balanced");
            this.telemetry = telemetry;
            // Continuous lifecycle timeline: arming events already used
            // t = Game.GameTime - armingBeginMs. Keep the same origin so
            // RACE_START continues after ARM_READY instead of restarting at 0.
            int nowGame = 0;
            try { nowGame = Game.GameTime; } catch { }
            t0 = (armingBeginMs > 0 && armingBeginMs <= nowGame) ? armingBeginMs : nowGame;

            try { route.Reset(); } catch { }
            try { corridor.Reset(); } catch { }
            try { drivingReference.Reset(); } catch { }
            try { perception.Reset(); } catch { }
            try { localWorld.Reset(); } catch { }
            try { physicalSurface.Reset(); } catch { }
            try { spatialPlanner.Reset(); } catch { }
            try { recovery.Reset(t0); } catch { }
            try { trajViz.Reset(); } catch { }
            try { speedViz.Reset(); } catch { }

            Vector3 origin;
            try { origin = vehicle.Position; } catch { origin = Game.Player.Character.Position; }
            float originHeading = SafeHeading(vehicle);
            float originSpeed = 0f;
            try { originSpeed = vehicle.Speed; } catch { }
            // SINGLE authoritative geometry: copy accepted snapshot, no GPS natives.
            route.ImportSnapshot(snapshot);
            capability.Seed(vehicle);
            try
            {
                Vector3 min;
                Vector3 max;
                vehicle.Model.GetDimensions(out min, out max);
                float len = Math.Abs(max.Y - min.Y);
                float wid = Math.Abs(max.X - min.X);
                if (len > 1f && len < 20f) egoHalfLength = len * 0.5f;
                if (wid > 0.8f && wid < 8f) egoHalfWidth = wid * 0.5f;
            }
            catch { egoHalfLength = 2.3f; egoHalfWidth = 1.0f; }
            LookaheadM = this.profile.LookaheadForSpeed(0f);
            try { corridor.Update(route, origin, LookaheadM, nowGame); } catch { }
            try { route.Update(origin, originHeading, 0f, nowGame, corridor.HalfWidth); } catch { }

            actuator = new DirectActuator();
            actuator.Attach(driver, vehicle, EffectiveCruise(), style, refreshMs, stuckMs);
            try
            {
                var direct = actuator as DirectActuator;
                telemetry?.Event(Math.Max(0, Game.GameTime - t0), "OWNERSHIP",
                    direct != null ? direct.TakeoverDetail : "direct-cast-failed");
            }
            catch { }
            viz.Enabled = debugViz;
            try { telemetry?.Event(Math.Max(0, Game.GameTime - t0), "DEBUG_VIZ", $"enabled={(debugViz ? 1 : 0)}"); } catch { }
            try { telemetry?.Event(Math.Max(0, Game.GameTime - t0), "SPATIAL_BUILD", SpatialBuildTag); } catch { }

            hasKin = false;
            lastSpeed = originSpeed;
            lastSignedLong = originSpeed;
            lastPos = origin;
            lastKinT = t0;
            lastKinHeading = originHeading;
            lastHealth = -1f;
            lastAccelLong = 0f;
            lastSlipDeg = 0f;
            lastYawRate = 0f;
            lastPlanMs = 0;
            lastCorrMs = 0;
            lastTeleMs = 0;
            lastGpsRetryMs = 0;
            lastGpsMissingLogMs = -100000;
            lastLostLogMs = -100000;
            lastLoggedLost = false;
            routeLostSinceMs = -1;
            referenceInvalidSinceMs = -1;
            plannerInvalidSinceMs = -1;
            recoveryEnteredMs = -1;
            lastRecoveryStage = RecoveryPrimitive.Stage.Idle;
            stallSinceMs = -1;
            lastStallEventMs = -100000;
            stallWasActive = false;
            TestFailed = false;
            TestFailureReason = "";
            lastRoadModelEventMs = -100000;
            roadModelWasLow = false;
            lastPhysicalSurfaceEventMs = -100000;
            lastPhysicalSurfaceErrorMs = -100000;
            lastRefRawKappa = 0f;
            lastRefKappa = 0f;
            lastRefHeadStep = 0f;
            lastRefRoadClamp = 0;
            lastRefDetail = "";
            lastRoadReference = null;
            TargetSpeed = 0f;
            SpeedLimit = "Cruise";
            ActualSpeed = originSpeed;
            FinishGap = RaceMath.FlatDistance(origin, this.finish);
            maneuverPlanId = 0;
            hasCurrent = false;
            lastIntentLog = "";
            lastIntentLogMs = -100000;
            lastStabilityMode = "";
            lastStabilityEventMs = -100000;
            lastVizError = "";
            lastVizErrorMs = -100000;
            lastEgoFwd = RaceMath.VectorFromHeading(originHeading);
            lastEgoHeading = originHeading;

            desiredRoadSpeed = EffectiveCruise();
            commandedSpeed = RaceMath.Clamp(originSpeed, 0f, EffectiveCruise());
            commandedInit = true;
            prevPlanMs = t0;

            bool needJoin = Math.Abs(route.Lateral) > JoinLatThreshM
                || Math.Abs(route.HeadingErrorDeg) > JoinHeadThreshDeg;
            joined = !needJoin;
            joinState = IsGpsSource() ? (joined ? "Track" : "Join") : "GpsWait";

            Running = true;

            try
            {
                string poseChk = PoseConnector.SelfTest();
                string steerChk = DirectActuator.SteeringSignSelfTest();
                string headingChk = HeadingConventionCheck(vehicle);
                int tEv = 0;
                try { tEv = nowGame - t0; } catch { }
                telemetry?.Event(tEv, "ROUTE", $"src={route.Source};pts={route.Points.Count};len={route.TotalLength:F0};simpleCruise={EffectiveCruise():F0};poseCheck={poseChk};steerCheck={steerChk};headingCheck={headingChk};gpsOnly=1;recovery=ForwardOnly;controller=V2;spatialPlanner=V1+Gates;fromSnapshot=1");
                telemetry?.Event(tEv, "ACTUATOR", $"Direct;LayeredLocalWorld + SpatialPlannerV1+Gates + persistent cmd speed;iniPassing={enablePassing};gtaRejoin=OFF(override ini={useGtaRejoin});recovery=ForwardOnly;controller=V2");
                string vStart = "";
                try { vStart = route.ValidateStart(origin, originHeading, out string vr) ? $"valid;{vr}" : $"INVALID;{vr}"; }
                catch { vStart = "validate-exc"; }
                telemetry?.Event(tEv, "START_POSE", $"egoHead={originHeading:F0};routeHead={route.RouteHeadingDeg:F0};headErr={route.HeadingErrorDeg:F0};lat={route.Lateral:F1};dist={route.DistToRoute:F1};s={route.AlongS:F0};startValid={vStart};join={joinState};needJoin={needJoin};fromSnapshot=1;noSecondGps=1");
                if (!IsGpsSource())
                    telemetry?.Event(tEv, "SNAPSHOT_MISMATCH", $"non-gps after import src={route.Source};should-never-happen");
                if (poseChk != "OK")
                    telemetry?.Event(tEv, "POSE_CONNECTOR_FAIL", $"selftest={poseChk}");
                if (steerChk != "OK")
                    telemetry?.Event(tEv, "STEER_SIGN_FAIL", $"selftest={steerChk}");
                if (!headingChk.StartsWith("OK"))
                    telemetry?.Event(tEv, "HEADING_CONVENTION_FAIL", headingChk);
            }
            catch { }
        }

        /// GPS-only gate for Simple mode. FallbackWalk / StraightFallback are
        /// never valid control references (they step/snap toward the finish
        /// and can teleport progress while the car sits still).
        public bool IsGpsSource()
        {
            try { return route.Built && route.Source != null && route.Source.StartsWith("Gps"); }
            catch { return false; }
        }

        public bool Valid()
        {
            try { return Running && actuator != null && actuator.Valid(); }
            catch { return false; }
        }

        public void Stop()
        {
            Running = false;
            try { actuator?.Stop(); } catch { }
            try { physicalSurface.Reset(); } catch { }
        }

        private float EffectiveCruise()
        {
            float c = Math.Min(cruiseSetting, simpleCruiseCap);
            if (c < 5f) c = 5f;
            if (c > 25f) c = 25f;
            return c;
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
            float signedLongSpeed = RaceMath.FlatDot(new Vector3(egoVel.X, egoVel.Y, 0f), egoFwd);
            float forwardPlanSpeed = Math.Max(0f, signedLongSpeed);

            ActualSpeed = egoSpeed;
            FinishGap = RaceMath.FlatDistance(egoPos, finish);
            lastEgoFwd = egoFwd;
            lastEgoHeading = egoHeading;

            UpdateKinematics(now, egoPos, egoVel, egoSpeed, signedLongSpeed, egoHeading);

            if (!hasKin)
            {
                hasKin = true;
                lastSpeed = egoSpeed;
                lastSignedLong = signedLongSpeed;
                lastPos = egoPos;
                lastKinT = now;
                lastKinHeading = egoHeading;
                try { lastHealth = vehicle.HealthFloat; } catch { lastHealth = -1f; }
                return;
            }

            bool doPlan = now - lastPlanMs >= profile.ReactionIntervalMs;
            if (doPlan)
            {
                lastPlanMs = now;
                try { route.Update(egoPos, egoHeading, forwardPlanSpeed, now, corridor.HalfWidth); } catch { }

                // GPS-only: retry briefly, never drive on fallback.
                if (!IsGpsSource())
                {
                    joinState = "GpsWait";
                    try
                    {
                        if (now - t0 < 12000 && now - lastGpsRetryMs > 1000)
                        {
                            lastGpsRetryMs = now;
                            string ulog;
                            if (route.TryUpgradeToGps(egoPos, egoHeading, forwardPlanSpeed, now, corridor.HalfWidth, out ulog))
                            {
                                try { corridor.Update(route, egoPos, LookaheadM, now); } catch { }
                                try { telemetry?.Event(t, "GPS_ROUTE", $"upgraded;{ulog}"); } catch { }
                                try { hasCurrent = false; } catch { }
                                // Re-evaluate join latch on the fresh geometry.
                                bool needJoin = Math.Abs(route.Lateral) > JoinLatThreshM
                                    || Math.Abs(route.HeadingErrorDeg) > JoinHeadThreshDeg;
                                joined = !needJoin;
                                joinState = joined ? "Track" : "Join";
                                try { telemetry?.Event(t, "GPS_OK", $"src={route.Source};join={joinState};lat={route.Lateral:F1};headErr={route.HeadingErrorDeg:F0}"); } catch { }
                            }
                        }
                    }
                    catch { }
                    if (!IsGpsSource())
                    {
                        // HOLD: do not control the car from fallback geometry.
                        // Keep logging so the test FAILS visibly instead of
                        // driving off on a snapped walk.
                        SendHold(egoPos, egoFwd, "GpsMissing");
                        if (now - lastGpsMissingLogMs > 2000)
                        {
                            lastGpsMissingLogMs = now;
                            try { telemetry?.Event(t, "GPS_MISSING", $"src={route.Source};holding;never-fallback;elapsed={(now - t0) / 1000f:F0}s"); } catch { }
                        }
                        LogIntent(t, now);
                        // Fall through to actuator tick + telemetry below.
                        goto AfterPlan;
                    }
                }
            }

            // Route loss is now a recovery trigger, not an immediate test
            // abort. Keep timing it for diagnostics; recovery gets the chance
            // to change the pose before we declare the run unrecoverable.
            if (doPlan && IsGpsSource())
            {
                if (route.IsLost)
                {
                    if (routeLostSinceMs < 0) routeLostSinceMs = now;
                }
                else
                {
                    routeLostSinceMs = -1;
                }
            }

            if (now - lastCorrMs >= 200)
            {
                lastCorrMs = now;
                try { corridor.Update(route, egoPos, LookaheadM, now); } catch { }
            }

            // Persistent perception feeds the local 2.5D road-surface world model. Actor
            // prediction is evaluated per candidate in world space.
            try
            {
                if (IsGpsSource())
                {
                    float brakeCap = capability.ABrakeMax > 1f ? capability.ABrakeMax : 6f;
                    float reactionS = profile != null ? Math.Max(0.5f, profile.SafetyTimeS) : 1f;
                    perception.Update(vehicle, driver, null, null, forwardPlanSpeed,
                        brakeCap, reactionS, now, 120, route, corridor);
                }
            }
            catch { }

            if (doPlan && IsGpsSource())
            {
                bool trafficBlocked = false;
                try
                {
                    trafficBlocked = hasCurrent
                        && current.ConstrainHandle != -1
                        && current.TargetSpeed < 3.0f;
                }
                catch { }

                if (!recovery.Active)
                {
                    // Planner uncertainty is NEVER a recovery trigger. Recovery
                    // requires physical evidence that the car's pose/motion is
                    // actually bad.
                    bool severeRoutePose = false;
                    bool physicallyStuck = false;
                    try
                    {
                        severeRoutePose = route.IsLost
                            && routeLostSinceMs >= 0
                            && now - routeLostSinceMs >= 1000
                            && (route.DistToRoute > 8f || Math.Abs(route.HeadingErrorDeg) > 55f);
                        physicallyStuck = !trafficBlocked
                            && commandedSpeed > 4f
                            && forwardPlanSpeed < 0.7f
                            && stallSinceMs >= 0
                            && now - stallSinceMs >= 2500;
                    }
                    catch { }

                    if (trafficBlocked)
                    {
                        try { recovery.Reset(now); } catch { }
                    }
                    else if (severeRoutePose)
                    {
                        EnterRecovery("physical-route-loss:" + route.LossReason, now);
                    }
                    else if (physicallyStuck)
                    {
                        EnterRecovery("physical-stuck", now);
                    }
                }

                if (recovery.Active)
                {
                    bool poseRecovered = false;
                    try
                    {
                        poseRecovered = recovery.Current == RecoveryPrimitive.Stage.Rejoin
                            && !route.IsLost
                            && Math.Abs(route.Lateral) < 2.0f
                            && Math.Abs(route.HeadingErrorDeg) < 12f
                            && (now - recoveryEnteredMs) > 500;
                    }
                    catch { }

                    if (poseRecovered)
                    {
                        try { telemetry?.Event(t, "RECOVERY_EXIT", $"stage={recovery.Current};s={route.AlongS:F0};lat={route.Lateral:F1};headErr={route.HeadingErrorDeg:F0}"); } catch { }
                        try { recovery.Exit(now); } catch { }
                        try { spatialPlanner.Reset(); } catch { }
                        try { localWorld.Reset(); } catch { }
                        lastRecoveryStage = RecoveryPrimitive.Stage.Idle;
                        recoveryEnteredMs = -1;
                        plannerInvalidSinceMs = -1;
                        joined = true;
                        joinState = "Track";
                    }
                    else if (recoveryEnteredMs > 0 && now - recoveryEnteredMs > 18000)
                    {
                        TestFailed = true;
                        TestFailureReason = $"recovery-timeout;stage={recovery.Current};reason={recovery.Reason};dist={route.DistToRoute:F1};headErr={route.HeadingErrorDeg:F0}";
                        try { telemetry?.Event(t, "TEST_FAIL", TestFailureReason); } catch { }
                        SendHold(egoPos, egoFwd, "RecoveryTimeout");
                        goto AfterPlan;
                    }
                    else
                    {
                        ManeuverCommand rm = recovery.Tick(route, corridor, egoPos, egoFwd,
                            egoHeading, signedLongSpeed, route.AlongS, now, Math.Min(EffectiveCruise(), 6f));
                        rm.PlanId = ++maneuverPlanId;
                        joinState = "Recovery:" + recovery.Current;
                        TargetSpeed = Math.Max(0f, rm.TargetSpeed);
                        commandedSpeed = TargetSpeed;
                        commandedInit = true;
                        SpeedLimit = rm.Reason ?? joinState;
                        hasCurrent = false;
                        if (recovery.Current != lastRecoveryStage)
                        {
                            lastRecoveryStage = recovery.Current;
                            try { telemetry?.Event(t, "RECOVERY_STAGE", $"stage={recovery.Current};reason={recovery.Reason};s={route.AlongS:F0};lat={route.Lateral:F1};headErr={route.HeadingErrorDeg:F0}"); } catch { }
                        }
                        try { actuator.SetManeuver(rm); } catch { }
                        LogIntent(t, now);
                        goto AfterPlan;
                    }
                }

                LookaheadM = profile.LookaheadForSpeed(forwardPlanSpeed);
                float dtPlan = 0.1f;
                try
                {
                    if (prevPlanMs > 0)
                    {
                        dtPlan = (now - prevPlanMs) / 1000f;
                        if (dtPlan < 0.03f) dtPlan = 0.03f;
                        if (dtPlan > 0.5f) dtPlan = 0.5f;
                    }
                }
                catch { dtPlan = 0.1f; }
                prevPlanMs = now;

                // One-time startup join: only when NOT yet joined and still
                // misaligned. After that, spatial planning owns normal driving;
                // explicit recovery is a separate physical-failure mode.
                ManeuverCommand m;
                if (!joined)
                {
                    bool alignedNow = Math.Abs(route.Lateral) <= JoinedLatM
                        && Math.Abs(route.HeadingErrorDeg) <= JoinedHeadDeg;
                    if (alignedNow)
                    {
                        joined = true;
                        joinState = "Track";
                        try { telemetry?.Event(t, "JOIN_DONE", $"latched to Track;lat={route.Lateral:F1};headErr={route.HeadingErrorDeg:F0};s={route.AlongS:F0}"); } catch { }
                        m = BuildTrack(egoPos, forwardPlanSpeed, dtPlan);
                    }
                    else
                    {
                        joinState = "Join";
                        m = BuildJoin(egoPos, forwardPlanSpeed, dtPlan);
                    }
                }
                else
                {
                    joinState = "Track";
                    m = BuildTrack(egoPos, forwardPlanSpeed, dtPlan);
                }
                m.PlanId = ++maneuverPlanId;
                TargetSpeed = Math.Max(0f, commandedSpeed);
                // SpeedLimit names the ROAD limit, never the ramp lag.
                SpeedLimit = m.Reason ?? "Cruise";
                PublishViz(m, forwardPlanSpeed, t, egoHeading);
                try { actuator.SetManeuver(m); } catch { }
                LogIntent(t, now);

                // Lost/misaligned is telemetry only: never recover, never
                // reverse, never rejoin. The run must fail visibly if the
                // basic follower cannot hold the road.
                try
                {
                    if (route.IsLost && now - lastLostLogMs > 2000)
                    {
                        lastLostLogMs = now;
                        telemetry?.Event(t, "ROUTE_LOST", $"{route.LossReason};s={route.AlongS:F0};dist={route.DistToRoute:F1};headErr={route.HeadingErrorDeg:F0};recovery-enabled");
                    }
                    else if (!route.IsLost && lastLoggedLost)
                    {
                        try { telemetry?.Event(t, "ROUTE_FOUND", $"s={route.AlongS:F0}"); } catch { }
                    }
                    lastLoggedLost = route.IsLost;
                }
                catch { }
            }

        AfterPlan:
            try { actuator.OnTick(false, joinState); } catch { }
            try
            {
                actuator?.UpdatePathError(egoPos, egoHeading, forwardPlanSpeed, route,
                    hasCurrent ? current : new TrajectoryCandidate(), hasCurrent, TargetSpeed);
            }
            catch { }

            // Stability state comes from the actuator that has the command and
            // the physical response in the same tick. Event on transitions and
            // periodically while degraded so a failure is obvious without
            // reconstructing every CSV row.
            try
            {
                var peNow = actuator != null ? actuator.LastError : new PathFollowingError();
                string sm = peNow.Valid ? (peNow.StabilityMode ?? "") : "";
                if (!string.IsNullOrEmpty(sm)
                    && (sm != lastStabilityMode || (sm != "Normal" && now - lastStabilityEventMs > 2000)))
                {
                    lastStabilityMode = sm;
                    lastStabilityEventMs = now;
                    telemetry?.Event(t, "STABILITY",
                        $"mode={sm};vLong={peNow.SignedLongMps:F1};vLat={peNow.LateralVelMps:F1};"
                        + $"slip={peNow.SlipDeg:F1};yaw={peNow.YawRateDegS:F1};yawTgt={peNow.DesiredYawRateDegS:F1};"
                        + $"headErr={peNow.HeadingErrDeg:F1};latErr={peNow.LateralErrM:F1};"
                        + $"steer={peNow.SteerDeg:F1};satS={peNow.SteerSaturationS:F2};"
                        + $"thr={peNow.Throttle01:F2};brk={peNow.Brake01:F2}");
                }
            }
            catch { }

            try
            {
                DetectAndLogStall(now, t, egoPos);
            }
            catch { }

            // PhysicalSurfaceMap is deliberately observer-only in this pass.
            // Keeping it off the control path lets us judge the representation
            // visually before it can change race behavior.
            if (viz.Enabled)
            {
                try
                {
                    physicalSurface.Tick(
                        vehicle, lastRoadReference, route,
                        egoPos, egoHeading, now);

                    if (now - lastPhysicalSurfaceEventMs > 2000)
                    {
                        lastPhysicalSurfaceEventMs = now;
                        telemetry?.Event(t, "PHYSICAL_SURFACE",
                            SpatialBuildTag + ";" + physicalSurface.Detail);
                    }
                }
                catch (Exception ex)
                {
                    if (now - lastPhysicalSurfaceErrorMs > 2000)
                    {
                        lastPhysicalSurfaceErrorMs = now;
                        telemetry?.Event(t, "PHYSICAL_SURFACE_ERROR",
                            ex.GetType().Name + ":" + ex.Message);
                    }
                }
            }

            try
            {
                if (viz.Enabled)
                {
                    viz.Draw(route, corridor, trajViz, perception, speedViz,
                        lastRoadReference, localWorld, physicalSurface,
                        egoPos, lastEgoFwd, egoSpeed, LookaheadM, TargetSpeed);
                    if (!string.IsNullOrEmpty(viz.LastError)
                        && (viz.LastError != lastVizError || now - lastVizErrorMs > 3000))
                    {
                        lastVizError = viz.LastError;
                        lastVizErrorMs = now;
                        telemetry?.Event(t, "DEBUG_VIZ_ERROR", viz.LastError);
                    }
                }
            }
            catch (Exception ex)
            {
                if (now - lastVizErrorMs > 3000)
                {
                    lastVizErrorMs = now;
                    telemetry?.Event(t, "DEBUG_VIZ_ERROR", "outer:" + ex.Message);
                }
            }

            if (now - lastTeleMs >= 100)
            {
                lastTeleMs = now;
                WriteSample(t, egoPos, egoSpeed);
            }

            lastSpeed = egoSpeed;
            lastSignedLong = signedLongSpeed;
            lastPos = egoPos;
            lastKinT = now;
        }

        private void EnterRecovery(string reason, int now)
        {
            if (recovery.Active) return;
            try
            {
                recovery.Enter(reason, now, route.AlongS);
                recoveryEnteredMs = now;
                lastRecoveryStage = recovery.Current;
                plannerInvalidSinceMs = -1;
                joined = true;
                joinState = "Recovery:" + recovery.Current;
                spatialPlanner.Reset();
                localWorld.Reset();
                telemetry?.Event(now - t0, "RECOVERY_ENTER",
                    $"reason={reason};s={route.AlongS:F0};lat={route.Lateral:F1};headErr={route.HeadingErrorDeg:F0}");
            }
            catch { }
        }

        private void SendHold(Vector3 egoPos, Vector3 egoFwd, string why)
        {
            try
            {
                var holdPt = new Vector3(egoPos.X + egoFwd.X * 12f, egoPos.Y + egoFwd.Y * 12f, egoPos.Z);
                var hold = new ManeuverCommand
                {
                    Path = new List<Vector3> { egoPos, holdPt },
                    StationS = new List<float> { 0f, 12f },
                    SpeedProfile = new List<float> { 0f, 0f },
                    AimPoint = holdPt,
                    TargetSpeed = 0f,
                    Style = style,
                    Reason = why,
                    Reverse = false,
                    PlanId = maneuverPlanId + 1,
                };
                // Do not advance persistent commanded speed while holding for
                // GPS: the car must launch from rest once GPS arrives.
                TargetSpeed = 0f;
                SpeedLimit = why;
                hasCurrent = false;
                try { actuator.SetManeuver(hold); } catch { }
            }
            catch { }
        }

        // --- TRACK: executable local road reference.
        // GPS remains the global route/progress source, but Direct never sees
        // its raw 5 m zig-zags. DrivingReference creates the smooth local path.
        private ManeuverCommand BuildTrack(Vector3 egoPos, float egoSpeed, float dtPlan)
        {
            float cruise = EffectiveCruise();
            float look = LookaheadM;

            // Raw GPS is only the global/topological route. Build a separate
            // executable reference that removes short GPS zig-zags and rounds
            // junction vertices while staying on GTA's drivable road.
            DrivingReference.Result rr = null;
            try { rr = drivingReference.Build(route, route.AlongS, look, egoSpeed); }
            catch { rr = null; }

            if (rr == null || !rr.Valid || rr.Path == null || rr.Path.Count < 3)
            {
                int now = Game.GameTime;
                lastRefDetail = rr != null ? rr.Detail : "null";
                float routeRemain = Math.Max(0f, route.TotalLength - route.AlongS);
                bool terminalPhase = routeRemain <= 45f && FinishGap <= 100f;

                if (terminalPhase)
                {
                    try
                    {
                        telemetry?.Event(now - t0, "TERMINAL_SPATIAL",
                            $"reference-ended;routeRemain={routeRemain:F1};finishGap={FinishGap:F1};ref={lastRefDetail}");
                    }
                    catch { }
                    return BuildTerminalSpatial(egoPos, egoSpeed, dtPlan, cruise);
                }

                if (referenceInvalidSinceMs < 0)
                {
                    referenceInvalidSinceMs = now;
                    try { telemetry?.Event(now - t0, "REFERENCE_UNCERTAIN", lastRefDetail); } catch { }
                }
                // Perception uncertainty is not an obstacle. Preserve the last
                // known maneuver at a reduced speed instead of emergency stop
                // or reverse.
                joinState = "RoadUncertain";
                return BuildFailSoftFromCurrent(egoPos, Math.Min(6f, cruise), "RoadUncertain");
            }

            referenceInvalidSinceMs = -1;
            lastRoadReference = rr;
            lastRefRawKappa = rr.RawMaxKappa;
            lastRefKappa = rr.MaxKappa;
            lastRefHeadStep = rr.MaxHeadingStepDeg;
            lastRefRoadClamp = rr.RoadConstrainedPoints;
            lastRefDetail = rr.Detail;

            try
            {
                float minRoadConf = 1f;
                int lowCount = 0;
                int laneHint = 0;
                string sourceHint = "?";
                for (int i = 0; i < rr.RoadConfidence.Count; i++)
                {
                    float cf = rr.RoadConfidence[i];
                    if (cf < minRoadConf) minRoadConf = cf;
                    if (cf < 0.45f) lowCount++;
                    if (laneHint <= 0 && i < rr.RoadLaneCount.Count) laneHint = rr.RoadLaneCount[i];
                    if (sourceHint == "?" && i < rr.RoadSource.Count && !string.IsNullOrEmpty(rr.RoadSource[i]))
                        sourceHint = rr.RoadSource[i];
                }
                bool low = minRoadConf < 0.40f || lowCount > rr.RoadConfidence.Count / 3;
                int now = Game.GameTime;
                if (low != roadModelWasLow || (low && now - lastRoadModelEventMs > 2000))
                {
                    roadModelWasLow = low;
                    lastRoadModelEventMs = now;
                    telemetry?.Event(now - t0, low ? "ROAD_MODEL_LOW" : "ROAD_MODEL_OK",
                        $"minConf={minRoadConf:F2};low={lowCount}/{rr.RoadConfidence.Count};laneHint={laneHint};src={sourceHint};{rr.Detail}");
                }
            }
            catch { }

            SpatialPlannerV1.Result sp = null;
            try
            {
                // The physical observer owns the cell visualization now.
                // Do not also build the legacy LocalWorld debug grid every plan;
                // road supports themselves remain visible in RaceDebugViz.
                localWorld.Build(rr, perception, egoPos, lastEgoHeading, egoHalfLength, egoHalfWidth, false);
                sp = spatialPlanner.Plan(localWorld, route, capability, profile,
                    egoPos, lastEgoHeading, egoSpeed, lastYawRate,
                    cruise, Game.GameTime, finish);
            }
            catch { sp = null; }

            if (sp == null || !sp.Valid || sp.Chosen.Path == null || sp.Chosen.Path.Count < 3)
            {
                int now = Game.GameTime;
                string why = sp != null ? sp.Detail : "null";
                if (plannerInvalidSinceMs < 0)
                {
                    plannerInvalidSinceMs = now;
                    try { telemetry?.Event(now - t0, "SPATIAL_PLAN_UNCERTAIN", why); } catch { }
                }
                // Spatial search uncertainty is not proof of blockage. Keep
                // moving on the known smooth reference while the world model
                // rebuilds; never resurrect LocalPlannerV2 rails here.
                joinState = "SpatialUncertain";
                var latsFallback = new List<float>(rr.Path.Count);
                for (int i = 0; i < rr.Path.Count; i++) latsFallback.Add(0f);
                return BuildCommandFromPath(rr.Path, rr.StationS, latsFallback,
                    egoSpeed, dtPlan, Math.Min(cruise, 7f), "SpatialUncertain", egoPos);
            }

            plannerInvalidSinceMs = -1;
            referenceInvalidSinceMs = -1;
            joinState = sp.Intent;
            return BuildCommandFromCandidate(sp.Chosen, egoSpeed, dtPlan, cruise, sp.RoadDesired);
        }

        private ManeuverCommand BuildTerminalSpatial(
            Vector3 egoPos, float egoSpeed, float dtPlan, float cruise)
        {
            try
            {
                // No long DrivingReference is required here: nearby structural
                // road nodes + ordered route/finish gates are sufficient for
                // the final local search.
                localWorld.Build(null, perception, egoPos, lastEgoHeading,
                    egoHalfLength, egoHalfWidth, false);
                var sp = spatialPlanner.Plan(localWorld, route, capability, profile,
                    egoPos, lastEgoHeading, egoSpeed, lastYawRate,
                    cruise, Game.GameTime, finish);
                if (sp != null && sp.Valid && sp.Chosen.Path != null && sp.Chosen.Path.Count >= 3)
                {
                    referenceInvalidSinceMs = -1;
                    plannerInvalidSinceMs = -1;
                    joinState = "Terminal";
                    return BuildCommandFromCandidate(
                        sp.Chosen, egoSpeed, dtPlan, cruise, sp.RoadDesired);
                }
            }
            catch { }

            joinState = "TerminalUncertain";
            return BuildFailSoftFromCurrent(
                egoPos, Math.Min(5f, cruise), "TerminalUncertain");
        }

        private ManeuverCommand BuildFailSoftFromCurrent(Vector3 egoPos, float cap, string why)
        {
            try
            {
                if (hasCurrent && current.Path != null && current.Path.Count >= 2
                    && current.StationS != null && current.StationS.Count == current.Path.Count)
                {
                    float v = Math.Max(2.5f, Math.Min(cap, commandedSpeed > 0f ? commandedSpeed : cap));
                    var prof = new List<float>(current.Path.Count);
                    for (int i = 0; i < current.Path.Count; i++) prof.Add(v);
                    commandedSpeed = v;
                    commandedInit = true;
                    desiredRoadSpeed = v;
                    TargetSpeed = v;
                    SpeedLimit = why;
                    return new ManeuverCommand
                    {
                        Path = new List<Vector3>(current.Path),
                        StationS = new List<float>(current.StationS),
                        SpeedProfile = prof,
                        AimPoint = current.AimPoint,
                        TargetSpeed = v,
                        Style = style,
                        Reason = why,
                        Reverse = false,
                    };
                }
            }
            catch { }

            var p = new Vector3(
                egoPos.X + lastEgoFwd.X * 15f,
                egoPos.Y + lastEgoFwd.Y * 15f,
                egoPos.Z);
            float crawl = Math.Max(2.5f, Math.Min(cap, 4f));
            commandedSpeed = crawl;
            commandedInit = true;
            desiredRoadSpeed = crawl;
            TargetSpeed = crawl;
            SpeedLimit = why;
            return new ManeuverCommand
            {
                Path = new List<Vector3> { egoPos, p },
                StationS = new List<float> { 0f, 15f },
                SpeedProfile = new List<float> { crawl, crawl },
                AimPoint = p,
                TargetSpeed = crawl,
                Style = style,
                Reason = why,
                Reverse = false,
            };
        }

        private ManeuverCommand BuildPlannerStop(Vector3 egoPos, string why)
        {
            var p = new Vector3(
                egoPos.X + lastEgoFwd.X * 10f,
                egoPos.Y + lastEgoFwd.Y * 10f,
                egoPos.Z);
            commandedSpeed = 0f;
            commandedInit = true;
            TargetSpeed = 0f;
            SpeedLimit = why;
            hasCurrent = false;
            return new ManeuverCommand
            {
                Path = new List<Vector3> { egoPos, p },
                StationS = new List<float> { 0f, 10f },
                SpeedProfile = new List<float> { 0f, 0f },
                AimPoint = p,
                TargetSpeed = 0f,
                Style = style,
                Reason = why,
                Reverse = false,
            };
        }

        private ManeuverCommand BuildCommandFromCandidate(
            TrajectoryCandidate chosen, float egoSpeed, float dtPlan, float cruise, float roadDesired)
        {
            float aBrake = capability.UsableBrake(profile.GripFactor);
            float aAcc = Math.Max(2f, aBrake * 0.55f);
            float aDec = Math.Max(3f, aBrake);
            float desired = RaceMath.Clamp(chosen.TargetSpeed, 0f, cruise);
            desiredRoadSpeed = RaceMath.Clamp(roadDesired, 0f, cruise);

            if (!commandedInit)
            {
                commandedSpeed = RaceMath.Clamp(egoSpeed, 0f, cruise);
                commandedInit = true;
            }
            else if (desired > commandedSpeed)
            {
                commandedSpeed = Math.Min(desired, commandedSpeed + aAcc * dtPlan);
            }
            else if (desired < commandedSpeed)
            {
                commandedSpeed = Math.Max(desired, commandedSpeed - aDec * dtPlan);
            }
            commandedSpeed = RaceMath.Clamp(commandedSpeed, 0f, cruise);

            var raw = chosen.SpeedProfile ?? new List<float>();
            var ss = chosen.StationS ?? new List<float>();
            var prof = new List<float>(raw.Count);
            for (int i = 0; i < raw.Count; i++)
            {
                float s = i < ss.Count ? ss[i] : i * 4f;
                float reach = (float)Math.Sqrt(commandedSpeed * commandedSpeed
                    + 2f * aAcc * Math.Max(0f, s));
                float v = Math.Min(raw[i], reach);
                prof.Add(RaceMath.Clamp(v, 0f, cruise));
            }
            if (prof.Count == 0)
            {
                prof.Add(commandedSpeed);
                prof.Add(commandedSpeed);
            }
            prof[0] = commandedSpeed;

            string limit = chosen.SpeedLimiting ?? "Cruise";
            if ((limit == "Cruise" || string.IsNullOrEmpty(limit)) && commandedSpeed < desired - 0.5f)
                limit = "AccelRamp";

            current = chosen;
            current.SpeedProfile = prof;
            current.TargetSpeed = commandedSpeed;
            current.SpeedLimiting = limit;
            current.MeanSpeed = Mean(prof);
            current.MinSpeed = Min(prof, cruise);
            current.RequiredDecel = Math.Max(0f, egoSpeed - desired);
            hasCurrent = true;

            return new ManeuverCommand
            {
                Path = new List<Vector3>(current.Path),
                StationS = new List<float>(current.StationS),
                SpeedProfile = new List<float>(prof),
                AimPoint = current.AimPoint,
                TargetSpeed = commandedSpeed,
                Style = style,
                Reason = limit,
                Reverse = false,
            };
        }

        // --- JOIN: one-time pose-feasible merge, only until aligned.
        // Uses PoseConnector (correct -tan sign for GTA headings) with ego snap at path[0].
        // Guarded by VerifyToward so a sign regression holds instead of
        // driving a path that leaves away from the nose.
        private ManeuverCommand BuildJoin(Vector3 egoPos, float egoSpeed, float dtPlan)
        {
            float cruise = EffectiveCruise();
            float look = LookaheadM;
            float startLat = RaceMath.Clamp(route.Lateral, -18f, 18f);
            float headErr = route.HeadingErrorDeg;
            List<float> lats;
            List<float> ss;
            var path = PoseConnector.BuildPath(route, egoPos, startLat, headErr, 0f, look, 5f, out lats, out ss);
            float firstTang = PoseConnector.FirstTangentErrorDeg(path, lastEgoHeading);
            try
            {
                if (!PoseConnector.VerifyToward(lastEgoHeading, route.RouteHeadingDeg, firstTang))
                {
                    try { telemetry?.Event(Game.GameTime - t0, "POSE_CONNECTOR_FAIL", $"egoHead={lastEgoHeading:F0};routeHead={route.RouteHeadingDeg:F0};headErr={headErr:F0};firstTang={firstTang:F0}"); } catch { }
                    var hold = new List<Vector3> { egoPos, new Vector3(egoPos.X + lastEgoFwd.X * 12f, egoPos.Y + lastEgoFwd.Y * 12f, egoPos.Z) };
                    return new ManeuverCommand
                    {
                        Path = hold,
                        StationS = new List<float> { 0f, 12f },
                        SpeedProfile = new List<float> { Math.Min(cruise, 5f), Math.Min(cruise, 5f) },
                        AimPoint = hold[1],
                        TargetSpeed = Math.Min(cruise, 5f),
                        Style = style,
                        Reason = "PoseConnectorFail",
                        Reverse = false,
                    };
                }
            }
            catch { }
            var cmd = BuildCommandFromPath(path, ss, lats, egoSpeed, dtPlan, cruise, "Join", egoPos);
            // Tag first-tangent info for telemetry via current candidate.
            return cmd;
        }

        // Shared speed logic: curvature -> braking-feasible desired -> persistent
        // commanded (MoveTowards) -> forward-reachable profile. Actual speed
        // NEVER redefines desired or commanded.
        private ManeuverCommand BuildCommandFromPath(List<Vector3> path, List<float> ss, List<float> lats,
            float egoSpeed, float dtPlan, float cruise, string mode, Vector3 egoPos)
        {
            float aLat = capability.UsableLat(profile.GripFactor);
            float aBrake = capability.UsableBrake(profile.GripFactor);
            float top = 60f;
            try { top = capability.TopSpeedEst; } catch { }
            int n = path.Count;
            if (n < 2)
            {
                var hold = new List<Vector3> { egoPos, new Vector3(egoPos.X + lastEgoFwd.X * 12f, egoPos.Y + lastEgoFwd.Y * 12f, egoPos.Z) };
                return new ManeuverCommand
                {
                    Path = hold,
                    StationS = new List<float> { 0f, 12f },
                    SpeedProfile = new List<float> { 0f, 0f },
                    AimPoint = hold[1],
                    TargetSpeed = 0f,
                    Style = style,
                    Reason = mode + "-ShortPath",
                    Reverse = false,
                };
            }
            // First compute the road-only envelope. Keep this separate
            // from actor constraints so desRoad_mps continues to mean exactly
            // "what the road/curvature allows".
            var kappas = new List<float>(n);
            var roadAllow = new float[n];
            for (int i = 0; i < n; i++)
            {
                float k = CurvatureOfPathAt(path, i);
                kappas.Add(k);
                float vc = k < 1e-5f ? cruise : (float)Math.Sqrt(aLat / k);
                if (vc > cruise) vc = cruise;
                if (top > 5f && vc > top) vc = top;
                roadAllow[i] = vc;
            }

            var roadDes = new float[n];
            roadDes[n - 1] = roadAllow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float ds = Math.Max(1f, ss[i + 1] - ss[i]);
                float vr = (float)Math.Sqrt(roadDes[i + 1] * roadDes[i + 1] + 2f * aBrake * ds);
                roadDes[i] = Math.Min(roadAllow[i], vr);
            }
            float roadDesired = RaceMath.Clamp(roadDes[0], 0f, cruise);
            desiredRoadSpeed = roadDesired;

            // Then constrain THIS executable path by predicted actors. We reuse
            // the joint planner's swept-envelope logic but not its forward
            // actual-speed limiter: Simple's persistent commandedSpeed remains
            // the single acceleration authority (avoids the old actual+1 bug).
            float[] jointAllow;
            float[] plannerTarget;
            float[] plannerArrival;
            int constrainHandle;
            string constrainKind;
            float constrainS;
            float minPredClearance;
            SpeedPlanner.ProfileForPath(
                path, ss, kappas, lats,
                route.AlongS, perception, corridor,
                egoSpeed, cruise, aLat, aBrake, top,
                profile, cruise,
                out jointAllow, out plannerTarget, out plannerArrival,
                out constrainHandle, out constrainKind, out constrainS,
                out minPredClearance);

            // Re-run only the braking pass over the joint allow envelope.
            // This preserves future-obstacle braking while keeping acceleration
            // independent of measured actual speed.
            var vDes = new float[n];
            vDes[n - 1] = jointAllow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float ds = Math.Max(1f, ss[i + 1] - ss[i]);
                float vr = (float)Math.Sqrt(vDes[i + 1] * vDes[i + 1] + 2f * aBrake * ds);
                vDes[i] = Math.Min(jointAllow[i], vr);
            }
            float desired = RaceMath.Clamp(vDes[0], 0f, cruise);
            bool trafficLimited = constrainHandle != -1 && desired < roadDesired - 0.25f;

            // PERSISTENT commanded speed: ramp from PREVIOUS commanded toward
            // desired at physical limits. dtPlan is the real plan interval.
            // This is the fix for "actual+1" collapse: error = desired/actual
            // stays meaningful (e.g. 18 vs 10 -> +8) and Direct can pull.
            float aAcc = Math.Max(2f, aBrake * 0.55f);
            float aDec = Math.Max(3f, aBrake);
            if (!commandedInit)
            {
                commandedSpeed = RaceMath.Clamp(egoSpeed, 0f, cruise);
                commandedInit = true;
            }
            else
            {
                if (desired > commandedSpeed)
                {
                    float step = aAcc * dtPlan;
                    commandedSpeed = Math.Min(desired, commandedSpeed + step);
                }
                else if (desired < commandedSpeed)
                {
                    float step = aDec * dtPlan;
                    commandedSpeed = Math.Max(desired, commandedSpeed - step);
                }
            }
            if (commandedSpeed < 0f) commandedSpeed = 0f;
            if (commandedSpeed > cruise) commandedSpeed = cruise;

            // Final executable profile: first point IS the persistent command;
            // future points respect both the braking-feasible envelope and what
            // is forward-reachable from the command at full accel.
            var prof = new List<float>(n);
            for (int i = 0; i < n; i++)
            {
                float s = ss[i];
                float reach = (float)Math.Sqrt(commandedSpeed * commandedSpeed + 2f * aAcc * Math.Max(0f, s));
                float v = Math.Min(vDes[i], reach);
                // Never exceed cruise; never go negative.
                if (v > cruise) v = cruise;
                if (v < 0f) v = 0f;
                prof.Add(v);
            }
            // Guarantee prof[0] equals commanded exactly (no interpolation drift).
            prof[0] = commandedSpeed;

            float maxKappa = 0f;
            for (int i = 0; i < n; i++) { float k = CurvatureOfPathAt(path, i); if (k > maxKappa) maxKappa = k; }
            float firstTang = 0f;
            try
            {
                if (mode == "Join")
                    firstTang = PoseConnector.FirstTangentErrorDeg(path, lastEgoHeading);
            }
            catch { }

            // Name the real limiting layer. Traffic wins over curvature when
            // it is the reason the executable target is below the road target.
            string limit;
            if (trafficLimited)
                limit = $"Traffic:{(string.IsNullOrEmpty(constrainKind) ? "Actor" : constrainKind)}#{constrainHandle}";
            else if (roadDesired < cruise - 0.5f)
                limit = "Curvature";
            else if (commandedSpeed < desired - 0.5f)
                limit = "AccelRamp";
            else limit = "Cruise";

            current = new TrajectoryCandidate
            {
                LateralM = 0f,
                LookaheadM = LookaheadM,
                AimPoint = path.Count > 0 ? path[path.Count - 1] : egoPos,
                Score = 0f,
                ClearanceM = 999f,
                CurveCost = 0f,
                TacticalBias = 0f,
                RejectReason = "",
                Path = path,
                MinMarginM = MinMargin(lats, ss),
                MaxKappa = maxKappa,
                StationS = new List<float>(ss),
                SpeedProfile = new List<float>(prof),
                ArrivalT = plannerArrival != null ? new List<float>(plannerArrival) : new List<float>(ss.Count),
                TargetSpeed = commandedSpeed,
                SpeedLimiting = limit,
                ConstrainHandle = constrainHandle,
                ConstrainKind = constrainKind ?? "",
                ConstrainS = constrainS,
                MinPredClearance = minPredClearance,
                MeanSpeed = Mean(prof),
                MinSpeed = Min(prof, cruise),
                RequiredDecel = 0f,
                CandidateIndex = 0,
                FirstTangentErrDeg = firstTang,
                RouteHeadErrDeg = route.HeadingErrorDeg,
            };
            hasCurrent = true;
            return new ManeuverCommand
            {
                Path = new List<Vector3>(path),
                StationS = new List<float>(ss),
                SpeedProfile = new List<float>(prof),
                AimPoint = current.AimPoint,
                TargetSpeed = commandedSpeed,
                Style = style,
                Reason = limit,
                Reverse = false,
            };
        }

        private void PublishViz(ManeuverCommand m, float egoSpeed, int t, float egoHeading)
        {
            try
            {
                trajViz.LastCandidates.Clear();
                trajViz.DebugGateCenters.Clear();
                for (int gi = 0; gi < spatialPlanner.DebugGateCenters.Count; gi++)
                    trajViz.DebugGateCenters.Add(spatialPlanner.DebugGateCenters[gi]);

                if (joined && spatialPlanner.LastCandidates.Count > 0)
                {
                    for (int i = 0; i < spatialPlanner.LastCandidates.Count; i++)
                        trajViz.LastCandidates.Add(spatialPlanner.LastCandidates[i]);
                }
                else if (hasCurrent)
                {
                    trajViz.LastCandidates.Add(current);
                }
                trajViz.Chosen = current;
                trajViz.HasChosen = hasCurrent;
                trajViz.PlanId = maneuverPlanId;
                trajViz.BrakingPointS = -1f;
                speedViz.TargetSpeed = TargetSpeed;
                speedViz.Limiting = SpeedLimit;
                speedViz.ProfileS.Clear();
                speedViz.ProfileAllowed.Clear();
                speedViz.ProfileTarget.Clear();
                if (m.SpeedProfile != null)
                {
                    for (int i = 0; i < m.SpeedProfile.Count; i++)
                    {
                        float s = (m.StationS != null && i < m.StationS.Count) ? m.StationS[i] : i * 5f;
                        speedViz.ProfileS.Add(s);
                        speedViz.ProfileAllowed.Add(m.SpeedProfile[i]);
                        speedViz.ProfileTarget.Add(m.SpeedProfile[i]);
                    }
                }
                speedViz.BrakingPointS = -1f;
            }
            catch { }
        }

        private void LogIntent(int t, int now)
        {
            try
            {
                string key = joinState + "|" + TargetSpeed.ToString("F0") + "|" + SpeedLimit;
                if (key != lastIntentLog || now - lastIntentLogMs > 4000)
                {
                    lastIntentLog = key;
                    lastIntentLogMs = now;
                    var c = hasCurrent ? current : new TrajectoryCandidate();
                    telemetry?.Event(t, "INTENT",
                        $"intent={joinState};desRoad={desiredRoadSpeed:F1};cmd={commandedSpeed:F1};v={TargetSpeed:F1};lim={SpeedLimit};"
                        + $"egoHead={lastEgoHeading:F0};routeHead={route.RouteHeadingDeg:F0};"
                        + $"headErr={route.HeadingErrorDeg:F0};firstTang={c.FirstTangentErrDeg:F1};"
                        + $"maxKappa={c.MaxKappa:F4};rawGpsK={lastRefRawKappa:F4};refK={lastRefKappa:F4};"
                        + $"refHeadStep={lastRefHeadStep:F0};roadClamp={lastRefRoadClamp};"
                        + $"constr={(c.ConstrainHandle != -1 ? (c.ConstrainKind ?? "Actor") + "#" + c.ConstrainHandle + "@" + c.ConstrainS.ToString("F0") : "none")};"
                        + $"minClear={c.MinPredClearance:F1};latTarget={c.LateralM:F1};score={c.Score:F1};"
                        + $"roadModel={lastRefDetail};world={localWorld.Detail};spatial={spatialPlanner.LastDecision};s={route.AlongS:F0};{route.LocDetail}");
                }
            }
            catch { }
        }

        private void UpdateKinematics(int now, Vector3 egoPos, Vector3 egoVel, float egoSpeed, float signedLongSpeed, float egoHeading)
        {
            if (!hasKin) return;
            float dtS = (now - lastKinT) / 1000f;
            if (dtS <= 0f || dtS > 0.6f) return;
            float accel = (signedLongSpeed - lastSignedLong) / dtS;
            lastAccelLong = accel;
            float dhDeg = RaceMath.HeadingDiffDeg(egoHeading, lastKinHeading);
            float yawRate = 0f;
            try { yawRate = dhDeg * (float)Math.PI / 180f / dtS; } catch { }
            lastYawRate = yawRate;
            float latA = Math.Max(0f, signedLongSpeed) * yawRate;
            float slip = 0f;
            try
            {
                float vFlat = RaceMath.FlatLength(new Vector3(egoVel.X, egoVel.Y, 0f));
                if (vFlat > 2f && egoSpeed > 2f)
                {
                    var velDir = RaceMath.FlatNormalize(new Vector3(egoVel.X, egoVel.Y, 0f));
                    Vector3 fwd;
                    try { fwd = RaceMath.FlatNormalize(new Vector3(vehicle.ForwardVector.X, vehicle.ForwardVector.Y, 0f)); }
                    catch { fwd = RaceMath.VectorFromHeading(egoHeading); }
                    slip = RaceMath.SignedAngleDeg(fwd, velDir);
                }
            }
            catch { }
            lastSlipDeg = slip;
            lastKinHeading = egoHeading;

            // Capability learning WITHOUT ImpactClassifier-driven behavior:
            // gate only on the capability's own stability (slip/yaw) and sane
            // dt/speed. No Crashed/recovery coupling here.
            try
            {
                if (signedLongSpeed > 4f)
                    capability.Observe(accel, latA, signedLongSpeed, dtS, yawRate, slip);
            }
            catch { }
            try
            {
                float h = vehicle.HealthFloat;
                if (h >= 0f) lastHealth = h;
            }
            catch { }
        }

        private void WriteSample(int t, Vector3 egoPos, float egoSpeed)
        {
            if (telemetry == null) return;
            try
            {
                float offCorr = corridor.OffCorridor(route.Lateral);
                float curv = route.CurvatureAhead(80f);
                var c = hasCurrent ? current : new TrajectoryCandidate();
                var pe = actuator != null ? actuator.LastError : new PathFollowingError();
                // Local pursuit point Direct is actually chasing (sEgo + ld on
                // the persistent reference), not the distant final aim.
                // For TRACK this is ~route.PointAtS(AlongS + ld) when aligned.
                Vector3 lookPt = Vector3.Zero;
                try
                {
                    float ld = pe.Valid ? pe.LookaheadM : RaceMath.Clamp(6f + egoSpeed * 0.7f, 8f, 28f);
                    if (hasCurrent && current.Path != null && current.Path.Count >= 2
                        && current.StationS != null && current.StationS.Count == current.Path.Count)
                        lookPt = PointAtLocalS(current.Path, current.StationS, ld);
                    else
                        lookPt = route.PointAtS(route.AlongS + ld);
                }
                catch { try { lookPt = hasCurrent ? current.AimPoint : egoPos; } catch { } }
                int gear = 0;
                int nextGear = 0;
                float rpm = 0f;
                int gtaTrafficLight = 0;
                int burnout = 0;
                int seatOk = 0;
                try { gear = vehicle.CurrentGear; } catch { }
                try { nextGear = vehicle.NextGear; } catch { }
                try { rpm = vehicle.CurrentRPM; } catch { }
                try { gtaTrafficLight = vehicle.IsStoppedAtTrafficLights ? 1 : 0; } catch { }
                try { burnout = vehicle.IsInBurnout ? 1 : 0; } catch { }
                try
                {
                    var vd = vehicle.Driver;
                    seatOk = vd != null && vd.Exists() && driver != null && driver.Exists()
                        && vd.Handle == driver.Handle ? 1 : 0;
                }
                catch { }

                telemetry.Sample(t, style, joinState,
                    route.AlongS, route.Progress01, LookaheadM,
                    route.Lateral, corridor.HalfWidth, offCorr, route.HeadingErrorDeg, curv,
                    c.LateralM, c.Score, 0f, 0f,
                    commandedSpeed, egoSpeed, SpeedLimit ?? "Cruise", 0f,
                    capability.ABrakeMax, capability.ALatMax, perception.Count,
                    perception.NearestAheadDist, perception.NearestAheadTtc, perception.NearestAheadClosing,
                    actuator.CurrentCruise, actuator.CurrentStyle,
                    route.IsLost ? 1 : 0, "Normal",
                    actuator.ReissueCount, FinishGap,
                    route.Source ?? "?", corridor.MinHalfWidthAhead(LookaheadM),
                    pe.Valid ? pe.LateralErrM : 0f, pe.Valid ? pe.HeadingErrDeg : 0f,
                    pe.Valid ? pe.SpeedErrMps : 0f, pe.Valid ? pe.DistToPathM : 999f,
                    -1f, capability.Confidence,
                    actuator.ActuatorName ?? "?", c.RejectReason ?? "",
                    c.MinMarginM, c.MaxKappa,
                    c.CandidateIndex, c.MeanSpeed, c.MinSpeed,
                    c.ConstrainHandle, c.ConstrainKind ?? "", c.ConstrainS,
                    c.MinPredClearance, maneuverPlanId,
                    pe.Valid ? pe.SteerDeg : 0f, pe.Valid ? pe.Throttle01 : 0f,
                    pe.Valid ? pe.Brake01 : 0f, pe.Valid ? pe.LocalTargetMps : commandedSpeed,
                    lastEgoHeading, route.RouteHeadingDeg, c.FirstTangentErrDeg,
                    route.ExpectedS, route.LocJumpM,
                    desiredRoadSpeed, commandedSpeed, lookPt.X, lookPt.Y, joinState,
                    pe.Valid ? pe.SignedLongMps : 0f,
                    pe.Valid ? pe.LateralVelMps : 0f,
                    pe.Valid ? pe.SlipDeg : lastSlipDeg,
                    pe.Valid ? pe.YawRateDegS : lastYawRate * 180f / (float)Math.PI,
                    pe.Valid ? pe.DesiredYawRateDegS : 0f,
                    pe.Valid ? pe.SteerActualDeg : 0f,
                    pe.Valid ? pe.ThrottleActual01 : 0f,
                    pe.Valid ? pe.ThrottlePowerActual01 : 0f,
                    pe.Valid ? pe.BrakeActual01 : 0f,
                    pe.Valid ? pe.SteerSaturationS : 0f,
                    pe.Valid ? pe.StabilityMode : "",
                    gear, nextGear, rpm,
                    gtaTrafficLight, burnout, seatOk,
                    c.OpposingFraction, c.UnknownFraction, c.MeanFlowCost,
                    c.ElevationDeltaM, c.RouteGatesPassed, c.SurfaceComponentId,
                    c.GateMissCost, spatialPlanner.LastPlanMs,
                    pe.Valid ? pe.RawDesiredYawRateDegS : 0f);
            }
            catch { }
        }

        private void DetectAndLogStall(int now, int t, Vector3 egoPos)
        {
            if (actuator == null || vehicle == null || !vehicle.Exists()) return;
            var pe = actuator.LastError;
            if (!pe.Valid) return;

            bool candidate = pe.ThrottleActual01 >= 0.85f
                && pe.BrakeActual01 <= 0.10f
                && Math.Abs(pe.SignedLongMps) < 1.0f
                && pe.LocalTargetMps >= 4f;

            if (!candidate)
            {
                if (stallWasActive)
                {
                    try { telemetry?.Event(t, "STALL_END", $"duration={(now - stallSinceMs) / 1000f:F1}s;vLong={pe.SignedLongMps:F1}"); } catch { }
                }
                stallSinceMs = -1;
                stallWasActive = false;
                return;
            }

            if (stallSinceMs < 0) stallSinceMs = now;
            if (now - stallSinceMs < 600) return;

            bool first = !stallWasActive;
            stallWasActive = true;
            if (!first && now - lastStallEventMs < 2000) return;
            lastStallEventMs = now;

            int gear = 0;
            int nextGear = 0;
            float rpm = 0f;
            bool burnout = false;
            bool gtaTrafficLight = false;
            bool seatOk = false;
            try { gear = vehicle.CurrentGear; } catch { }
            try { nextGear = vehicle.NextGear; } catch { }
            try { rpm = vehicle.CurrentRPM; } catch { }
            try { burnout = vehicle.IsInBurnout; } catch { }
            try { gtaTrafficLight = vehicle.IsStoppedAtTrafficLights; } catch { }
            try
            {
                var vd = vehicle.Driver;
                seatOk = vd != null && vd.Exists() && driver != null && driver.Exists() && vd.Handle == driver.Handle;
            }
            catch { }

            int density = -1;
            int nodeFlags = 0;
            bool nodeOk = false;
            try
            {
                var dOut = new OutputArgument();
                var fOut = new OutputArgument();
                nodeOk = Function.Call<bool>(Hash.GET_VEHICLE_NODE_PROPERTIES,
                    egoPos.X, egoPos.Y, egoPos.Z, dOut, fOut);
                if (nodeOk)
                {
                    density = dOut.GetResult<int>();
                    nodeFlags = fOut.GetResult<int>();
                }
            }
            catch { }

            bool nodeJunction = (nodeFlags & (1 << 7)) != 0;
            bool nodeTrafficLight = (nodeFlags & (1 << 8)) != 0;
            bool nodeGiveWay = (nodeFlags & (1 << 9)) != 0;

            TrackedActor lead = new TrackedActor();
            bool hasLead = false;
            try { hasLead = perception.TryGetLeadOnRoute(out lead, route.AlongS, corridor, 25f); } catch { }
            string leadText = hasLead
                ? $"kind={lead.Kind};handle={lead.Handle};routeDist={lead.RouteDist:F1};dist={lead.Dist:F1};lat={lead.RouteLateral:F1};leadV={lead.SpeedAlong:F1}"
                : "none";

            string detail =
                $"age={(now - stallSinceMs) / 1000f:F1}s;vLong={pe.SignedLongMps:F2};vLat={pe.LateralVelMps:F2};"
                + $"target={pe.LocalTargetMps:F1};thr={pe.ThrottleActual01:F2};thrP={pe.ThrottlePowerActual01:F2};"
                + $"brk={pe.BrakeActual01:F2};gear={gear};nextGear={nextGear};rpm={rpm:F2};"
                + $"burnout={(burnout ? 1 : 0)};gtaTrafficLight={(gtaTrafficLight ? 1 : 0)};seatOk={(seatOk ? 1 : 0)};"
                + $"nodeOk={(nodeOk ? 1 : 0)};density={density};junction={(nodeJunction ? 1 : 0)};"
                + $"nodeTrafficLight={(nodeTrafficLight ? 1 : 0)};giveWay={(nodeGiveWay ? 1 : 0)};"
                + $"lead={leadText};actors={perception.Count}";
            try { telemetry?.Event(t, first ? "STALL_BEGIN" : "STALL", detail); } catch { }
        }

        private static Vector3 PointAtLocalS(List<Vector3> path, List<float> ss, float s)
        {
            if (path == null || path.Count == 0) return Vector3.Zero;
            if (ss == null || ss.Count != path.Count) return path[path.Count - 1];
            if (s <= 0f) return path[0];
            if (s >= ss[ss.Count - 1]) return path[path.Count - 1];
            for (int i = 0; i < ss.Count - 1; i++)
            {
                if (s >= ss[i] && s <= ss[i + 1])
                {
                    float ds = ss[i + 1] - ss[i];
                    float t = ds > 1e-4f ? (s - ss[i]) / ds : 0f;
                    return new Vector3(
                        path[i].X + (path[i + 1].X - path[i].X) * t,
                        path[i].Y + (path[i + 1].Y - path[i].Y) * t,
                        path[i].Z + (path[i + 1].Z - path[i].Z) * t);
                }
            }
            return path[path.Count - 1];
        }

        private static float CurvatureOfPathAt(List<Vector3> path, int k)
        {
            try
            {
                if (path == null || path.Count < 3) return 0f;
                int i0 = Math.Max(0, k - 1);
                int i1 = k;
                int i2 = Math.Min(path.Count - 1, k + 1);
                if (i0 == i1 || i1 == i2) return 0f;
                var d0 = new Vector3(path[i1].X - path[i0].X, path[i1].Y - path[i0].Y, 0f);
                var d1 = new Vector3(path[i2].X - path[i1].X, path[i2].Y - path[i1].Y, 0f);
                float l0 = RaceMath.FlatLength(d0);
                float l1 = RaceMath.FlatLength(d1);
                if (l0 < 0.5f || l1 < 0.5f) return 0f;
                float dh = Math.Abs(RaceMath.SignedAngleDeg(d0, d1)) * (float)Math.PI / 180f;
                return dh / Math.Max((l0 + l1) * 0.5f, 1f);
            }
            catch { return 0f; }
        }

        private float MinMargin(List<float> lats, List<float> ss)
        {
            try
            {
                float m = 99f;
                for (int i = 0; i < lats.Count; i++)
                {
                    float half = corridor.HalfWidthAt(ss[i]);
                    float margin = half - Math.Abs(lats[i]);
                    if (margin < m) m = margin;
                }
                return m;
            }
            catch { return 99f; }
        }

        private static float Mean(List<float> v)
        {
            if (v == null || v.Count == 0) return 0f;
            float s = 0f;
            foreach (var x in v) s += x;
            return s / v.Count;
        }

        private static float Min(List<float> v, float fb)
        {
            if (v == null || v.Count == 0) return fb;
            float m = float.MaxValue;
            foreach (var x in v) if (x < m) m = x;
            return m;
        }

        private static string HeadingConventionCheck(Vehicle v)
        {
            try
            {
                if (v == null || !v.Exists()) return "FAIL no-vehicle";
                var nativeFwd = RaceMath.FlatNormalize(new Vector3(v.ForwardVector.X, v.ForwardVector.Y, 0f));
                var fromHeading = RaceMath.VectorFromHeading(v.Heading);
                float dot = RaceMath.FlatDot(nativeFwd, fromHeading);
                float vecHeading = RaceMath.HeadingFromVector(nativeFwd);
                float err = Math.Abs(RaceMath.HeadingDiffDeg(vecHeading, v.Heading));
                if (dot < 0.98f || err > 5f)
                    return $"FAIL dot={dot:F3};vehHead={v.Heading:F1};vecHead={vecHeading:F1};err={err:F1}";
                return $"OK dot={dot:F3};err={err:F1}";
            }
            catch (Exception ex) { return "FAIL exc:" + ex.Message; }
        }

        private static float SafeHeading(Vehicle v)
        {
            try { return v.Heading; } catch { return 0f; }
        }
    }
}
