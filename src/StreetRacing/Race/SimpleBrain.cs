using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;
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
    ///   -> persistent acceleration-limited commanded speed
    ///      (MoveTowards(prevCommanded, desired, limit*dt), NEVER actual+1)
    ///   -> Direct steering/throttle/brake.
    ///
    /// Explicitly DISABLED by construction for this milestone (do not re-add):
    ///   Perception / civilian traffic avoidance / passing (FOLLOW/PASS) /
    ///   player race tactics / candidate trajectory scoring / seven lateral
    ///   choices / Crashed state / ImpactClassifier-driven behavior /
    ///   RecoveryPrimitive / GTA DriveTo fallback / FallbackWalk /
    ///   StraightFallback. If the NPC hits traffic, that is acceptable: we are
    ///   testing route-following competence, not avoidance.
    ///
    /// Speed separation (the recursive bug this fixes):
    ///   desiredRoadSpeed = road allows (cruise + curvature + braking distance).
    ///   commandedSpeed  = persistent ramp toward desired (accel/decel limits).
    ///   actualSpeed     = vehicle.Speed (measured, NEVER feeds desired).
    ///   localTarget     = SpeedAtS(profile, sEgo) ~= commandedSpeed.
    /// Telemetry exposes all four separately. SpeedLimit is "Curvature" only
    /// when the ROAD caps speed; when the ramp lags behind desired it is
    /// "AccelRamp", never mislabelled as curvature.
    ///
    /// Path separation (the ego-anchored bug this fixes):
    ///   TRACK path geometry is anchored to the ROUTE (PointAtS(AlongS + s)),
    ///   so cross-track error stays meaningful (2 m left reads ~+2 m).
    ///   The old KEEP_LINE rebuilt a Hermite through ego every 100 ms, which
    ///   zeroed the error by construction and hid drift. PoseConnector is used
    ///   ONLY for the initial JOIN when start pose is offset/misaligned; once
    ///   aligned we latch to TRACK permanently. No rejoin, no recovery: if the
    ///   follower stops, leaves the road, misaligns or crashes, the test must
    ///   FAIL with logs, not hide behind reverse/rejoin.
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
        private float lastRefRawKappa;
        private float lastRefKappa;
        private float lastRefHeadStep;
        private int lastRefRoadClamp;
        private string lastRefDetail = "";
        private Vector3 lastEgoFwd = new Vector3(0f, 1f, 0f);
        private float lastEgoHeading;

        // --- Milestone persistent state (never reconstructed from actualSpeed).
        private float desiredRoadSpeed = 18f;
        private float commandedSpeed;
        private bool commandedInit;
        private int prevPlanMs = -1;

        // --- One-time join latch. Once TRACK, never go back (no recovery).
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
            try { trajViz.Reset(); } catch { }
            try { speedViz.Reset(); } catch { }

            Vector3 origin;
            try { origin = vehicle.Position; } catch { origin = Game.Player.Character.Position; }
            float originHeading = SafeHeading(vehicle);
            float originSpeed = 0f;
            try { originSpeed = vehicle.Speed; } catch { }
            route.Build(origin, finish);
            capability.Seed(vehicle);
            LookaheadM = this.profile.LookaheadForSpeed(0f);
            try { corridor.Update(route, origin, LookaheadM, t0); } catch { }
            try { route.Update(origin, originHeading, 0f, t0, corridor.HalfWidth); } catch { }

            actuator = new DirectActuator();
            actuator.Attach(driver, vehicle, EffectiveCruise(), style, refreshMs, stuckMs);
            viz.Enabled = debugViz;

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
            TestFailed = false;
            TestFailureReason = "";
            lastRefRawKappa = 0f;
            lastRefKappa = 0f;
            lastRefHeadStep = 0f;
            lastRefRoadClamp = 0;
            lastRefDetail = "";
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
                telemetry?.Event(0, "ROUTE", $"src={route.Source};pts={route.Points.Count};len={route.TotalLength:F0};simpleCruise={EffectiveCruise():F0};poseCheck={poseChk};steerCheck={steerChk};headingCheck={headingChk};gpsOnly=1;recovery=OFF;passing=OFF");
                telemetry?.Event(0, "ACTUATOR", $"Direct;milestone GPS-centerline follower (route-anchored, persistent cmd speed);passing=OFF(override ini={enablePassing});gtaRejoin=OFF(override ini={useGtaRejoin});recovery=OFF");
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
            LookaheadM = this.profile.LookaheadForSpeed(0f);
            try { corridor.Update(route, origin, LookaheadM, nowGame); } catch { }
            try { route.Update(origin, originHeading, 0f, nowGame, corridor.HalfWidth); } catch { }

            actuator = new DirectActuator();
            actuator.Attach(driver, vehicle, EffectiveCruise(), style, refreshMs, stuckMs);
            viz.Enabled = debugViz;

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
            TestFailed = false;
            TestFailureReason = "";
            lastRefRawKappa = 0f;
            lastRefKappa = 0f;
            lastRefHeadStep = 0f;
            lastRefRoadClamp = 0;
            lastRefDetail = "";
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
                telemetry?.Event(tEv, "ROUTE", $"src={route.Source};pts={route.Points.Count};len={route.TotalLength:F0};simpleCruise={EffectiveCruise():F0};poseCheck={poseChk};steerCheck={steerChk};headingCheck={headingChk};gpsOnly=1;recovery=OFF;passing=OFF;fromSnapshot=1");
                telemetry?.Event(tEv, "ACTUATOR", $"Direct;milestone GPS-centerline follower (route-anchored, persistent cmd speed);passing=OFF(override ini={enablePassing});gtaRejoin=OFF(override ini={useGtaRejoin});recovery=OFF");
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

            // Test-mode invariant: once localization is persistently lost,
            // stop the experiment instead of letting Track brute-force walls.
            // Recovery will be a separate, explicit subsystem later.
            if (doPlan && IsGpsSource())
            {
                if (route.IsLost)
                {
                    if (routeLostSinceMs < 0) routeLostSinceMs = now;
                    if (!TestFailed && now - routeLostSinceMs >= 1200)
                    {
                        TestFailed = true;
                        TestFailureReason = $"route-lost {route.LossReason};dist={route.DistToRoute:F1};headErr={route.HeadingErrorDeg:F0};s={route.AlongS:F0}";
                        joinState = "RouteLostHold";
                        try { telemetry?.Event(t, "TEST_FAIL", TestFailureReason); } catch { }
                        SendHold(egoPos, egoFwd, "RouteLostHold");
                    }
                }
                else
                {
                    routeLostSinceMs = -1;
                }

                if (TestFailed)
                {
                    joinState = "RouteLostHold";
                    SendHold(egoPos, egoFwd, "RouteLostHold");
                    goto AfterPlan;
                }
            }

            if (now - lastCorrMs >= 200)
            {
                lastCorrMs = now;
                try { corridor.Update(route, egoPos, LookaheadM, now); } catch { }
            }

            if (doPlan && IsGpsSource())
            {
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

                // One-time join: only when NOT yet joined and still misaligned.
                // Once joined, stay in TRACK forever (no rejoin, no recovery).
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
                        telemetry?.Event(t, "ROUTE_LOST", $"{route.LossReason};s={route.AlongS:F0};dist={route.DistToRoute:F1};headErr={route.HeadingErrorDeg:F0};TRACK-ONLY(no-recovery)");
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
                if (viz.Enabled)
                    viz.Draw(route, corridor, trajViz, null, speedViz, egoPos, lastEgoFwd, egoSpeed, LookaheadM, TargetSpeed);
            }
            catch { }

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

        // --- TRACK: persistent GPS centerline reference.
        // Geometry is anchored to the ROUTE (PointAtS(AlongS + s)), never
        // rebuilt through ego. path[0] is the route center at current AlongS,
        // so Direct's ClosestOnPath reports the TRUE cross-track error.
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
                lastRefDetail = rr != null ? rr.Detail : "null";
                try { telemetry?.Event(Game.GameTime - t0, "REFERENCE_INVALID", lastRefDetail); } catch { }
                var holdPt = new Vector3(egoPos.X + lastEgoFwd.X * 12f, egoPos.Y + lastEgoFwd.Y * 12f, egoPos.Z);
                return new ManeuverCommand
                {
                    Path = new List<Vector3> { egoPos, holdPt },
                    StationS = new List<float> { 0f, 12f },
                    SpeedProfile = new List<float> { 0f, 0f },
                    AimPoint = holdPt,
                    TargetSpeed = 0f,
                    Style = style,
                    Reason = "ReferenceInvalid",
                    Reverse = false,
                };
            }

            lastRefRawKappa = rr.RawMaxKappa;
            lastRefKappa = rr.MaxKappa;
            lastRefHeadStep = rr.MaxHeadingStepDeg;
            lastRefRoadClamp = rr.RoadConstrainedPoints;
            lastRefDetail = rr.Detail;

            var lats = new List<float>(rr.Path.Count);
            for (int i = 0; i < rr.Path.Count; i++) lats.Add(0f);
            return BuildCommandFromPath(rr.Path, rr.StationS, lats, egoSpeed, dtPlan, cruise, "Track", egoPos);
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
            var vAllow = new float[n];
            for (int i = 0; i < n; i++)
            {
                float k = CurvatureOfPathAt(path, i);
                float vc = k < 1e-5f ? cruise : (float)Math.Sqrt(aLat / k);
                if (vc > cruise) vc = cruise;
                if (top > 5f && vc > top) vc = top;
                vAllow[i] = vc;
            }
            // Backwards braking pass: desired at ego already accounts for bends
            // ahead, so the car brakes BEFORE the bend, not inside it.
            var vDes = new float[n];
            vDes[n - 1] = vAllow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float ds = Math.Max(1f, ss[i + 1] - ss[i]);
                float vr = (float)Math.Sqrt(vDes[i + 1] * vDes[i + 1] + 2f * aBrake * ds);
                vDes[i] = Math.Min(vAllow[i], vr);
            }
            float desired = vDes[0];
            if (desired < 0f) desired = 0f;
            if (desired > cruise) desired = cruise;
            desiredRoadSpeed = desired;

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

            // SpeedLimit names the ROAD vs RAMP bottleneck honestly:
            // Curvature only when the road itself caps; AccelRamp when the
            // persistent command still lags behind a higher desired.
            string limit;
            if (desired < cruise - 0.5f) limit = "Curvature";
            else if (commandedSpeed < desired - 0.5f) limit = "AccelRamp";
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
                ArrivalT = new List<float>(ss.Count),
                TargetSpeed = commandedSpeed,
                SpeedLimiting = limit,
                ConstrainHandle = -1,
                ConstrainKind = "",
                ConstrainS = -1f,
                MinPredClearance = 999f,
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
                if (hasCurrent) trajViz.LastCandidates.Add(current);
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
                        + $"s={route.AlongS:F0};{route.LocDetail}");
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
                try { gear = vehicle.CurrentGear; } catch { }
                try { nextGear = vehicle.NextGear; } catch { }
                try { rpm = vehicle.CurrentRPM; } catch { }

                telemetry.Sample(t, style, joinState,
                    route.AlongS, route.Progress01, LookaheadM,
                    route.Lateral, corridor.HalfWidth, offCorr, route.HeadingErrorDeg, curv,
                    c.LateralM, c.Score, 0f, 0f,
                    commandedSpeed, egoSpeed, SpeedLimit ?? "Cruise", 0f,
                    capability.ABrakeMax, capability.ALatMax, 0,
                    999f, 999f, 0f,
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
                    gear, nextGear, rpm);
            }
            catch { }
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
