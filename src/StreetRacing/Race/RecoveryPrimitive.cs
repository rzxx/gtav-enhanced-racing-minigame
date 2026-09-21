using System;
using System.Collections.Generic;
using GTA.Math;
using StreetRacing.Control;

namespace StreetRacing.Race
{
    /// Explicit recovery primitives (Phase 3), owned by the Simple driver.
    ///
    /// Retired: Crashed-as-normal-state, decel-only crash classification, and
    /// the deadlock where "no feasible merge -> targetSpeed=0 forever" left
    /// the pose unchanged so it never became mergeable.
    ///
    /// Entry requires corroborated evidence (not decel alone):
    ///   - actual collision flag + strong decel, or damage drop, or
    ///   - severe off-route state (IsLost), or heading-incompatible route, or
    ///   - sustained lack of progress (low speed + no AlongS advance).
    ///
    /// Explicit action sequence for the current safety pass:
    ///   Stop -> Forward/Reposition -> Rejoin.
    /// Reverse recovery is temporarily disabled because two independent GTA
    /// runs hard-froze while the reverse primitive was active.
    /// Reverse is commanded here via ManeuverCommand.Reverse, never inferred
    /// inside DirectActuator. If no merge exists, the primitive CRAWLS to
    /// change pose (never holds 0 indefinitely). GTA DriveTo may be used ONLY
    /// as an emergency low-speed rejoin tool when configured, never as the
    /// normal driver.
    internal sealed class RecoveryPrimitive
    {
        public enum Stage { Idle, Stop, Reverse, ReverseSettle, Forward, Rejoin }

        public Stage Current = Stage.Idle;
        public string Reason = "";
        public int SinceMs;
        public bool Active => Current != Stage.Idle;

        private int stopUntil;
        private float reverseStartS;
        private float reverseTargetM = 6f;

        // Lack-of-progress detector state (corroborated stuck, not one sample).
        private float progressMarkS = -9999f;
        private int progressMarkMs;

        public void Reset(int nowMs)
        {
            Current = Stage.Idle;
            Reason = "";
            SinceMs = nowMs;
            stopUntil = 0;
            reverseStartS = 0f;
            progressMarkS = -9999f;
            progressMarkMs = nowMs;
        }

        /// Should recovery take over? Corroborated evidence only.
        public bool ShouldEnter(RaceRoute route, float egoSpeed, float alongS,
            bool hasCollided, float healthDrop, float accelLong, int nowMs)
        {
            if (Active) return true;
            try
            {
                // Severe off-route / heading states are structural, not tuning.
                if (route != null && route.Built)
                {
                    if (route.IsLost) return true;
                    if (Math.Abs(route.HeadingErrorDeg) > RaceRoute.PlanInvalidHeadErrDeg) return true;
                }
                // Corroborated contact: collision flag or damage + strong decel.
                if ((hasCollided && accelLong < -8f) || (healthDrop >= 4f && accelLong < -3f))
                    return true;
                // Sustained lack of progress: slow + no AlongS advance.
                if (progressMarkS < -9000f) { progressMarkS = alongS; progressMarkMs = nowMs; return false; }
                if (egoSpeed < 1.5f && (nowMs - progressMarkMs) > 4000 && (alongS - progressMarkS) < 2f)
                    return true;
                if ((nowMs - progressMarkMs) > 4000) { progressMarkS = alongS; progressMarkMs = nowMs; }
            }
            catch { }
            return false;
        }

        public void Enter(string reason, int nowMs, float alongS)
        {
            Current = Stage.Stop;
            Reason = reason ?? "recover";
            SinceMs = nowMs;
            stopUntil = nowMs + 800;
            reverseStartS = alongS;
        }

        public void Exit(int nowMs)
        {
            Current = Stage.Idle;
            Reason = "";
            SinceMs = nowMs;
            try { progressMarkS = -9999f; } catch { }
        }

        /// Build the recovery maneuver for this tick. Never returns a
        /// permanent zero-speed hold: Forward/Rejoin always move so the pose
        /// changes and can become mergeable.
        public ManeuverCommand Tick(RaceRoute route, RoadCorridor corridor,
            Vector3 egoPos, Vector3 egoFwd, float egoHeading, float signedLongMps,
            float alongS, int nowMs, float recCruise)
        {
            var hold = new ManeuverCommand
            {
                Path = new List<Vector3> { egoPos, new Vector3(egoPos.X + egoFwd.X * 10f, egoPos.Y + egoFwd.Y * 10f, egoPos.Z) },
                StationS = new List<float> { 0f, 10f },
                SpeedProfile = new List<float> { 0f, 0f },
                AimPoint = new Vector3(egoPos.X + egoFwd.X * 10f, egoPos.Y + egoFwd.Y * 10f, egoPos.Z),
                TargetSpeed = 0f,
                Reason = "Recovery:" + Current,
                Reverse = false,
            };
            try
            {
                // Find a heading-compatible future merge (may fail).
                Vector3 mergePt = route.RecoveryTarget();
                float mergeS = alongS + 40f;
                string mergeDetail = "none";
                bool haveMerge = false;
                try { haveMerge = route.TryGetRecoveryMerge(egoPos, egoHeading, out mergePt, out mergeS, out mergeDetail); }
                catch { haveMerge = false; }

                switch (Current)
                {
                    case Stage.Stop:
                        if (nowMs >= stopUntil)
                        {
                            // Reverse recovery is intentionally disabled for
                            // now. Two telemetry runs hard-froze GTA while the
                            // reverse stage was active; a slow forward recovery
                            // is preferable to forcing a full game restart.
                            if (haveMerge)
                            {
                                Current = Stage.Rejoin;
                                SinceMs = nowMs;
                                Reason = "rejoin-direct:" + mergeDetail;
                            }
                            else
                            {
                                Current = Stage.Forward;
                                SinceMs = nowMs;
                                Reason = "forward-seek-merge:" + mergeDetail;
                            }
                        }
                        return hold;

                    case Stage.Reverse:
                    case Stage.ReverseSettle:
                        {
                            Current = Stage.Forward;
                            SinceMs = nowMs;
                            Reason = "reverse-disabled-safety";
                            goto case Stage.Forward;
                        }

                    case Stage.Forward:
                        {
                            // Crawl forward to change pose even with no merge.
                            // With a merge, aim a pose-feasible connector at it.
                            if (haveMerge)
                            {
                                Current = Stage.Rejoin;
                                SinceMs = nowMs;
                                Reason = "rejoin:" + mergeDetail;
                                goto case Stage.Rejoin;
                            }
                            // No merge: move TOWARD the route, not blindly along
                            // whatever direction the nose happens to point.
                            float crawl = Math.Min(recCruise, 3.5f);
                            Vector3 target = route.RecoveryTarget();
                            Vector3 toTarget = RaceMath.FlatNormalize(new Vector3(
                                target.X - egoPos.X, target.Y - egoPos.Y, 0f));
                            float ahead = RaceMath.FlatDot(toTarget, egoFwd);

                            // If the useful route target is behind, make a
                            // conservative forward arc instead of reversing.
                            // This is intentionally dumb-but-safe until recovery
                            // is folded into the spatial planner itself.
                            float egoWeight = ahead < -0.35f ? 0.85f : 0.45f;
                            float targetWeight = 1f - egoWeight;
                            if (ahead < -0.35f) crawl = Math.Min(crawl, 2.2f);
                            Vector3 blended = RaceMath.FlatNormalize(new Vector3(
                                egoFwd.X * egoWeight + toTarget.X * targetWeight,
                                egoFwd.Y * egoWeight + toTarget.Y * targetWeight, 0f));
                            var guide = new Vector3(
                                egoPos.X + blended.X * 10f,
                                egoPos.Y + blended.Y * 10f,
                                egoPos.Z);
                            float targetDist = RaceMath.FlatDistance(egoPos, target);
                            var path = new List<Vector3> { egoPos, guide };
                            var ss = new List<float> { 0f, RaceMath.FlatDistance(egoPos, guide) };
                            if (targetDist > 12f && targetDist < 45f)
                            {
                                float acc = ss[ss.Count - 1] + RaceMath.FlatDistance(guide, target);
                                path.Add(target);
                                ss.Add(acc);
                            }
                            if ((nowMs - SinceMs) > 8000)
                                Reason = "seek-merge-timeout";

                            var prof = new List<float>();
                            for (int i = 0; i < path.Count; i++) prof.Add(crawl);
                            return new ManeuverCommand
                            {
                                Path = path,
                                StationS = ss,
                                SpeedProfile = prof,
                                AimPoint = path[path.Count - 1],
                                TargetSpeed = crawl,
                                Reason = "Recovery:Forward-ToRoute",
                                Reverse = false,
                            };
                        }

                    case Stage.Rejoin:
                        {
                            float lookahead = mergeS - alongS;
                            if (lookahead < 15f) lookahead = 15f;
                            if (lookahead > 80f) lookahead = 80f;
                            float startLat = RaceMath.Clamp(route.Lateral, -18f, 18f);
                            float headErr = route.HeadingErrorDeg;
                            List<float> lats;
                            List<float> ss;
                            var path = PoseConnector.BuildPath(route, egoPos, startLat, headErr,
                                0f, lookahead, 5f, out lats, out ss);
                            if (path == null || path.Count < 3)
                            {
                                // Degenerate: fall back to crawl, never hold-0.
                                Current = Stage.Forward;
                                SinceMs = nowMs;
                                goto case Stage.Forward;
                            }
                            float kappa = TrajectoryPlanner.MaxCurvatureOf(path);
                            if (kappa > 0.25f)
                            {
                                // U-turn-like: crawl to change pose instead.
                                Current = Stage.Forward;
                                SinceMs = nowMs;
                                Reason = $"infeasible-kappa {kappa:F3}";
                                goto case Stage.Forward;
                            }
                            float v = Math.Min(recCruise, 6f);
                            var prof = new List<float>(path.Count);
                            var st = new List<float>(ss);
                            for (int i = 0; i < path.Count; i++) prof.Add(v);
                            return new ManeuverCommand
                            {
                                Path = path,
                                StationS = st,
                                SpeedProfile = prof,
                                AimPoint = path[path.Count - 1],
                                TargetSpeed = v,
                                Reason = "Recovery:Rejoin",
                                Reverse = false,
                            };
                        }

                    default:
                        return hold;
                }
            }
            catch { return hold; }
        }
    }
}
