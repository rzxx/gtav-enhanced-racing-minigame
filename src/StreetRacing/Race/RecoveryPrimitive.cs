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
    /// Explicit action sequence:
    ///   Stop -> Reverse (controlled distance, explicit Reverse=true) ->
    ///   Reposition/Forward (crawl toward a heading-compatible future segment)
    ///   -> Rejoin (pose-feasible connector to that segment).
    /// Reverse is commanded here via ManeuverCommand.Reverse, never inferred
    /// inside DirectActuator. If no merge exists, the primitive CRAWLS to
    /// change pose (never holds 0 indefinitely). GTA DriveTo may be used ONLY
    /// as an emergency low-speed rejoin tool when configured, never as the
    /// normal driver.
    internal sealed class RecoveryPrimitive
    {
        public enum Stage { Idle, Stop, Reverse, Forward, Rejoin }

        public Stage Current = Stage.Idle;
        public string Reason = "";
        public int SinceMs;
        public bool Active => Current != Stage.Idle;

        private int stopUntil;
        private float reverseStartS;
        private float reverseTargetM = 8f;

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
            Vector3 egoPos, Vector3 egoFwd, float egoHeading, float egoSpeed,
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
                            // If we have a merge behind us or are nose-into an
                            // obstacle, back up a controlled distance first.
                            // Otherwise go straight to Forward (no pointless reverse).
                            bool needReverse = !haveMerge || Math.Abs(route.HeadingErrorDeg) > 60f || egoSpeed < 0.5f;
                            if (needReverse)
                            {
                                Current = Stage.Reverse;
                                SinceMs = nowMs;
                                reverseStartS = alongS;
                                Reason = "reverse:" + mergeDetail;
                            }
                            else
                            {
                                Current = Stage.Forward;
                                SinceMs = nowMs;
                                Reason = "forward:" + mergeDetail;
                            }
                        }
                        return hold;

                    case Stage.Reverse:
                        {
                            // Controlled reverse: 8 m at ~3 m/s, steering toward
                            // the merge bearing (inverted by Direct for reverse).
                            // Distance measured by AlongS change OR time fallback.
                            float backed = Math.Abs(alongS - reverseStartS);
                            float heldS = (nowMs - SinceMs) / 1000f;
                            if (backed >= reverseTargetM || heldS > 6f)
                            {
                                Current = Stage.Forward;
                                SinceMs = nowMs;
                                Reason = "reverse-done";
                                goto case Stage.Forward;
                            }
                            // Reverse path: straight back along -ego heading.
                            var back = new Vector3(egoPos.X - egoFwd.X * 12f, egoPos.Y - egoFwd.Y * 12f, egoPos.Z);
                            return new ManeuverCommand
                            {
                                Path = new List<Vector3> { egoPos, back },
                                StationS = new List<float> { 0f, 12f },
                                SpeedProfile = new List<float> { 3f, 3f },
                                AimPoint = back,
                                TargetSpeed = 3f,
                                Reason = "Recovery:Reverse",
                                Reverse = true, // EXPLICIT: only place this is set
                            };
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
                            // No merge: slow forward crawl along the nose so the
                            // pose changes and a merge can appear. NEVER 0.
                            float crawl = Math.Min(recCruise, 4f);
                            var fwd = new Vector3(egoPos.X + egoFwd.X * 15f, egoPos.Y + egoFwd.Y * 15f, egoPos.Z);
                            if ((nowMs - SinceMs) > 8000)
                            {
                                // Still nothing after 8 s of crawling: stay in
                                // Forward (caller may escalate to GTA rejoin).
                                Reason = "crawl-no-merge";
                            }
                            return new ManeuverCommand
                            {
                                Path = new List<Vector3> { egoPos, fwd },
                                StationS = new List<float> { 0f, 15f },
                                SpeedProfile = new List<float> { crawl, crawl },
                                AimPoint = fwd,
                                TargetSpeed = crawl,
                                Reason = "Recovery:Forward-Crawl",
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
