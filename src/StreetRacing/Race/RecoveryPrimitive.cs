using System;
using System.Collections.Generic;
using GTA.Math;
using StreetRacing.Control;

namespace StreetRacing.Race
{
    /// Explicit low-speed recovery owned by the Simple driver.
    ///
    /// Safety invariant: Tick may perform at most ONE stage transition and
    /// always returns to the game loop. No goto/recursion between Forward and
    /// Rejoin is allowed. A previous Forward -> Rejoin -> Forward goto cycle
    /// could repeat forever in one script tick when a merge existed but its
    /// connector was infeasible, hard-freezing GTA.
    ///
    /// Current safety pass:
    ///   Stop -> Forward/Reposition -> Rejoin.
    /// Reverse remains disabled until recovery is folded into the spatial
    /// planner and can be tested without risking full-game freezes.
    internal sealed class RecoveryPrimitive
    {
        public enum Stage { Idle, Stop, Reverse, ReverseSettle, Forward, Rejoin }

        public Stage Current = Stage.Idle;
        public string Reason = "";
        public int SinceMs;
        public bool Active => Current != Stage.Idle;

        private int stopUntil;
        private int nextRejoinAttemptMs;

        // Lack-of-progress detector state (corroborated stuck, not one sample).
        private float progressMarkS = -9999f;
        private int progressMarkMs;

        public void Reset(int nowMs)
        {
            Current = Stage.Idle;
            Reason = "";
            SinceMs = nowMs;
            stopUntil = 0;
            nextRejoinAttemptMs = nowMs;
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
                if (route != null && route.Built)
                {
                    if (route.IsLost) return true;
                    if (Math.Abs(route.HeadingErrorDeg) > RaceRoute.PlanInvalidHeadErrDeg) return true;
                }
                if ((hasCollided && accelLong < -8f) || (healthDrop >= 4f && accelLong < -3f))
                    return true;

                if (progressMarkS < -9000f)
                {
                    progressMarkS = alongS;
                    progressMarkMs = nowMs;
                    return false;
                }
                if (egoSpeed < 1.5f && (nowMs - progressMarkMs) > 4000
                    && (alongS - progressMarkS) < 2f)
                    return true;
                if ((nowMs - progressMarkMs) > 4000)
                {
                    progressMarkS = alongS;
                    progressMarkMs = nowMs;
                }
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
            nextRejoinAttemptMs = nowMs;
        }

        public void Exit(int nowMs)
        {
            Current = Stage.Idle;
            Reason = "";
            SinceMs = nowMs;
            nextRejoinAttemptMs = nowMs;
            try { progressMarkS = -9999f; } catch { }
        }

        public ManeuverCommand Tick(RaceRoute route, RoadCorridor corridor,
            Vector3 egoPos, Vector3 egoFwd, float egoHeading, float signedLongMps,
            float alongS, int nowMs, float recCruise)
        {
            ManeuverCommand hold = BuildHold(egoPos, egoFwd);
            try
            {
                Vector3 mergePt = route.RecoveryTarget();
                float mergeS = alongS + 40f;
                string mergeDetail = "none";
                bool haveMerge = false;
                try
                {
                    haveMerge = route.TryGetRecoveryMerge(
                        egoPos, egoHeading, out mergePt, out mergeS, out mergeDetail);
                }
                catch
                {
                    haveMerge = false;
                    mergeDetail = "merge-query-exc";
                }

                switch (Current)
                {
                    case Stage.Stop:
                        if (nowMs < stopUntil)
                            return hold;

                        // Transition only; execute the new stage next planning
                        // tick. This keeps every Tick finite and observable.
                        if (haveMerge)
                        {
                            Current = Stage.Rejoin;
                            SinceMs = nowMs;
                            nextRejoinAttemptMs = nowMs;
                            Reason = "rejoin-direct:" + mergeDetail;
                        }
                        else
                        {
                            Current = Stage.Forward;
                            SinceMs = nowMs;
                            nextRejoinAttemptMs = nowMs + 350;
                            Reason = "forward-seek-merge:" + mergeDetail;
                        }
                        return hold;

                    case Stage.Reverse:
                    case Stage.ReverseSettle:
                        // Old telemetry can still put us here after hot reload.
                        // Never execute reverse in the safety build.
                        Current = Stage.Forward;
                        SinceMs = nowMs;
                        nextRejoinAttemptMs = nowMs + 350;
                        Reason = "reverse-disabled-safety";
                        return BuildForward(route, egoPos, egoFwd, nowMs, recCruise);

                    case Stage.Forward:
                        if (haveMerge && nowMs >= nextRejoinAttemptMs)
                        {
                            ManeuverCommand rejoin;
                            string reject;
                            if (TryBuildRejoin(route, egoPos, alongS, mergeS,
                                recCruise, out rejoin, out reject))
                            {
                                Current = Stage.Rejoin;
                                SinceMs = nowMs;
                                Reason = "rejoin:" + mergeDetail;
                                return rejoin;
                            }

                            // Crucial: do NOT jump back into Stage.Forward and
                            // immediately retry the same Rejoin in this tick.
                            // Crawl for a while so the pose can actually change.
                            nextRejoinAttemptMs = nowMs + 650;
                            Reason = "rejoin-reject:" + reject;
                        }

                        return BuildForward(route, egoPos, egoFwd, nowMs, recCruise);

                    case Stage.Rejoin:
                        if (!haveMerge)
                        {
                            Current = Stage.Forward;
                            SinceMs = nowMs;
                            nextRejoinAttemptMs = nowMs + 650;
                            Reason = "merge-lost:" + mergeDetail;
                            return BuildForward(route, egoPos, egoFwd, nowMs, recCruise);
                        }

                        {
                            ManeuverCommand rejoin;
                            string reject;
                            if (TryBuildRejoin(route, egoPos, alongS, mergeS,
                                recCruise, out rejoin, out reject))
                                return rejoin;

                            Current = Stage.Forward;
                            SinceMs = nowMs;
                            nextRejoinAttemptMs = nowMs + 650;
                            Reason = "rejoin-reject:" + reject;
                            return BuildForward(route, egoPos, egoFwd, nowMs, recCruise);
                        }

                    default:
                        return hold;
                }
            }
            catch (Exception ex)
            {
                try { Reason = "tick-exc:" + ex.Message; } catch { }
                return hold;
            }
        }

        private static ManeuverCommand BuildHold(Vector3 egoPos, Vector3 egoFwd)
        {
            Vector3 aim = new Vector3(
                egoPos.X + egoFwd.X * 10f,
                egoPos.Y + egoFwd.Y * 10f,
                egoPos.Z);
            return new ManeuverCommand
            {
                Path = new List<Vector3> { egoPos, aim },
                StationS = new List<float> { 0f, 10f },
                SpeedProfile = new List<float> { 0f, 0f },
                AimPoint = aim,
                TargetSpeed = 0f,
                Reason = "Recovery:Stop",
                Reverse = false,
            };
        }

        private ManeuverCommand BuildForward(
            RaceRoute route, Vector3 egoPos, Vector3 egoFwd,
            int nowMs, float recCruise)
        {
            float crawl = Math.Min(recCruise, 3.5f);
            Vector3 target = route.RecoveryTarget();
            Vector3 toTarget = RaceMath.FlatNormalize(new Vector3(
                target.X - egoPos.X, target.Y - egoPos.Y, 0f));
            float ahead = RaceMath.FlatDot(toTarget, egoFwd);

            // If the useful route target is behind, do not attempt a heroic
            // U-turn. Make a conservative forward arc and let another planning
            // tick reconsider the topology.
            float egoWeight = ahead < -0.35f ? 0.85f : 0.45f;
            float targetWeight = 1f - egoWeight;
            if (ahead < -0.35f)
                crawl = Math.Min(crawl, 2.2f);

            Vector3 blended = RaceMath.FlatNormalize(new Vector3(
                egoFwd.X * egoWeight + toTarget.X * targetWeight,
                egoFwd.Y * egoWeight + toTarget.Y * targetWeight, 0f));
            if (RaceMath.FlatLength(blended) < 0.2f)
                blended = egoFwd;

            Vector3 guide = new Vector3(
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

            string r = Reason;
            if ((nowMs - SinceMs) > 8000)
                r = "seek-merge-timeout";

            var prof = new List<float>(path.Count);
            for (int i = 0; i < path.Count; i++)
                prof.Add(crawl);

            return new ManeuverCommand
            {
                Path = path,
                StationS = ss,
                SpeedProfile = prof,
                AimPoint = path[path.Count - 1],
                TargetSpeed = crawl,
                Reason = string.IsNullOrEmpty(r)
                    ? "Recovery:Forward-ToRoute"
                    : "Recovery:Forward-ToRoute;" + r,
                Reverse = false,
            };
        }

        private static bool TryBuildRejoin(
            RaceRoute route, Vector3 egoPos, float alongS, float mergeS,
            float recCruise, out ManeuverCommand command, out string reject)
        {
            command = new ManeuverCommand();
            reject = "unknown";
            try
            {
                float lookahead = mergeS - alongS;
                if (lookahead < 15f) lookahead = 15f;
                if (lookahead > 80f) lookahead = 80f;

                float startLat = RaceMath.Clamp(route.Lateral, -18f, 18f);
                float headErr = route.HeadingErrorDeg;
                List<float> lats;
                List<float> ss;
                var path = PoseConnector.BuildPath(
                    route, egoPos, startLat, headErr,
                    0f, lookahead, 5f, out lats, out ss);

                if (path == null || path.Count < 3)
                {
                    reject = "degenerate-path";
                    return false;
                }

                float kappa = TrajectoryPlanner.MaxCurvatureOf(path);
                if (kappa > 0.25f)
                {
                    reject = $"kappa={kappa:F3}";
                    return false;
                }

                float v = Math.Min(recCruise, 6f);
                var prof = new List<float>(path.Count);
                for (int i = 0; i < path.Count; i++)
                    prof.Add(v);

                command = new ManeuverCommand
                {
                    Path = path,
                    StationS = new List<float>(ss),
                    SpeedProfile = prof,
                    AimPoint = path[path.Count - 1],
                    TargetSpeed = v,
                    Reason = "Recovery:Rejoin",
                    Reverse = false,
                };
                reject = "";
                return true;
            }
            catch (Exception ex)
            {
                try { reject = "exc:" + ex.Message; } catch { reject = "exc"; }
                return false;
            }
        }
    }
}
