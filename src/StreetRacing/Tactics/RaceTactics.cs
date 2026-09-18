using System;
using GTA.Math;

namespace StreetRacing.Tactics
{
    internal enum TacticalMode
    {
        Cruise,
        Follow,
        AttackSetup,
        OvertakeLeft,
        OvertakeRight,
        Commit,
        Abort,
        SideBySide,
        Defend,
        CornerPrep,
        Recovery,
        Crashed,
    }

    /// Racing behaviour: follow / attack / overtake L-R / commit-abort /
    /// side-by-side / defend / corner prep / crash-recovery.
    ///
    /// Awareness is preserved while rules are not: overtakes may cross lanes
    /// (the trajectory planner owns the full road width), but every commit
    /// requires a predicted-clear lane with TTC margin — aborts are first
    /// class, not failures.
    internal sealed class RaceTactics
    {
        public TacticalMode Mode = TacticalMode.Cruise;
        public TacticalMode PrevMode = TacticalMode.Cruise;
        public string Reason = "init";
        public int SinceMs;
        public float DesiredLateral; // -1..+1, planner bias
        public int OvertakeSide;     // -1 right, +1 left, 0 none
        public bool ChangedThisTick;

        private int commitSince;
        private int abortCooldownUntil;

        public void Update(
            RaceRoute route,
            RoadCorridor corridor,
            Perception perception,
            Vector3 egoPos,
            Vector3 egoFwd,
            float egoSpeed,
            Vector3 rivalPos,
            Vector3 rivalVel,
            float rivalDist,
            bool rivalIsAhead,
            float curveLimit,
            float cruise,
            int nowMs,
            DriverProfile profile,
            bool justImpacted)
        {
            ChangedThisTick = false;
            TacticalMode want = Mode;
            string reason = Reason;
            float wantLat = 0f;
            int wantSide = 0;

            var toRival = new Vector3(rivalPos.X - egoPos.X, rivalPos.Y - egoPos.Y, 0f);
            float rLong = RaceMath.FlatDot(toRival, egoFwd);
            float rLat = RaceMath.FlatCross(egoFwd, toRival);
            bool alongside = Math.Abs(rLong) < 9f && Math.Abs(rLat) < 4f && rivalDist < 14f;

            // Priority 0: crash / route loss.
            if (justImpacted && egoSpeed < 5f)
            {
                want = TacticalMode.Crashed;
                reason = "impact+slow";
            }
            else if (route.IsLost)
            {
                want = TacticalMode.Recovery;
                reason = "route:" + route.LossReason;
            }
            else if (Mode == TacticalMode.Crashed && egoSpeed > 6f && nowMs - SinceMs > 1200)
            {
                want = TacticalMode.Recovery;
                reason = "regained-motion";
            }
            else if (Mode == TacticalMode.Recovery && !route.IsLost)
            {
                want = TacticalMode.Cruise;
                reason = "route-reacquired";
            }
            else if (Mode == TacticalMode.Crashed && nowMs - SinceMs > 5000)
            {
                want = TacticalMode.Recovery;
                reason = "crash-timeout";
            }
            else
            {
                // Priority 1: sharp corner ahead suspends attacks.
                bool sharpCorner = curveLimit < cruise * 0.62f && route.CurvatureAhead(80f) > 0.004f;
                if (sharpCorner && (Mode == TacticalMode.Cruise || Mode == TacticalMode.Follow ||
                    Mode == TacticalMode.AttackSetup || Mode == TacticalMode.CornerPrep))
                {
                    want = TacticalMode.CornerPrep;
                    reason = "curvature";
                    // Take the inside line for the corner.
                    float h0 = route.HeadingAhead(0f);
                    float h1 = route.HeadingAhead(80f);
                    wantLat = RaceMath.Clamp(RaceMath.HeadingDiffDeg(h1, h0) / 25f, -0.7f, 0.7f);
                }
                else if (Mode == TacticalMode.CornerPrep && !sharpCorner)
                {
                    want = rivalIsAhead && rivalDist < 60f ? TacticalMode.Follow : TacticalMode.Cruise;
                    reason = "corner-clear";
                }
                else if (Mode == TacticalMode.SideBySide && nowMs - SinceMs > 4000)
                {
                    // Timeout first (before the alongside latch below):
                    // rubbing alongside forever (e.g. both stopped at race
                    // start) must resolve into a racing mode, not a
                    // permanent hold. Re-evaluate from Follow/Cruise.
                    want = rivalIsAhead && rivalDist < 60f ? TacticalMode.Follow : TacticalMode.Cruise;
                    reason = "side-by-side-timeout";
                    wantLat = 0f;
                }
                else if (alongside)
                {
                    want = TacticalMode.SideBySide;
                    reason = rLat > 0 ? "rival-left" : "rival-right";
                    // Hold your side, don't squeeze: bias away slightly.
                    wantLat = rLat > 0 ? -0.35f : 0.35f;
                }
                else if (Mode == TacticalMode.SideBySide && !alongside)
                {
                    want = rivalIsAhead ? TacticalMode.Follow : TacticalMode.Cruise;
                    reason = "clear-of-rival";
                }
                // Priority 2: defend when the rival is closing from behind.
                else if (!rivalIsAhead && rivalDist < 22f && egoSpeed > 10f &&
                    ClosingFromBehind(egoPos, egoFwd, egoSpeed, rivalPos, rivalVel))
                {
                    // Only defend if we are roughly on pace (not in attack).
                    if (Mode != TacticalMode.OvertakeLeft && Mode != TacticalMode.OvertakeRight &&
                        Mode != TacticalMode.Commit)
                    {
                        want = TacticalMode.Defend;
                        reason = "rival-closing";
                        float h0 = route.HeadingAhead(0f);
                        float h1 = route.HeadingAhead(80f);
                        wantLat = RaceMath.Clamp(RaceMath.HeadingDiffDeg(h1, h0) / 30f, -0.6f, 0.6f);
                    }
                }
                else if (Mode == TacticalMode.Defend && (rivalIsAhead || rivalDist > 30f ||
                    !ClosingFromBehind(egoPos, egoFwd, egoSpeed, rivalPos, rivalVel)))
                {
                    want = rivalIsAhead && rivalDist < 60f ? TacticalMode.Follow : TacticalMode.Cruise;
                    reason = "threat-gone";
                }
                // Priority 3: attack chain when the rival is ahead and catchable.
                else if (rivalIsAhead && rivalDist < 70f)
                {
                    float eagerness = profile.OvertakeEagerness;
                    float attackRange = 18f + 30f * eagerness;
                    if (rivalDist < 30f && egoSpeed > 8f)
                    {
                        // Choose the freer side: corridor space + predicted traffic.
                        int side = ChooseOvertakeSide(corridor, perception, rLat, egoSpeed, profile);
                        if (Mode != TacticalMode.OvertakeLeft && Mode != TacticalMode.OvertakeRight &&
                            Mode != TacticalMode.Commit && nowMs > abortCooldownUntil)
                        {
                            want = side >= 0 ? TacticalMode.OvertakeLeft : TacticalMode.OvertakeRight;
                            reason = "attack";
                            wantSide = side >= 0 ? 1 : -1;
                            wantLat = side >= 0 ? 0.65f : -0.65f;
                        }
                        else if (Mode == TacticalMode.OvertakeLeft || Mode == TacticalMode.OvertakeRight)
                        {
                            want = Mode;
                            wantSide = Mode == TacticalMode.OvertakeLeft ? 1 : -1;
                            wantLat = wantSide > 0 ? 0.65f : -0.65f;
                            reason = Reason;
                            // Commit when the chosen side is predicted clear with
                            // TTC margin for the duration of the pass.
                            if (SideClear(perception, wantSide, egoSpeed, profile) && rivalDist < attackRange)
                            {
                                want = TacticalMode.Commit;
                                reason = "gap-open";
                                commitSince = nowMs;
                            }
                            // Abort when the gap collapses or a corner looms.
                            else if (SideBlocked(perception, wantSide, egoSpeed, profile) ||
                                (curveLimit < cruise * 0.55f))
                            {
                                want = TacticalMode.Abort;
                                reason = SideBlocked(perception, wantSide, egoSpeed, profile) ? "gap-closed" : "corner";
                                abortCooldownUntil = nowMs + 2500;
                            }
                        }
                        else if (Mode == TacticalMode.Commit)
                        {
                            want = Mode;
                            wantSide = OvertakeSide != 0 ? OvertakeSide : (rLat >= 0 ? -1 : 1);
                            wantLat = wantSide > 0 ? 0.65f : -0.65f;
                            reason = Reason;
                            if (SideBlocked(perception, wantSide, egoSpeed, profile))
                            {
                                want = TacticalMode.Abort;
                                reason = "commit-gap-closed";
                                abortCooldownUntil = nowMs + 2500;
                            }
                            else if (!rivalIsAhead || rivalDist > attackRange + 25f || nowMs - commitSince > 8000)
                            {
                                want = rivalIsAhead ? TacticalMode.Follow : TacticalMode.Cruise;
                                reason = "pass-complete";
                                wantSide = 0;
                                wantLat = 0f;
                            }
                        }
                        else if (Mode == TacticalMode.Abort)
                        {
                            want = Mode;
                            wantLat = 0f;
                            reason = Reason;
                            if (nowMs - SinceMs > 1500)
                            {
                                want = TacticalMode.Follow;
                                reason = "tucked-in";
                            }
                        }
                        else if (Mode == TacticalMode.AttackSetup)
                        {
                            want = side >= 0 ? TacticalMode.OvertakeLeft : TacticalMode.OvertakeRight;
                            reason = "side-chosen";
                            wantSide = side >= 0 ? 1 : -1;
                            wantLat = wantSide > 0 ? 0.65f : -0.65f;
                        }
                    }
                    else
                    {
                        // Too far to pass yet: follow / set up.
                        if (rivalDist < 45f)
                        {
                            want = TacticalMode.Follow;
                            reason = "closing";
                        }
                        else
                        {
                            want = TacticalMode.AttackSetup;
                            reason = "in-range";
                            wantLat = rLat > 0 ? 0.3f : -0.3f;
                        }
                    }
                }
                // Pass finished (rival behind us now): any attack mode resolves to
                // Cruise, or Defend if they are immediately counter-attacking
                // (handled above on the next tick). No distance threshold — a
                // completed pass with the rival 10 m behind is still complete.
                else if (!rivalIsAhead &&
                    (Mode == TacticalMode.Follow || Mode == TacticalMode.AttackSetup ||
                     Mode == TacticalMode.Abort || Mode == TacticalMode.OvertakeLeft ||
                     Mode == TacticalMode.OvertakeRight || Mode == TacticalMode.Commit))
                {
                    want = TacticalMode.Cruise;
                    reason = "pass-complete";
                    wantSide = 0;
                }
                else if (Mode == TacticalMode.Follow && (!rivalIsAhead || rivalDist > 70f))
                {
                    want = TacticalMode.Cruise;
                    reason = "gap-too-big";
                }
            }

            if (want != Mode)
            {
                PrevMode = Mode;
                Mode = want;
                SinceMs = nowMs;
                Reason = reason;
                ChangedThisTick = true;
                if (want == TacticalMode.Commit) commitSince = nowMs;
            }
            else if (Reason != reason && reason != null)
            {
                Reason = reason;
            }
            DesiredLateral = RaceMath.Clamp(wantLat, -1f, 1f);
            // Preserve chosen side through commit/abort so telemetry shows intent.
            if (wantSide != 0) OvertakeSide = wantSide;
            if (want == TacticalMode.Cruise || want == TacticalMode.Follow) OvertakeSide = 0;
        }

        private static bool ClosingFromBehind(Vector3 egoPos, Vector3 egoFwd, float egoSpeed,
            Vector3 rivalPos, Vector3 rivalVel)
        {
            var toR = new Vector3(rivalPos.X - egoPos.X, rivalPos.Y - egoPos.Y, 0f);
            if (RaceMath.FlatDot(toR, egoFwd) > -1f) return false; // not behind
            var rel = new Vector3(rivalVel.X, rivalVel.Y, 0f);
            // Rival velocity component toward ego along ego forward.
            float closing = RaceMath.FlatDot(rel, egoFwd) - egoSpeed * 0.9f;
            // Fallback: no velocity (parked pool entry) -> use proximity only.
            if (RaceMath.FlatLength(rel) < 1f)
                return RaceMath.FlatLength(toR) < 14f;
            return closing > 1.5f;
        }

        private static int ChooseOvertakeSide(RoadCorridor corridor, Perception perception,
            float rivalLat, float egoSpeed, DriverProfile profile)
        {
            // Prefer the side opposite the rival, penalised by predicted traffic.
            float leftClear = SideClearance(perception, 1, egoSpeed);
            float rightClear = SideClearance(perception, -1, egoSpeed);
            float need = profile.ClearanceNeed(8f);
            float leftScore = leftClear + (rivalLat < 0 ? 4f : 0f);
            float rightScore = rightClear + (rivalLat > 0 ? 4f : 0f);
            // Narrow road: stay put unless a side is clearly free.
            // Uses the sampled profile ahead, not just the current slice.
            float narrowHalf = corridor.MinHalfWidthAhead(60f);
            if (narrowHalf < 4.5f)
            {
                if (Math.Max(leftClear, rightClear) < need) return rivalLat < 0 ? 1 : -1;
            }
            return leftScore >= rightScore ? 1 : -1;
        }

        private static float SideClearance(Perception perception, int side, float egoSpeed)
        {
            float worst = 999f;
            foreach (var a in perception.Actors)
            {
                // Sidewalk clutter can never intersect an on-road pass.
                if (a.OffRoadway && (a.Kind == ActorKind.Ped || a.Kind == ActorKind.Obstacle) && a.Dist > 8f)
                    continue;
                // Route frame first (road-following); ego cone only as fallback.
                float lat, dist, ttc, closing;
                bool ahead;
                if (a.RouteValid)
                {
                    lat = a.RouteLateral;
                    dist = a.RouteDist;
                    ttc = Math.Min(a.Ttc, a.RouteTtc);
                    closing = Math.Max(a.ClosingSpeed, a.ClosingAlong);
                    ahead = dist > -6f && dist < 170f;
                }
                else
                {
                    if (!a.IsAhead) continue;
                    lat = a.Lateral;
                    dist = a.Dist;
                    ttc = a.Ttc;
                    closing = a.ClosingSpeed;
                    ahead = true;
                }
                if (!ahead) continue;
                bool onSide = side > 0 ? lat > -1f : lat < 1f;
                if (!onSide) continue;
                if (dist < worst) worst = dist;
                if (ttc < 3f && closing > 2f)
                    worst = Math.Min(worst, dist * 0.5f);
            }
            return worst;
        }

        private static bool SideClear(Perception perception, int side, float egoSpeed, DriverProfile profile)
        {
            float need = profile.ClearanceNeed(14f + egoSpeed * 0.35f);
            foreach (var a in perception.Actors)
            {
                if (a.OffRoadway && (a.Kind == ActorKind.Ped || a.Kind == ActorKind.Obstacle) && a.Dist > 8f)
                    continue;
                float lat, dist, ttc;
                bool ahead;
                if (a.RouteValid)
                {
                    lat = a.RouteLateral;
                    dist = a.RouteDist;
                    ttc = Math.Min(a.Ttc, a.RouteTtc);
                    ahead = dist > -6f && dist < 170f;
                }
                else
                {
                    if (!a.IsAhead) continue;
                    lat = a.Lateral;
                    dist = a.Dist;
                    ttc = a.Ttc;
                    ahead = true;
                }
                if (!ahead) continue;
                bool onSide = side > 0 ? lat > -2f : lat < 2f;
                if (!onSide) continue;
                if (dist < need && ttc < 4f) return false;
                if (a.Kind == ActorKind.Ped && dist < need * 0.7f) return false;
            }
            return true;
        }

        private static bool SideBlocked(Perception perception, int side, float egoSpeed, DriverProfile profile)
        {
            float need = profile.ClearanceNeed(10f + egoSpeed * 0.25f);
            foreach (var a in perception.Actors)
            {
                if (a.OffRoadway && (a.Kind == ActorKind.Ped || a.Kind == ActorKind.Obstacle) && a.Dist > 8f)
                    continue;
                float lat, dist, ttc;
                bool ahead;
                if (a.RouteValid)
                {
                    lat = a.RouteLateral;
                    dist = a.RouteDist;
                    ttc = Math.Min(a.Ttc, a.RouteTtc);
                    ahead = dist > -6f && dist < 170f;
                }
                else
                {
                    if (!a.IsAhead) continue;
                    lat = a.Lateral;
                    dist = a.Dist;
                    ttc = a.Ttc;
                    ahead = true;
                }
                if (!ahead) continue;
                bool onSide = side > 0 ? lat > -2.5f : lat < 2.5f;
                if (!onSide) continue;
                if (dist < need && (ttc < 2.5f || dist < 12f)) return true;
            }
            return false;
        }
    }
}
