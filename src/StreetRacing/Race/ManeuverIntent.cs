using System;

namespace StreetRacing.Race
{
    /// Persistent maneuver intent (Phase 4).
    ///
    /// The legacy planner reconsidered seven unrelated whole-road lateral
    /// choices every ~100 ms with almost no maneuver commitment, so the
    /// lateral target jumped across the road width (-3.8, +10.7, ...) and
    /// the speed planner then braked for the curvature it had just created.
    ///
    /// Collapsed rule: the trajectory planner generates paths INSIDE the
    /// chosen intent instead of independently reconsidering the entire road
    /// width every plan tick. An intent persists for a meaningful period and
    /// only changes when completed, clearly unsafe, impossible, or tactical
    /// conditions materially change (with dwell hysteresis).
    internal enum ManeuverIntent
    {
        KEEP_LINE,   // default: one pose-feasible center trajectory
        FOLLOW,      // single slower/stopped civilian ahead, passing unsafe
        PASS_LEFT,   // committed pass on the left of one blocker
        PASS_RIGHT,  // committed pass on the right of one blocker
        RECOVER,     // explicit recovery primitive owns control
    }

    internal sealed class ManeuverIntentState
    {
        public ManeuverIntent Current = ManeuverIntent.KEEP_LINE;
        public int SinceMs;
        public string Reason = "init";
        public bool ChangedThisTick;

        // Minimum dwell before a non-forced intent switch (maneuver hysteresis).
        public int DwellMs = 1500;
        private bool initialized;

        public void Reset(int nowMs)
        {
            Current = ManeuverIntent.KEEP_LINE;
            SinceMs = nowMs;
            Reason = "init";
            ChangedThisTick = false;
            initialized = true;
        }

        /// Request an intent. Returns the (possibly unchanged) current intent.
        /// forced=true bypasses dwell (unsafe/impossible/completed).
        public ManeuverIntent Request(ManeuverIntent want, string reason, int nowMs, bool forced)
        {
            ChangedThisTick = false;
            if (!initialized) { Current = want; SinceMs = nowMs; Reason = reason ?? ""; initialized = true; ChangedThisTick = true; return Current; }
            if (want == Current) { if (reason != null) Reason = reason; return Current; }
            bool dwellOk = (nowMs - SinceMs) >= DwellMs;
            if (!forced && !dwellOk) return Current; // hold: temporal continuity
            Current = want;
            SinceMs = nowMs;
            Reason = reason ?? "";
            ChangedThisTick = true;
            return Current;
        }

        public void Force(ManeuverIntent want, string reason, int nowMs)
        {
            Request(want, reason, nowMs, true);
        }

        public float HeldS(int nowMs) => (nowMs - SinceMs) / 1000f;
    }
}
