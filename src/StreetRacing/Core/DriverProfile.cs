namespace StreetRacing
{
    /// Driver skill / personality as planner parameters — never as random
    /// steering noise. Risk tolerance widens accepted gaps and raises speed
    /// in corners; caution widens safety margins and lookahead.
    internal sealed class DriverProfile
    {
        public string Name = "Balanced";
        /// How far ahead (in seconds of travel) the planner looks for aim + speed.
        public float LookaheadTimeS = 2.5f;
        public float LookaheadMinM = 35f;
        public float LookaheadMaxM = 150f;
        /// Control / perception cadence.
        public int ReactionIntervalMs = 100;
        /// Extra stopped gap kept to obstacles (m).
        public float SafetyMarginM = 7f;
        /// Extra time gap kept to moving traffic (s).
        public float SafetyTimeS = 1.1f;
        /// Multiplier on measured grip (0.7 = leaves margin, 1.0 = uses all).
        public float GripFactor = 0.85f;
        /// 0 = cautious, 1 = psycho. Scales clearance acceptance + overtake eagerness.
        public float RiskTolerance = 0.6f;
        /// Extra slowdown before corners (1.0 = neutral, >1 = earlier braking).
        public float CornerCaution = 1.0f;
        public float OvertakeEagerness = 0.7f;

        public float LookaheadForSpeed(float speed)
        {
            float d = speed * LookaheadTimeS;
            if (d < LookaheadMinM) d = LookaheadMinM;
            if (d > LookaheadMaxM) d = LookaheadMaxM;
            return d;
        }

        public float ClearanceNeed(float baseNeed)
        {
            // Risk-tolerant drivers accept tighter gaps.
            return baseNeed * (1.35f - 0.7f * RiskTolerance);
        }

        public static DriverProfile FromName(string name, float riskOverride = -1f)
        {
            var p = new DriverProfile();
            string n = (name ?? "balanced").Trim().ToLowerInvariant();
            switch (n)
            {
                case "cautious":
                case "calm":
                    p.Name = "Cautious";
                    p.LookaheadTimeS = 3.0f;
                    p.SafetyMarginM = 10f;
                    p.SafetyTimeS = 1.5f;
                    p.GripFactor = 0.72f;
                    p.RiskTolerance = 0.25f;
                    p.CornerCaution = 1.15f;
                    p.OvertakeEagerness = 0.35f;
                    p.ReactionIntervalMs = 120;
                    break;
                case "aggressive":
                case "psycho":
                    p.Name = "Aggressive";
                    p.LookaheadTimeS = 2.2f;
                    p.SafetyMarginM = 5f;
                    p.SafetyTimeS = 0.8f;
                    p.GripFactor = 0.95f;
                    p.RiskTolerance = 0.85f;
                    p.CornerCaution = 0.92f;
                    p.OvertakeEagerness = 0.9f;
                    p.ReactionIntervalMs = 90;
                    break;
                default:
                    p.Name = "Balanced";
                    break;
            }
            if (riskOverride >= 0f && riskOverride <= 1f)
            {
                p.RiskTolerance = riskOverride;
            }
            return p;
        }
    }
}
