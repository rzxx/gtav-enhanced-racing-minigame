using System;
using GTA;

namespace StreetRacing
{
    internal enum SampleKind
    {
        Normal,
        Braking,
        Impact,
        Teleport,
    }

    /// Distinguishes real braking from crashes and position teleports.
    /// The old detector flagged any `dec < -7` as HARD_BRAKE, so `dec < -20`
    /// impact/teleport samples (physically impossible for tyres) polluted
    /// braking analysis. Anything beyond plausible tyre decel is classified
    /// as impact or teleport, never as braking.
    internal static class ImpactClassifier
    {
        /// Plausible DriveV limits (generous so modded cars don't false-trip).
        public const float MaxPlausibleBrakeDecel = 12f;   // m/s^2
        public const float HardBrakeThreshold = 5.5f;

        public static SampleKind Classify(
            float accel, float dtS, float displacementM, float expectedM,
            float healthDrop, bool hasCollided, float speed)
        {
            // Teleport: moved far more (or far less with a jump) than physics
            // allows in one sample. Threshold scales with dt.
            float jump = Math.Abs(displacementM - expectedM);
            float teleportTol = Math.Max(12f, speed * dtS * 2.5f + 10f);
            if (jump > teleportTol && dtS < 0.5f)
                return SampleKind.Teleport;
            if (displacementM > Math.Max(30f, speed * dtS + 25f) && dtS < 0.5f)
                return SampleKind.Teleport;

            // Impact: impossible decel, or strong decel corroborated by damage
            // / collision flags. Health is int-based; any multi-point drop in
            // one 100 ms sample is a hit, not wear.
            if (accel < -MaxPlausibleBrakeDecel)
                return SampleKind.Impact;
            if (healthDrop >= 4f && accel < -3f)
                return SampleKind.Impact;
            if (hasCollided && accel < -8f)
                return SampleKind.Impact;

            if (accel < -HardBrakeThreshold)
                return SampleKind.Braking;
            return SampleKind.Normal;
        }

        public static bool TryReadCollision(Vehicle v, out bool collided)
        {
            collided = false;
            try
            {
                if (v == null || !v.Exists()) return false;
                collided = v.HasCollided;
                return true;
            }
            catch { return false; }
        }
    }
}
