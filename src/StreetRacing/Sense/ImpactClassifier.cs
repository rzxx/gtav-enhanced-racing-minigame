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
    /// Collapsed rule (Phase 3): deceleration ALONE never classifies a crash.
    /// Almost all legacy IMPACT events had zero damage: hard braking /
    /// physics spikes fed Crashed -> Recovery and poisoned normal driving.
    /// Impact now requires corroborated evidence: actual collision flag or
    /// damage drop, plus strong decel. Anything else is Braking/Normal.
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

            // Impact: strong decel CORROBORATED by damage / collision flags.
            // Decel alone (even < -12) is braking or a physics spike, never
            // an impact: legacy traces showed almost all IMPACT events with
            // zero damage, feeding false Crashed -> Recovery.
            if (healthDrop >= 4f && accel < -3f)
                return SampleKind.Impact;
            if (hasCollided && accel < -8f)
                return SampleKind.Impact;
            if (accel < -MaxPlausibleBrakeDecel && (hasCollided || healthDrop >= 4f))
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
