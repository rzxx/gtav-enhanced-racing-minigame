using System;
using GTA;
using GTA.Math;

namespace StreetRacing
{
    /// Per-vehicle braking / cornering capability, seeded from handling data
    /// and refined from observation. Never hardcoded by vehicle class: two
    /// examples of the same model with different mods / damage converge to
    /// different estimates, which is exactly what DriveV needs.
    ///
    /// SPIN REJECTION: yaw/spin samples must never train lateral grip. A
    /// spinning car shows huge heading-derived yawRate * speed as apparent
    /// "lateral accel", but the tyres are sliding, not gripping. Learning is
    /// gated on stability (small slip angle between nose and velocity,
    /// plausible yaw rate, no airborne/teleport) and tracked with a
    /// confidence 0..1 that rises on stable samples and collapses on
    /// unstable ones. Only stable, physically valid samples adapt the caps.
    internal sealed class VehicleCapability
    {
        public float ABrakeMax = 7f;    // usable longitudinal decel (m/s^2)
        public float ALatMax = 7.5f;    // usable lateral accel (m/s^2)
        public float TopSpeedEst = 50f;
        public bool SeededFromHandling;

        // Learning state.
        public float Confidence = 0.5f; // 0..1, starts neutral
        public int StableSamples;
        public int UnstableSamples;
        public float LastSlipDeg;
        public float LastYawRate;
        public bool LastStable = true;

        private float obsBrakePeak;
        private float obsLatPeak;
        private float obsTop;

        public void Seed(Vehicle v)
        {
            ABrakeMax = 7f;
            ALatMax = 7.5f;
            TopSpeedEst = 50f;
            SeededFromHandling = false;
            Confidence = 0.5f;
            StableSamples = 0;
            UnstableSamples = 0;
            LastSlipDeg = 0f;
            LastYawRate = 0f;
            LastStable = true;
            obsBrakePeak = 0f;
            obsLatPeak = 0f;
            obsTop = 0f;
            try
            {
                if (v == null || !v.Exists()) return;
                // Cheap derived caps first (always available).
                try
                {
                    float mb = v.MaxBraking;
                    if (mb > 0.05f && mb < 5f) ABrakeMax = RaceMath.Clamp(mb * 7f, 4f, 11f);
                }
                catch { }
                try
                {
                    float mt = v.MaxTraction;
                    if (mt > 0.2f && mt < 6f) ALatMax = RaceMath.Clamp(mt * 3.4f, 4f, 11f);
                }
                catch { }
                // Richer handling fields when exposed.
                try
                {
                    var h = v.HandlingData;
                    if (h != null)
                    {
                        if (h.BrakeForce > 0.05f)
                            ABrakeMax = RaceMath.Clamp(h.BrakeForce * 7f, 4f, 11.5f);
                        if (h.TractionCurveMax > 0.2f)
                            ALatMax = RaceMath.Clamp(h.TractionCurveMax * 4.4f, 4f, 11.5f);
                        if (h.InitialDriveMaxFlatVelocity > 10f)
                            TopSpeedEst = h.InitialDriveMaxFlatVelocity * 1.05f;
                        SeededFromHandling = true;
                    }
                }
                catch { }
                // NOTE: Entity.MaxSpeed is set-only in SHVDN (a cap, not a readout),
                // so top speed beyond handling comes purely from observation.
            }
            catch { }
        }

        /// Called every tick with measured motion. Impact/teleport samples
        /// must be excluded by the caller (see ImpactClassifier). Spin/yaw
        /// samples are excluded HERE via slip + yaw gates, even if the caller
        /// thought the sample was Normal/Braking.
        public void Observe(float accelLong, float latAccel, float speed, float dtS,
            float yawRateRadS, float slipDeg)
        {
            LastYawRate = yawRateRadS;
            LastSlipDeg = slipDeg;
            if (dtS <= 0f || dtS > 0.5f) { LastStable = false; return; }
            if (speed < 4f) { LastStable = false; return; }

            bool stable = IsStable(speed, yawRateRadS, slipDeg, latAccel, accelLong);
            LastStable = stable;
            if (stable)
            {
                StableSamples++;
                Confidence = Math.Min(1f, Confidence + 0.015f);
            }
            else
            {
                UnstableSamples++;
                // Unstable motion destroys trust fast (one spin != grip).
                Confidence = Math.Max(0f, Confidence - 0.12f);
                return; // never learn from slides / spins / jumps
            }

            // Require a minimum of stable evidence before moving the caps at
            // all (avoids single-sample spikes even when marked stable).
            if (Confidence < 0.15f) return;

            // Braking: sustained strong decel raises the estimate slowly;
            // single spikes never do (they are impacts, filtered upstream).
            if (accelLong < -2f && accelLong > -ImpactClassifier.MaxPlausibleBrakeDecel)
            {
                // Braking while sliding sideways is not tyre braking.
                if (Math.Abs(slipDeg) < 12f && Math.Abs(yawRateRadS) < 0.5f)
                {
                    float a = -accelLong;
                    if (a > obsBrakePeak) obsBrakePeak = Math.Min(a, 11.5f);
                }
            }
            float la = Math.Abs(latAccel);
            if (la > 1f && la < 12f && speed > 8f)
            {
                // Grip learning needs nose≈velocity and calm yaw: a drifting
                // car at 40 deg slip is not demonstrating 10 m/s^2 of grip.
                if (Math.Abs(slipDeg) < 12f && Math.Abs(yawRateRadS) < 0.55f)
                {
                    if (la > obsLatPeak) obsLatPeak = Math.Min(la, 11.5f);
                }
            }
            if (speed > obsTop && Math.Abs(slipDeg) < 15f && Math.Abs(yawRateRadS) < 0.6f) obsTop = speed;

            // Blend observations toward the usable caps with slow adaptation:
            // trust handling as a prior, let measurement pull it. Rate scales
            // with confidence so early/uncertain running adapts slower.
            float rate = 0.008f + 0.022f * Confidence;
            if (obsBrakePeak > 3f)
                ABrakeMax += (Math.Min(obsBrakePeak * 1.05f, 11.5f) - ABrakeMax) * rate;
            if (obsLatPeak > 3f)
                ALatMax += (Math.Min(obsLatPeak * 1.05f, 11.5f) - ALatMax) * rate;
            if (obsTop > 10f && obsTop > TopSpeedEst)
                TopSpeedEst = obsTop;
        }

        /// Back-compat overload (assumes stable — prefer the full version).
        public void Observe(float accelLong, float latAccel, float speed, float dtS)
        {
            Observe(accelLong, latAccel, speed, dtS, 0f, 0f);
        }

        private static bool IsStable(float speed, float yawRate, float slipDeg, float latAccel, float accelLong)
        {
            if (Math.Abs(yawRate) > 0.9f) return false;          // spinning (>~50 deg/s)
            if (Math.Abs(slipDeg) > 20f) return false;           // sliding sideways
            if (speed > 8f && Math.Abs(yawRate) > 0.6f && Math.Abs(slipDeg) > 12f) return false; // drift
            if (Math.Abs(latAccel) > 12.5f) return false;        // impossible for tyres
            if (accelLong < -ImpactClassifier.MaxPlausibleBrakeDecel) return false;
            if (float.IsNaN(yawRate) || float.IsNaN(slipDeg) || float.IsNaN(latAccel)) return false;
            return true;
        }

        public float UsableBrake(float gripFactor)
        {
            return RaceMath.Clamp(ABrakeMax * gripFactor, 3.5f, 11.5f);
        }

        public float UsableLat(float gripFactor)
        {
            return RaceMath.Clamp(ALatMax * gripFactor, 3.5f, 11.5f);
        }

        public float StoppingDistance(float speed, float gripFactor, float reactionS)
        {
            float a = UsableBrake(gripFactor);
            if (a < 1f) a = 5f;
            return speed * reactionS + (speed * speed) / (2f * a);
        }
    }
}
