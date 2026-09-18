using System;
using GTA;
using GTA.Math;

namespace StreetRacing
{
    /// Per-vehicle braking / cornering capability, seeded from handling data
    /// and refined from observation. Never hardcoded by vehicle class: two
    /// examples of the same model with different mods / damage converge to
    /// different estimates, which is exactly what DriveV needs.
    internal sealed class VehicleCapability
    {
        public float ABrakeMax = 7f;    // usable longitudinal decel (m/s^2)
        public float ALatMax = 7.5f;    // usable lateral accel (m/s^2)
        public float TopSpeedEst = 50f;
        public bool SeededFromHandling;

        private float obsBrakePeak;
        private float obsLatPeak;
        private float obsTop;

        public void Seed(Vehicle v)
        {
            ABrakeMax = 7f;
            ALatMax = 7.5f;
            TopSpeedEst = 50f;
            SeededFromHandling = false;
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
        /// must be excluded by the caller (see ImpactClassifier).
        public void Observe(float accelLong, float latAccel, float speed, float dtS)
        {
            if (dtS <= 0f || dtS > 0.5f) return;
            if (speed < 4f) return;
            // Braking: sustained strong decel raises the estimate slowly;
            // single spikes never do (they are impacts, filtered upstream).
            if (accelLong < -2f && accelLong > -ImpactClassifier.MaxPlausibleBrakeDecel)
            {
                float a = -accelLong;
                if (a > obsBrakePeak) obsBrakePeak = Math.Min(a, 11.5f);
            }
            float la = Math.Abs(latAccel);
            if (la > 1f && la < 12f && speed > 8f)
            {
                if (la > obsLatPeak) obsLatPeak = Math.Min(la, 11.5f);
            }
            if (speed > obsTop) obsTop = speed;

            // Blend observations toward the usable caps with slow adaptation:
            // trust handling as a prior, let measurement pull it.
            if (obsBrakePeak > 3f)
                ABrakeMax += (Math.Min(obsBrakePeak * 1.05f, 11.5f) - ABrakeMax) * 0.02f;
            if (obsLatPeak > 3f)
                ALatMax += (Math.Min(obsLatPeak * 1.05f, 11.5f) - ALatMax) * 0.02f;
            if (obsTop > 10f && obsTop > TopSpeedEst)
                TopSpeedEst = obsTop;
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
