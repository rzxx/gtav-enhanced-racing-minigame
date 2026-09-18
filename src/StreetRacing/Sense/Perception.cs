using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;

namespace StreetRacing
{
    internal enum ActorKind
    {
        TrafficVehicle,
        Rival,
        Ped,
        Obstacle,
    }

    /// One perceived actor with motion-compensated threat metrics.
    /// TTC uses closing speed along the line of sight, not raw distance —
    /// a parked car 40 m ahead at 40 m/s (TTC ~1 s) outranks a car 20 m
    /// behind moving away.
    internal struct TrackedActor
    {
        public bool Valid;
        public ActorKind Kind;
        public Vector3 Position;
        public Vector3 Velocity;
        public float Dist;          // flat distance from ego
        public float Longitudinal;  // + ahead along ego forward
        public float Lateral;       // + left
        public float ClosingSpeed;  // + = approaching (m/s)
        public float Ttc;           // s, 999 = separating / static-safe
        public float Speed;         // actor ground speed
        public bool IsAhead;
    }

    /// Predictive perception well beyond stopping distance.
    /// Replaces the old 40 m / ~20 deg frontal sample at 2 Hz, which saw
    /// DriveV traffic (~30–42 m/s) only ~1 s before impact.
    ///
    /// - Range adapts to ego stopping distance + margin (up to 170 m).
    /// - Full surround (not a narrow cone); cones are applied at scoring time.
    /// - Relative velocity + TTC for every actor; predictions assume constant
    ///   velocity over the planner horizon (2–3 s).
    internal sealed class Perception
    {
        public readonly List<TrackedActor> Actors = new List<TrackedActor>();
        public float RangeM = 120f;
        public int LastScanMs;
        public int ClosestTtcIndex = -1;

        public int Count => Actors.Count;
        public int AheadCount;
        public float NearestAheadDist = 999f;
        public float NearestAheadTtc = 999f;
        public float NearestAheadClosing;

        public void Update(Vehicle ego, Vehicle rivalVehicle, Ped rivalPed, float egoSpeed,
            float brakeCap, float reactionTimeS, int nowMs, int minIntervalMs = 100)
        {
            if (nowMs - LastScanMs < minIntervalMs) return;
            LastScanMs = nowMs;
            Actors.Clear();
            AheadCount = 0;
            NearestAheadDist = 999f;
            NearestAheadTtc = 999f;
            NearestAheadClosing = 0f;
            ClosestTtcIndex = -1;

            Vector3 egoPos;
            Vector3 egoVel;
            Vector3 fwd;
            try
            {
                if (ego == null || !ego.Exists()) return;
                egoPos = ego.Position;
                egoVel = ego.Velocity;
                fwd = RaceMath.FlatNormalize(new Vector3(ego.ForwardVector.X, ego.ForwardVector.Y, 0f));
            }
            catch { return; }

            // Dynamic range: stopping distance + margin, so fast DriveV cars
            // see 120–170 m ahead instead of a fixed 40 m.
            float aB = brakeCap > 1f ? brakeCap : 6f;
            float stop = egoSpeed * reactionTimeS + (egoSpeed * egoSpeed) / (2f * aB);
            RangeM = RaceMath.Clamp(stop + 60f, 80f, 170f);

            var left = new Vector3(-fwd.Y, fwd.X, 0f);

            try
            {
                foreach (var v in World.GetNearbyVehicles(egoPos, RangeM))
                {
                    if (v == null || !v.Exists() || v == ego) continue;
                    AddVehicle(v.Position, SafeVelocity(v), ActorKind.TrafficVehicle,
                        egoPos, egoVel, fwd, left);
                }
            }
            catch { }

            // Rival (player) is always tracked explicitly even if outside the
            // vehicle pool for a frame (pool culling at range).
            try
            {
                Vehicle rv = rivalVehicle;
                if (rv != null && rv.Exists() && rv != ego)
                {
                    bool already = false;
                    foreach (var a in Actors)
                    {
                        if (RaceMath.FlatDistance(a.Position, rv.Position) < 2f) { already = true; break; }
                    }
                    if (!already && RaceMath.FlatDistance(egoPos, rv.Position) < RangeM + 30f)
                        AddVehicle(rv.Position, SafeVelocity(rv), ActorKind.Rival, egoPos, egoVel, fwd, left);
                    else if (already)
                        MarkRival(rv.Position);
                }
                else if (rivalPed != null && rivalPed.Exists())
                {
                    var rp = rivalPed.Position;
                    if (RaceMath.FlatDistance(egoPos, rp) < RangeM)
                    {
                        Vector3 rvv = new Vector3();
                        try { rvv = rivalPed.Velocity; } catch { }
                        AddVehicle(rp, rvv, ActorKind.Rival, egoPos, egoVel, fwd, left);
                    }
                }
            }
            catch { }

            // Peds: shorter range (perf + relevance), still TTC-aware.
            try
            {
                float pedRange = Math.Min(RangeM, 70f);
                foreach (var p in World.GetNearbyPeds(egoPos, pedRange))
                {
                    if (p == null || !p.Exists()) continue;
                    Vector3 pv = new Vector3();
                    try { pv = p.Velocity; } catch { }
                    AddVehicle(p.Position, pv, ActorKind.Ped, egoPos, egoVel, fwd, left);
                }
            }
            catch { }

            // Static obstacles (props, street furniture, barriers): short range,
            // capped count. Modelled as zero-velocity actors so TTC degrades to
            // dist/egoSpeed and the trajectory scorer treats them as blocks.
            try
            {
                float propRange = Math.Min(RangeM, 60f);
                int added = 0;
                foreach (var pr in World.GetNearbyProps(egoPos, propRange))
                {
                    if (pr == null || !pr.Exists()) continue;
                    if (added >= 10) break;
                    Vector3 pp;
                    try { pp = pr.Position; } catch { continue; }
                    float pd = RaceMath.FlatDistance(egoPos, pp);
                    if (pd < 4f) continue; // ignore what we're already touching
                    AddVehicle(pp, new Vector3(), ActorKind.Obstacle, egoPos, egoVel, fwd, left);
                    added++;
                }
            }
            catch { }

            // Summaries for planner + telemetry.
            float bestTtc = 999f;
            for (int i = 0; i < Actors.Count; i++)
            {
                var a = Actors[i];
                if (a.IsAhead)
                {
                    AheadCount++;
                    if (a.Dist < NearestAheadDist) NearestAheadDist = a.Dist;
                    if (a.ClosingSpeed > 0.5f && Math.Abs(a.Lateral) < 6f)
                    {
                        if (a.Dist < 60f || a.Ttc < NearestAheadTtc)
                        {
                            NearestAheadTtc = a.Ttc;
                            NearestAheadClosing = a.ClosingSpeed;
                        }
                    }
                }
                if (a.Ttc < bestTtc && a.ClosingSpeed > 0.5f)
                {
                    bestTtc = a.Ttc;
                    ClosestTtcIndex = i;
                }
            }

            // Keep the list bounded + sorted by threat for telemetry stability.
            Actors.Sort((x, y) => x.Ttc.CompareTo(y.Ttc));
            if (Actors.Count > 24)
                Actors.RemoveRange(24, Actors.Count - 24);
        }

        public bool TryGetClosestThreat(out TrackedActor actor)
        {
            actor = new TrackedActor();
            if (ClosestTtcIndex >= 0 && ClosestTtcIndex < Actors.Count)
            {
                actor = Actors[ClosestTtcIndex];
                return true;
            }
            // Fall back to nearest ahead.
            float bd = 999f;
            bool found = false;
            foreach (var a in Actors)
            {
                if (a.IsAhead && a.Dist < bd) { bd = a.Dist; actor = a; found = true; }
            }
            return found;
        }

        /// Constant-velocity prediction used by trajectory scoring.
        public Vector3 Predict(TrackedActor a, float dt)
        {
            return new Vector3(
                a.Position.X + a.Velocity.X * dt,
                a.Position.Y + a.Velocity.Y * dt,
                a.Position.Z);
        }

        private void AddVehicle(Vector3 pos, Vector3 vel, ActorKind kind,
            Vector3 egoPos, Vector3 egoVel, Vector3 fwd, Vector3 left)
        {
            var to = new Vector3(pos.X - egoPos.X, pos.Y - egoPos.Y, 0f);
            float dist = RaceMath.FlatLength(to);
            float lon = RaceMath.FlatDot(to, fwd);
            float lat = RaceMath.FlatCross(fwd, to);
            var rel = new Vector3(vel.X - egoVel.X, vel.Y - egoVel.Y, 0f);
            float closing = 0f;
            float ttc = 999f;
            if (dist > 0.5f)
            {
                var dir = new Vector3(to.X / dist, to.Y / dist, 0f);
                closing = -(rel.X * dir.X + rel.Y * dir.Y); // + = approaching
                if (closing > 0.5f) ttc = dist / closing;
            }
            float spd = RaceMath.FlatLength(vel);
            Actors.Add(new TrackedActor
            {
                Valid = true,
                Kind = kind,
                Position = pos,
                Velocity = new Vector3(vel.X, vel.Y, 0f),
                Dist = dist,
                Longitudinal = lon,
                Lateral = lat,
                ClosingSpeed = closing,
                Ttc = ttc,
                Speed = spd,
                IsAhead = lon > 0f && Math.Abs(RaceMath.SignedAngleDeg(fwd, to)) < 65f,
            });
        }

        private void MarkRival(Vector3 rivalPos)
        {
            for (int i = 0; i < Actors.Count; i++)
            {
                if (RaceMath.FlatDistance(Actors[i].Position, rivalPos) < 2f)
                {
                    var a = Actors[i];
                    a.Kind = ActorKind.Rival;
                    Actors[i] = a;
                    return;
                }
            }
        }

        private static Vector3 SafeVelocity(Entity e)
        {
            try { return e.Velocity; }
            catch { return new Vector3(); }
        }
    }
}
