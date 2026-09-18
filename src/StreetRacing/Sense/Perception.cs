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
    ///
    /// Route-frame fields (RouteS / RouteLateral / SpeedAlong) express the
    /// same actor in candidate-path coordinates. All planning decisions
    /// (trajectory scoring, speed limits, tactics) must use the route frame;
    /// ego-heading Longitudinal/Lateral/IsAhead are kept for telemetry and
    /// as a fallback when the route is lost.
    internal struct TrackedActor
    {
        public bool Valid;
        public ActorKind Kind;
        public Vector3 Position;
        public Vector3 Velocity;
        public float Dist;          // flat distance from ego
        public float Longitudinal;  // + ahead along ego forward
        public float Lateral;       // + left
        public float ClosingSpeed;  // + = approaching (m/s, line-of-sight)
        public float Ttc;           // s, 999 = separating / static-safe
        public float Speed;         // actor ground speed
        public bool IsAhead;

        // Route frame (valid when RouteValid).
        public bool RouteValid;
        public float RouteS;        // absolute arclength of projection
        public float RouteLateral;  // + = left of route direction
        public float RouteDist;     // RouteS - egoS (m, + = ahead along route)
        public float SpeedAlong;    // actor velocity projected on route dir
        public float ClosingAlong;  // egoSpeedAlong - SpeedAlong (+ = catching)
        public float RouteTtc;      // s from along-route closing
    }

    /// Predictive perception well beyond stopping distance.
    /// Replaces the old 40 m / ~20 deg frontal sample at 2 Hz, which saw
    /// DriveV traffic (~30–42 m/s) only ~1 s before impact.
    ///
    /// - Range adapts to ego stopping distance + margin (up to 170 m).
    /// - Full surround (not a narrow cone); cones are applied at scoring time.
    /// - Relative velocity + TTC for every actor; predictions assume constant
    ///   velocity over the planner horizon (2–3 s).
    /// - Every actor is additionally projected into route coordinates so
    ///   collision reasoning follows the road, not just the current nose.
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
            Update(ego, rivalVehicle, rivalPed, egoSpeed, brakeCap, reactionTimeS, nowMs, minIntervalMs, null, null);
        }

        public void Update(Vehicle ego, Vehicle rivalVehicle, Ped rivalPed, float egoSpeed,
            float brakeCap, float reactionTimeS, int nowMs, int minIntervalMs,
            RaceRoute route, RoadCorridor corridor)
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
            float egoS = (route != null && route.Built) ? route.AlongS : 0f;
            // Ego speed along the route (discounts sliding / wrong-way).
            float egoAlong = egoSpeed;
            try
            {
                if (route != null && route.Built)
                {
                    float he = Math.Abs(route.HeadingErrorDeg) * (float)Math.PI / 180f;
                    egoAlong = egoSpeed * (float)Math.Cos(Math.Min(he, 1.2f));
                    if (egoAlong < 0f) egoAlong = 0f;
                }
            }
            catch { }

            try
            {
                foreach (var v in World.GetNearbyVehicles(egoPos, RangeM))
                {
                    if (v == null || !v.Exists() || v == ego) continue;
                    AddVehicle(v.Position, SafeVelocity(v), ActorKind.TrafficVehicle,
                        egoPos, egoVel, fwd, left, route, corridor, egoS, egoAlong);
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
                        AddVehicle(rv.Position, SafeVelocity(rv), ActorKind.Rival, egoPos, egoVel, fwd, left, route, corridor, egoS, egoAlong);
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
                        AddVehicle(rp, rvv, ActorKind.Rival, egoPos, egoVel, fwd, left, route, corridor, egoS, egoAlong);
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
                    AddVehicle(p.Position, pv, ActorKind.Ped, egoPos, egoVel, fwd, left, route, corridor, egoS, egoAlong);
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
                    AddVehicle(pp, new Vector3(), ActorKind.Obstacle, egoPos, egoVel, fwd, left, route, corridor, egoS, egoAlong);
                    added++;
                }
            }
            catch { }

            // Summaries for planner + telemetry (computed BEFORE sort; the
            // threat index is resolved AFTER sort so it stays valid).
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
            }

            // Keep the list bounded + sorted by threat for telemetry stability.
            Actors.Sort((x, y) => x.Ttc.CompareTo(y.Ttc));
            if (Actors.Count > 24)
                Actors.RemoveRange(24, Actors.Count - 24);

            // Resolve the closest-TTC index AFTER sorting (the old code set it
            // before Sort, so it pointed at a different actor afterwards).
            float bestTtc = 999f;
            ClosestTtcIndex = -1;
            for (int i = 0; i < Actors.Count; i++)
            {
                var a = Actors[i];
                if (a.ClosingSpeed > 0.5f && a.Ttc < bestTtc)
                {
                    bestTtc = a.Ttc;
                    ClosestTtcIndex = i;
                }
            }
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

        /// Nearest actor ahead ON THE ROUTE (not just in the nose cone).
        /// Used by the speed planner's stopping logic.
        public bool TryGetLeadOnRoute(out TrackedActor actor, float egoS, RoadCorridor corridor, float maxDistM)
        {
            actor = new TrackedActor();
            float bd = float.MaxValue;
            bool found = false;
            foreach (var a in Actors)
            {
                if (!a.RouteValid) continue;
                if (a.RouteDist < -2f || a.RouteDist > maxDistM) continue;
                float half = corridor != null ? corridor.HalfWidthAt(Math.Max(0f, a.RouteDist)) : 7f;
                float latTol = half + (a.Kind == ActorKind.Ped ? 1.5f : 2f);
                if (Math.Abs(a.RouteLateral) > latTol) continue;
                if (a.RouteDist < bd) { bd = a.RouteDist; actor = a; found = true; }
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
            Vector3 egoPos, Vector3 egoVel, Vector3 fwd, Vector3 left,
            RaceRoute route, RoadCorridor corridor, float egoS, float egoAlong)
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
            var a = new TrackedActor
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
                RouteValid = false,
                RouteS = 0f,
                RouteLateral = 999f,
                RouteDist = 999f,
                SpeedAlong = 0f,
                ClosingAlong = 0f,
                RouteTtc = 999f,
            };
            // Route-frame projection (road-following, not nose-following).
            try
            {
                if (route != null && route.Built)
                {
                    var pr = route.ProjectOntoRoute(pos);
                    a.RouteValid = true;
                    a.RouteS = pr.S;
                    a.RouteLateral = pr.Lateral;
                    a.RouteDist = pr.S - egoS;
                    a.SpeedAlong = RaceMath.FlatDot(new Vector3(vel.X, vel.Y, 0f), pr.Dir);
                    a.ClosingAlong = egoAlong - a.SpeedAlong;
                    if (a.ClosingAlong > 0.5f && a.RouteDist > -2f)
                        a.RouteTtc = a.RouteDist / a.ClosingAlong;
                }
            }
            catch { a.RouteValid = false; }
            Actors.Add(a);
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
