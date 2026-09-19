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
        public int Handle;        // persistent entity handle (0 = unknown/coasted)
        public int SeenCount;     // observations merged for this track
        public bool Stale;        // true when coasted (not seen this scan)
        public bool OffRoadway;   // true when provably off the drivable surface
        public Vector3 Position;
        public Vector3 Velocity;
        public float HeadingDeg;    // world heading, GTA convention
        public float HalfLengthM;   // oriented footprint half-length
        public float HalfWidthM;    // oriented footprint half-width
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

    /// Predictive perception with PERSISTENT tracking.
    ///
    /// The old implementation cleared and rebuilt a TTC-sorted top-24 world
    /// every scan. Pool flicker (an actor missing for one frame, TTC jitter
    /// reordering the truncation) then destabilized both trajectory selection
    /// and the global speed planner, and GtaDriver turned that into constant
    /// DriveTo repaths.
    ///
    /// This implementation:
    ///   - tracks entities persistently by handle with short expiry;
    ///   - coasts missed tracks for a few frames (hysteresis) instead of
    ///     deleting them, so one culled frame cannot flip the plan;
    ///   - ALWAYS retains route/path-relevant actors, the nearby safety
    ///     bubble and the rival; TTC is one threat signal, not the retention
    ///     ordering (retention + display order are stable by relevance);
    ///   - flags provably-off-roadway peds/props so paths they cannot
    ///     intersect never constrain them (the joint planner skips those).
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

        public void Reset()
        {
            Actors.Clear();
            tracks.Clear();
            LastScanMs = 0;
            prevScanMs = 0;
            ClosestTtcIndex = -1;
            AheadCount = 0;
            NearestAheadDist = 999f;
            NearestAheadTtc = 999f;
            NearestAheadClosing = 0f;
            RangeM = 120f;
        }

        // Persistent tracks by entity handle.
        private readonly Dictionary<int, PersistedTrack> tracks = new Dictionary<int, PersistedTrack>();
        private int prevScanMs;

        private sealed class PersistedTrack
        {
            public int Handle;
            public ActorKind Kind;
            public Vector3 Position;
            public Vector3 Velocity;
            public float HeadingDeg;
            public float HalfLengthM;
            public float HalfWidthM;
            public int LastSeenMs;
            public int FirstSeenMs;
            public int SeenCount;
            public bool Stale;
        }

        private struct Observation
        {
            public int Handle;
            public ActorKind Kind;
            public Vector3 Position;
            public Vector3 Velocity;
            public float HeadingDeg;
            public float HalfLengthM;
            public float HalfWidthM;
        }

        public void Update(Vehicle ego, Vehicle rivalVehicle, Ped rivalPed, float egoSpeed,
            float brakeCap, float reactionTimeS, int nowMs, int minIntervalMs = 100)
        {
            Update(ego, null, rivalVehicle, rivalPed, egoSpeed, brakeCap, reactionTimeS, nowMs, minIntervalMs, null, null);
        }

        public void Update(Vehicle ego, Vehicle rivalVehicle, Ped rivalPed, float egoSpeed,
            float brakeCap, float reactionTimeS, int nowMs, int minIntervalMs,
            RaceRoute route, RoadCorridor corridor)
        {
            Update(ego, null, rivalVehicle, rivalPed, egoSpeed, brakeCap, reactionTimeS, nowMs, minIntervalMs, route, corridor);
        }

        /// <summary>
        /// Invariant: the AI vehicle's own driver and all occupants of the ego
        /// vehicle are NEVER perception actors.
        /// The driver sits at the ego center (Dist~0, RouteDist~0) so its
        /// Euclidean clearance is 0-(1.15+0.45)=-1.6m. Without exclusion it
        /// creates a false Ped@s=0 stop conflict; earliestStopS=0-gapStop&lt;0
        /// then propagates vObs[*]=0 and zeros the entire maneuver.
        /// Primary fix is here (handle + IsInVehicle exclusion with persistent
        /// track purge); SpeedPlanner has a defensive self-zone guard as backup.
        /// Pass egoDriver (the AI driver ped) explicitly; seated occupants are
        /// also skipped via IsInVehicle so vehicle+ped duplicates never form.
        /// </summary>
        public void Update(Vehicle ego, Ped egoDriver, Vehicle rivalVehicle, Ped rivalPed, float egoSpeed,
            float brakeCap, float reactionTimeS, int nowMs, int minIntervalMs,
            RaceRoute route, RoadCorridor corridor)
        {
            if (nowMs - LastScanMs < minIntervalMs) return;
            int dtScanMs = Math.Max(20, nowMs - prevScanMs);
            float dtScanS = dtScanMs / 1000f;
            if (dtScanS > 0.6f) dtScanS = 0.6f;
            prevScanMs = nowMs;
            LastScanMs = nowMs;

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

            float aB = brakeCap > 1f ? brakeCap : 6f;
            float stop = egoSpeed * reactionTimeS + (egoSpeed * egoSpeed) / (2f * aB);
            RangeM = RaceMath.Clamp(stop + 60f, 80f, 170f);

            var left = new Vector3(-fwd.Y, fwd.X, 0f);
            float egoS = (route != null && route.Built) ? route.AlongS : 0f;
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

            // --- Ego-occupant exclusion set (invariant: never perceive self).
            // The AI driver sits at ego center; any stale/coasted track with
            // its handle would otherwise appear as Ped@s=0 with clearance
            // -(carHalf+pedHalf) ~= -1.6m. Collect handles explicitly and purge
            // persistent tracks so one culled frame cannot resurrect self.
            var egoExclude = new System.Collections.Generic.HashSet<int>();
            try
            {
                if (egoDriver != null && egoDriver.Exists())
                {
                    int hd = SafeHandle(egoDriver);
                    if (hd != 0) egoExclude.Add(hd);
                }
            }
            catch { }
            try
            {
                if (ego != null && ego.Exists())
                {
                    try
                    {
                        var ed = ego.Driver;
                        if (ed != null && ed.Exists())
                        {
                            int hd = SafeHandle(ed);
                            if (hd != 0) egoExclude.Add(hd);
                        }
                    }
                    catch { }
                    try
                    {
                        var occs = ego.Occupants;
                        if (occs != null)
                        {
                            foreach (var o in occs)
                            {
                                try
                                {
                                    if (o != null && o.Exists())
                                    {
                                        int ho = SafeHandle(o);
                                        if (ho != 0) egoExclude.Add(ho);
                                    }
                                }
                                catch { }
                            }
                        }
                    }
                    catch { }
                }
            }
            catch { }
            try
            {
                foreach (int eh in egoExclude)
                    tracks.Remove(eh);
            }
            catch { }

            // --- Gather fresh observations (handle-keyed).
            var observed = new Dictionary<int, Observation>();
            int egoHandle = 0;
            try { if (ego != null && ego.Exists()) egoHandle = SafeHandle(ego); } catch { }
            try
            {
                foreach (var v in World.GetNearbyVehicles(egoPos, RangeM))
                {
                    if (v == null || !v.Exists() || v == ego) continue;
                    int hv = SafeHandle(v);
                    if (hv != 0 && hv == egoHandle) continue;
                    int h = SafeHandle(v);
                    if (h == 0) h = FallbackKey(v.Position);
                    if (observed.ContainsKey(h)) continue;
                    observed[h] = new Observation
                    {
                        Handle = h,
                        Kind = ActorKind.TrafficVehicle,
                        Position = v.Position,
                        Velocity = SafeVelocity(v),
                        HeadingDeg = SafeHeading(v),
                        HalfLengthM = VehicleHalfLength(v),
                        HalfWidthM = VehicleHalfWidth(v),
                    };
                }
            }
            catch { }

            try
            {
                Vehicle rv = rivalVehicle;
                if (rv != null && rv.Exists() && rv != ego)
                {
                    int h = SafeHandle(rv);
                    if (h == 0) h = FallbackKey(rv.Position);
                    Vector3 rvp = rv.Position;
                    if (RaceMath.FlatDistance(egoPos, rvp) < RangeM + 30f)
                    {
                        if (observed.TryGetValue(h, out var ex))
                        {
                            ex.Kind = ActorKind.Rival;
                            observed[h] = ex;
                        }
                        else
                        {
                            observed[h] = new Observation
                            {
                                Handle = h,
                                Kind = ActorKind.Rival,
                                Position = rvp,
                                Velocity = SafeVelocity(rv),
                        HeadingDeg = SafeHeading(rv),
                        HalfLengthM = VehicleHalfLength(rv),
                        HalfWidthM = VehicleHalfWidth(rv),
                            };
                        }
                    }
                }
                else if (rivalPed != null && rivalPed.Exists())
                {
                    var rp = rivalPed.Position;
                    if (RaceMath.FlatDistance(egoPos, rp) < RangeM)
                    {
                        int h = SafeHandle(rivalPed);
                        if (h == 0) h = FallbackKey(rp);
                        Vector3 rvv = new Vector3();
                        try { rvv = rivalPed.Velocity; } catch { }
                        if (observed.TryGetValue(h, out var ex))
                        {
                            ex.Kind = ActorKind.Rival;
                            observed[h] = ex;
                        }
                        else
                        {
                            observed[h] = new Observation
                            {
                                Handle = h,
                                Kind = ActorKind.Rival,
                                Position = rp,
                                Velocity = rvv,
                                HeadingDeg = SafeHeading(rivalPed),
                                HalfLengthM = 0.45f,
                                HalfWidthM = 0.35f,
                            };
                        }
                    }
                }
            }
            catch { }

            try
            {
                float pedRange = Math.Min(RangeM, 70f);
                foreach (var p in World.GetNearbyPeds(egoPos, pedRange))
                {
                    if (p == null || !p.Exists()) continue;
                    // Skip the rival on foot here; handled above as Rival.
                    try
                    {
                        if (rivalPed != null && rivalPed.Exists() && p.Handle == rivalPed.Handle) continue;
                    }
                    catch { }
                    // Invariant: AI's own driver + all ego occupants are never actors.
                    // Explicit handle check first (covers API gaps / stale handles).
                    try
                    {
                        int ph = SafeHandle(p);
                        if (ph != 0 && egoExclude.Contains(ph)) continue;
                    }
                    catch { }
                    // Any ped seated in a vehicle is represented by its vehicle.
                    // This is the primary self-ped guard (driver at ego center)
                    // and also prevents traffic vehicle+driver duplicates.
                    try
                    {
                        if (p.IsInVehicle()) continue;
                    }
                    catch { }
                    // Belt-and-braces: ped reporting ego as its current vehicle.
                    try
                    {
                        var cv = p.CurrentVehicle;
                        if (cv != null && cv.Exists() && ego != null && ego.Exists() && cv == ego) continue;
                    }
                    catch { }
                    int h = SafeHandle(p);
                    if (h == 0) h = FallbackKey(p.Position);
                    if (observed.ContainsKey(h)) continue;
                    Vector3 pv = new Vector3();
                    try { pv = p.Velocity; } catch { }
                    observed[h] = new Observation
                    {
                        Handle = h,
                        Kind = ActorKind.Ped,
                        Position = p.Position,
                        Velocity = pv,
                        HeadingDeg = SafeHeading(p),
                        HalfLengthM = 0.45f,
                        HalfWidthM = 0.35f,
                    };
                }
            }
            catch { }

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
                    if (pd < 4f) continue;
                    int h = SafeHandle(pr);
                    if (h == 0) h = FallbackKey(pp);
                    if (observed.ContainsKey(h)) continue;
                    observed[h] = new Observation
                    {
                        Handle = h,
                        Kind = ActorKind.Obstacle,
                        Position = pp,
                        Velocity = new Vector3(),
                        HeadingDeg = SafeHeading(pr),
                        HalfLengthM = EntityHalfLength(pr, 0.8f),
                        HalfWidthM = EntityHalfWidth(pr, 0.8f),
                    };
                    added++;
                }
            }
            catch { }

            // --- Merge into persistent tracks.
            var seenNow = new HashSet<int>();
            foreach (var kv in observed)
            {
                int h = kv.Key;
                var o = kv.Value;
                seenNow.Add(h);
                PersistedTrack t;
                if (tracks.TryGetValue(h, out t))
                {
                    // Handle reuse guard: teleport of an existing id -> reset.
                    try
                    {
                        if (RaceMath.FlatDistance(t.Position, o.Position) > 25f && t.SeenCount > 2)
                        {
                            t.FirstSeenMs = nowMs;
                            t.SeenCount = 0;
                        }
                    }
                    catch { }
                    t.Position = o.Position;
                    t.Velocity = o.Velocity;
                    t.HeadingDeg = o.HeadingDeg;
                    t.HalfLengthM = o.HalfLengthM;
                    t.HalfWidthM = o.HalfWidthM;
                    t.Kind = o.Kind; // rival marking wins
                    t.LastSeenMs = nowMs;
                    t.SeenCount++;
                    t.Stale = false;
                }
                else
                {
                    tracks[h] = new PersistedTrack
                    {
                        Handle = h,
                        Kind = o.Kind,
                        Position = o.Position,
                        Velocity = o.Velocity,
                        HeadingDeg = o.HeadingDeg,
                        HalfLengthM = o.HalfLengthM,
                        HalfWidthM = o.HalfWidthM,
                        LastSeenMs = nowMs,
                        FirstSeenMs = nowMs,
                        SeenCount = 1,
                        Stale = false,
                    };
                }
            }

            // Coast + expire.
            var toRemove = new List<int>();
            foreach (var kv in tracks)
            {
                var t = kv.Value;
                if (seenNow.Contains(kv.Key)) continue;
                float expiry = ExpiryFor(t.Kind);
                if (nowMs - t.LastSeenMs > expiry)
                {
                    toRemove.Add(kv.Key);
                }
                else
                {
                    // Coast forward so one culled frame cannot flip the plan.
                    try
                    {
                        t.Position = new Vector3(
                            t.Position.X + t.Velocity.X * dtScanS,
                            t.Position.Y + t.Velocity.Y * dtScanS,
                            t.Position.Z + t.Velocity.Z * dtScanS);
                    }
                    catch { }
                    t.Stale = true;
                }
            }
            foreach (int k in toRemove)
                tracks.Remove(k);

            // --- Build the planning list with structural retention.
            Actors.Clear();
            AheadCount = 0;
            NearestAheadDist = 999f;
            NearestAheadTtc = 999f;
            NearestAheadClosing = 0f;
            ClosestTtcIndex = -1;

            var scored = new List<KeyValuePair<TrackedActor, int>>();
            foreach (var kv in tracks)
            {
                var t = kv.Value;
                var a = BuildActor(t, egoPos, egoVel, fwd, left, route, corridor, egoS, egoAlong);
                // Hysteresis: single-frame ghosts never constrain (need 2 hits),
                // but the rival is trusted immediately.
                if (t.SeenCount < 2 && t.Kind != ActorKind.Rival && !t.Stale)
                {
                    // Still keep it for display/retention, but mark stale-ish so
                    // the joint planner treats it as unconfirmed? Keep simple:
                    // keep, since expiry/coast already damps flicker. No drop.
                }
                int keepRank = RetentionRank(a, corridor);
                if (keepRank < 0) continue; // irrelevant far field
                scored.Add(new KeyValuePair<TrackedActor, int>(a, keepRank));
            }

            // Stable relevance ordering (NOT TTC): rival, route-relevant by
            // distance, nearby bubble by distance. TTC never reorders.
            scored.Sort((x, y) =>
            {
                int r = x.Value.CompareTo(y.Value);
                if (r != 0) return r;
                var ax = x.Key;
                var ay = y.Key;
                bool rx = ax.Kind == ActorKind.Rival;
                bool ry = ay.Kind == ActorKind.Rival;
                if (rx != ry) return rx ? -1 : 1;
                float dx = ax.RouteValid ? ax.RouteDist : ax.Dist + 1000f;
                float dy = ay.RouteValid ? ay.RouteDist : ay.Dist + 1000f;
                // Prefer ahead route actors, then nearest.
                bool axRel = ax.RouteValid && ax.RouteDist > -10f && ax.RouteDist < 200f;
                bool ayRel = ay.RouteValid && ay.RouteDist > -10f && ay.RouteDist < 200f;
                if (axRel != ayRel) return axRel ? -1 : 1;
                if (axRel && ayRel) return dx.CompareTo(dy);
                return ax.Dist.CompareTo(ay.Dist);
            });

            int cap = 48;
            for (int i = 0; i < scored.Count && Actors.Count < cap; i++)
                Actors.Add(scored[i].Key);

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

        private static float ExpiryFor(ActorKind kind)
        {
            switch (kind)
            {
                case ActorKind.Rival: return 900f;
                case ActorKind.Ped: return 550f;
                case ActorKind.Obstacle: return 1000f;
                default: return 700f;
            }
        }

        // Retention rank: 0 rival, 1 route/path-relevant, 2 near safety bubble,
        // 3 imminent TTC, -1 drop. Always retains the three required classes.
        private static int RetentionRank(TrackedActor a, RoadCorridor corridor)
        {
            if (a.Kind == ActorKind.Rival) return 0;
            if (a.RouteValid && a.RouteDist > -10f && a.RouteDist < 175f)
            {
                float half = corridor != null ? corridor.HalfWidthAt(Math.Max(0f, a.RouteDist)) : 7f;
                float tol = half + (a.Kind == ActorKind.Ped ? 3f : 4f);
                if (Math.Abs(a.RouteLateral) < tol) return 1;
            }
            if (a.Dist < 30f) return 2;
            if (a.Ttc < 3.5f && a.ClosingSpeed > 2f) return 3;
            // Far route actors slightly beyond the horizon are still worth
            // keeping for the braking preview; everything else drops.
            if (a.RouteValid && a.RouteDist >= 175f && a.RouteDist < 230f) return 3;
            return -1;
        }

        public bool TryGetClosestThreat(out TrackedActor actor)
        {
            actor = new TrackedActor();
            if (ClosestTtcIndex >= 0 && ClosestTtcIndex < Actors.Count)
            {
                actor = Actors[ClosestTtcIndex];
                return true;
            }
            float bd = 999f;
            bool found = false;
            foreach (var a in Actors)
            {
                if (a.IsAhead && a.Dist < bd) { bd = a.Dist; actor = a; found = true; }
            }
            return found;
        }

        /// Nearest actor ahead ON THE ROUTE (not just in the nose cone).
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
                a.Position.Z + a.Velocity.Z * dt);
        }

        private TrackedActor BuildActor(PersistedTrack t,
            Vector3 egoPos, Vector3 egoVel, Vector3 fwd, Vector3 left,
            RaceRoute route, RoadCorridor corridor, float egoS, float egoAlong)
        {
            Vector3 pos = t.Position;
            Vector3 vel = t.Velocity;
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
                closing = -(rel.X * dir.X + rel.Y * dir.Y);
                if (closing > 0.5f) ttc = dist / closing;
            }
            float spd = RaceMath.FlatLength(vel);
            var a = new TrackedActor
            {
                Valid = true,
                Kind = t.Kind,
                Handle = t.Handle,
                SeenCount = t.SeenCount,
                Stale = t.Stale,
                OffRoadway = false,
                Position = pos,
                Velocity = vel,
                HeadingDeg = t.HeadingDeg,
                HalfLengthM = t.HalfLengthM > 0.1f ? t.HalfLengthM : (t.Kind == ActorKind.TrafficVehicle || t.Kind == ActorKind.Rival ? 2.3f : 0.5f),
                HalfWidthM = t.HalfWidthM > 0.1f ? t.HalfWidthM : (t.Kind == ActorKind.TrafficVehicle || t.Kind == ActorKind.Rival ? 1.0f : 0.5f),
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
                    try
                    {
                        if (corridor != null && a.RouteDist > -10f && a.RouteDist < 230f)
                        {
                            float half = corridor.HalfWidthAt(Math.Max(0f, a.RouteDist));
                            float tol = half + (t.Kind == ActorKind.Ped || t.Kind == ActorKind.Obstacle ? 2f : 2f);
                            a.OffRoadway = Math.Abs(a.RouteLateral) > tol;
                        }
                    }
                    catch { }
                }
            }
            catch { a.RouteValid = false; }
            return a;
        }

        private static float SafeHeading(Entity e)
        {
            try { return e.Heading; }
            catch { return 0f; }
        }

        private static float VehicleHalfLength(Vehicle v)
        {
            return EntityHalfLength(v, 2.3f);
        }

        private static float VehicleHalfWidth(Vehicle v)
        {
            return EntityHalfWidth(v, 1.0f);
        }

        private static float EntityHalfLength(Entity e, float fallback)
        {
            try
            {
                Vector3 min;
                Vector3 max;
                e.Model.GetDimensions(out min, out max);
                float len = Math.Abs(max.Y - min.Y);
                if (len > 0.2f && len < 30f) return len * 0.5f;
            }
            catch { }
            return fallback;
        }

        private static float EntityHalfWidth(Entity e, float fallback)
        {
            try
            {
                Vector3 min;
                Vector3 max;
                e.Model.GetDimensions(out min, out max);
                float width = Math.Abs(max.X - min.X);
                if (width > 0.2f && width < 15f) return width * 0.5f;
            }
            catch { }
            return fallback;
        }

        private static int SafeHandle(Entity e)
        {
            try { return e.Handle; }
            catch { return 0; }
        }

        private static int FallbackKey(Vector3 p)
        {
            // Entities without a readable handle (should be rare): quantize
            // position into a negative pseudo-key so repeated scans merge.
            try
            {
                int qx = (int)Math.Floor(p.X / 2f);
                int qy = (int)Math.Floor(p.Y / 2f);
                int k = (qx * 73856093) ^ (qy * 19349663);
                k = Math.Abs(k % 1000000) + 1;
                return -k;
            }
            catch { return -1; }
        }

        private static Vector3 SafeVelocity(Entity e)
        {
            try { return e.Velocity; }
            catch { return new Vector3(); }
        }
    }
}
