using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Local 2.5D world representation used by SpatialPlannerV1.
    ///
    /// XY is searched freely; Z is derived from a connected local road surface.
    /// Road supports also carry a soft traffic-flow preference. Opposing lanes
    /// stay usable for racing/overtakes, but are no longer free real estate.
    internal sealed class LocalWorldModel
    {
        internal struct RoadSupport
        {
            public Vector3 Center;
            public float HeadingDeg;          // structural road axis
            public float PreferredHeadingDeg; // route-progress direction
            public float Grade;               // dz / horizontal meter
            public float HalfLengthM;
            public float LeftM;
            public float RightM;
            public float Confidence;
            public int Lanes;
            public int ForwardLanes;
            public int BackwardLanes;
            public float MedianWidth;
            public float FlowConfidence;
            public int SurfaceComponentId; // connected 2.5D surface layer
            public string Source;
        }

        internal struct DebugCell
        {
            public Vector3 Position;
            public float SurfaceCost;
            public float FlowCost;
            public bool OnRoad;
            public bool Occupied;
        }

        internal struct PoseCost
        {
            public bool HardCollision;
            public float SurfaceCost;
            public float FlowCost;
            public float ActorCost;
            public float ClearanceM;
            public int BlockingHandle;
            public bool OnRoad;
            public float RoadConfidence;
            public float SurfaceZ;
            public float Grade;
            public bool OpposingSide;
            public int SurfaceComponentId;
        }

        private struct SurfaceQuery
        {
            public bool Found;
            public int Index;
            public float SignedOutside;
            public float Confidence;
            public float SurfaceZ;
            public float Grade;
            public float FlowCost;
            public bool OpposingSide;
            public int SurfaceComponentId;
        }

        public readonly List<RoadSupport> Road = new List<RoadSupport>();
        public readonly List<DebugCell> DebugCells = new List<DebugCell>();
        public string Detail { get; private set; } = "";

        private struct CachedRoadProbe
        {
            public Vector3 Center;
            public float HeadingDeg;
            public float PreferredHeadingDeg;
            public int SeenMs;
        }

        private readonly List<TrackedActor> actors = new List<TrackedActor>();
        private readonly List<CachedRoadProbe> positiveRoadProbeCache = new List<CachedRoadProbe>(180);
        private Perception perception;
        private int positiveProbeTests;
        private int positiveProbeHits;
        private Vector3 egoOrigin;
        private Vector3 egoForward;
        private Vector3 egoLeft;
        private float egoHalfLength = 2.3f;
        private float egoHalfWidth = 1.0f;

        public void Reset()
        {
            ClearFrame();
            positiveRoadProbeCache.Clear();
        }

        private void ClearFrame()
        {
            Road.Clear();
            DebugCells.Clear();
            actors.Clear();
            perception = null;
            positiveProbeTests = 0;
            positiveProbeHits = 0;
            Detail = "";
        }

        public void Build(
            DrivingReference.Result reference,
            Perception perception,
            Vector3 egoPos,
            float egoHeading,
            float egoHalfLength,
            float egoHalfWidth,
            bool buildDebugGrid = false)
        {
            ClearFrame();
            this.perception = perception;
            egoOrigin = egoPos;
            egoForward = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(egoHeading));
            egoLeft = new Vector3(-egoForward.Y, egoForward.X, 0f);
            this.egoHalfLength = RaceMath.Clamp(egoHalfLength, 1.5f, 4.5f);
            this.egoHalfWidth = RaceMath.Clamp(egoHalfWidth, 0.75f, 1.8f);

            AddReferenceSupports(reference);
            AddNearbyNodeSupports(egoPos, egoHeading);

            // Vehicle nodes and GET_CLOSEST_ROAD describe road structure, but
            // their inferred lane-width rectangles routinely under-fill broad
            // asphalt, junction aprons and highway merges. IS_POINT_ON_ROAD is
            // deliberately used as POSITIVE evidence only: a true sample may
            // expand traversable space, while a false sample never contracts it.
            // Probe hits persist briefly so the local surface is not rebuilt from
            // scratch every reaction tick.
            AddPositiveRoadProbes(reference, egoPos, egoHeading);
            AddCachedRoadProbes(egoPos);
            MergeRedundantSupports();
            int surfaceComponents = AssignSurfaceComponents();

            if (perception != null)
            {
                for (int i = 0; i < perception.Actors.Count; i++)
                {
                    var a = perception.Actors[i];
                    if (!a.Valid || a.Dist > 110f) continue;
                    actors.Add(a);
                }
            }

            if (buildDebugGrid) BuildDebugGrid();
            float maxAbsGrade = 0f;
            int alignedSupports = 0;
            for (int i = 0; i < Road.Count; i++)
            {
                float g = Math.Abs(Road[i].Grade);
                if (g > maxAbsGrade) maxAbsGrade = g;
                if (Road[i].FlowConfidence > 0.25f) alignedSupports++;
            }
            int probeSupports = 0;
            for (int i = 0; i < Road.Count; i++)
                if (Road[i].Source == "RoadProbe") probeSupports++;
            Detail = $"roadSupports={Road.Count};surfaceComponents={surfaceComponents};"
                + $"flowSupports={alignedSupports};probeSupports={probeSupports};"
                + $"probeHit={positiveProbeHits}/{positiveProbeTests};probeCache={positiveRoadProbeCache.Count};"
                + $"actors={actors.Count};maxGrade={maxAbsGrade:F2};debugCells={DebugCells.Count}";
        }

        public int LocateSurfaceComponent(Vector3 pos, float headingDeg)
        {
            var q = QuerySurface(pos, headingDeg, pos.Z, true, -1);
            return q.Found ? q.SurfaceComponentId : -1;
        }

        public bool TryProjectToSurface(
            Vector3 candidate,
            float headingDeg,
            float previousZ,
            out Vector3 projected,
            out float flowCost,
            out float confidence,
            out bool opposingSide)
        {
            int ignored;
            return TryProjectToSurface(candidate, headingDeg, previousZ, -1,
                out projected, out flowCost, out confidence, out opposingSide, out ignored);
        }

        public bool TryProjectToSurface(
            Vector3 candidate,
            float headingDeg,
            float previousZ,
            int surfaceComponentHint,
            out Vector3 projected,
            out float flowCost,
            out float confidence,
            out bool opposingSide,
            out int surfaceComponentId)
        {
            var q = QuerySurface(candidate, headingDeg, previousZ, true, surfaceComponentHint);
            if (!q.Found)
            {
                projected = candidate;
                flowCost = 0f;
                confidence = 0f;
                opposingSide = false;
                surfaceComponentId = surfaceComponentHint;
                return false;
            }

            projected = new Vector3(candidate.X, candidate.Y, q.SurfaceZ);
            flowCost = q.FlowCost;
            confidence = q.Confidence;
            opposingSide = q.OpposingSide;
            surfaceComponentId = q.SurfaceComponentId;
            return true;
        }

        public PoseCost EvaluatePose(Vector3 pos, float headingDeg, float timeS)
        {
            return EvaluatePose(pos, headingDeg, timeS, -1);
        }

        public PoseCost EvaluatePose(
            Vector3 pos, float headingDeg, float timeS, int surfaceComponentHint)
        {
            var q = QuerySurface(pos, headingDeg, pos.Z, false, surfaceComponentHint);
            var result = new PoseCost
            {
                HardCollision = false,
                SurfaceCost = 0f,
                FlowCost = 0f,
                ActorCost = 0f,
                ClearanceM = 999f,
                BlockingHandle = -1,
                OnRoad = q.Found,
                RoadConfidence = q.Confidence,
                SurfaceZ = q.Found ? q.SurfaceZ : pos.Z,
                Grade = q.Grade,
                OpposingSide = q.OpposingSide,
                SurfaceComponentId = q.SurfaceComponentId,
            };

            if (q.Found)
            {
                result.SurfaceCost = (1f - q.Confidence) * 2.0f;
                if (q.SignedOutside > -0.7f)
                    result.SurfaceCost += (q.SignedOutside + 0.7f) * 0.9f;
                result.FlowCost = q.FlowCost;
            }
            else
            {
                // Unknown/non-road remains searchable, but expensive. Distance
                // from the nearest structural support increases that cost.
                result.SurfaceCost = 14.0f
                    + Math.Min(24f, Math.Max(0f, q.SignedOutside) * 2.2f);
            }

            for (int i = 0; i < actors.Count; i++)
            {
                var a = actors[i];
                float clear = ActorClearance(a, pos, headingDeg, timeS);
                if (clear < result.ClearanceM)
                {
                    result.ClearanceM = clear;
                    result.BlockingHandle = a.Handle;
                }

                if (a.Kind == ActorKind.Debris)
                {
                    // Small road junk is a preference, not a wall. Hitting it
                    // is allowed when the alternative is traffic/oncoming.
                    float caution = 0.9f;
                    if (clear < caution)
                    {
                        float x = caution - clear;
                        result.ActorCost += x * x * 1.2f;
                    }
                    continue;
                }

                float hardMargin = a.Kind == ActorKind.Ped ? 0.9f : 0.15f;
                if (clear <= hardMargin)
                {
                    result.HardCollision = true;
                    result.ActorCost += a.Kind == ActorKind.Ped ? 5000f : 1500f;
                }
                else
                {
                    bool movingVehicle = a.Kind == ActorKind.TrafficVehicle
                        || a.Kind == ActorKind.Rival;
                    bool sameFlow = false;
                    if (movingVehicle && a.Speed > 2.0f)
                    {
                        float actorHeading = a.HeadingDeg;
                        if (RaceMath.FlatLength(a.Velocity) > 1.2f)
                            actorHeading = RaceMath.HeadingFromVector(
                                RaceMath.FlatNormalize(a.Velocity));
                        sameFlow = Math.Abs(
                            RaceMath.HeadingDiffDeg(actorHeading, headingDeg)) < 35f;
                    }

                    // Parallel moving traffic is not a circular exclusion zone.
                    // Keep true footprint collision hard, but allow racing-close
                    // side-by-side gaps without paying the same 2.8 m halo used
                    // for oncoming/static hazards.
                    float caution = a.Kind == ActorKind.Ped ? 4.5f
                        : sameFlow ? 1.45f : 2.8f;
                    float weight = a.Kind == ActorKind.Ped ? 16f
                        : sameFlow ? 1.6f : 5f;
                    if (clear < caution)
                    {
                        float x = caution - clear;
                        result.ActorCost += x * x * weight;
                    }
                }
            }

            return result;
        }

        public bool TryGetActor(int handle, out TrackedActor actor)
        {
            for (int i = 0; i < actors.Count; i++)
            {
                if (actors[i].Handle == handle)
                {
                    actor = actors[i];
                    return true;
                }
            }
            actor = new TrackedActor();
            return false;
        }

        public float ActorClearance(TrackedActor a, Vector3 egoPos, float egoHeadingDeg, float timeS)
        {
            Vector3 ap;
            try
            {
                ap = perception != null
                    ? perception.Predict(a, RaceMath.Clamp(timeS, 0f, 5f))
                    : new Vector3(
                        a.Position.X + a.Velocity.X * timeS,
                        a.Position.Y + a.Velocity.Y * timeS,
                        a.Position.Z + a.Velocity.Z * timeS);
            }
            catch { ap = a.Position; }

            if (Math.Abs(ap.Z - egoPos.Z) > 4.5f)
                return 999f;

            float actorHL = a.HalfLengthM > 0.2f ? a.HalfLengthM : 2.3f;
            float actorHW = a.HalfWidthM > 0.2f ? a.HalfWidthM : 1.0f;
            float dx = egoPos.X - ap.X;
            float dy = egoPos.Y - ap.Y;
            float centerDistSq = dx * dx + dy * dy;
            float broadRadius = actorHL + actorHW + egoHalfLength + egoHalfWidth + 5f;
            if (centerDistSq > broadRadius * broadRadius)
            {
                float centerDist = (float)Math.Sqrt(centerDistSq);
                return Math.Max(0f, centerDist - (actorHL + egoHalfLength));
            }

            float actorHeading = a.HeadingDeg;
            if (RaceMath.FlatLength(a.Velocity) > 1.2f)
                actorHeading = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(a.Velocity));

            Vector3 af = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(actorHeading));
            Vector3 al = new Vector3(-af.Y, af.X, 0f);
            Vector3 ef = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(egoHeadingDeg));
            Vector3 el = new Vector3(-ef.Y, ef.X, 0f);

            float egoOnAF = egoHalfLength * Math.Abs(RaceMath.FlatDot(ef, af))
                + egoHalfWidth * Math.Abs(RaceMath.FlatDot(el, af));
            float egoOnAL = egoHalfLength * Math.Abs(RaceMath.FlatDot(ef, al))
                + egoHalfWidth * Math.Abs(RaceMath.FlatDot(el, al));

            float stalePad = a.Stale ? 0.45f : 0.15f;
            float hl = actorHL + egoOnAF + stalePad;
            float hw = actorHW + egoOnAL + stalePad;

            Vector3 rel = new Vector3(egoPos.X - ap.X, egoPos.Y - ap.Y, 0f);
            float x = Math.Abs(RaceMath.FlatDot(rel, af)) - hl;
            float y = Math.Abs(RaceMath.FlatDot(rel, al)) - hw;

            if (x <= 0f && y <= 0f)
                return Math.Max(x, y);

            float ox = Math.Max(0f, x);
            float oy = Math.Max(0f, y);
            return (float)Math.Sqrt(ox * ox + oy * oy);
        }

        private void AddReferenceSupports(DrivingReference.Result reference)
        {
            if (reference == null || reference.Path == null || reference.Path.Count < 2) return;

            for (int i = 0; i < reference.Path.Count; i++)
            {
                Vector3 refDir = DirectionAt(reference.Path, i);
                float preferredHeading = RaceMath.HeadingFromVector(refDir);
                Vector3 center = i < reference.RoadCenter.Count
                    ? reference.RoadCenter[i]
                    : reference.Path[i];
                float heading = i < reference.RoadHeadingDeg.Count
                    ? reference.RoadHeadingDeg[i]
                    : preferredHeading;
                float left = i < reference.LeftRoadM.Count ? reference.LeftRoadM[i] : 3.5f;
                float right = i < reference.RightRoadM.Count ? reference.RightRoadM[i] : 3.5f;
                float conf = i < reference.RoadConfidence.Count ? reference.RoadConfidence[i] : 0.35f;
                int lanes = i < reference.RoadLaneCount.Count ? reference.RoadLaneCount[i] : 2;
                int forward = i < reference.RoadForwardLanes.Count ? reference.RoadForwardLanes[i] : 0;
                int backward = i < reference.RoadBackwardLanes.Count ? reference.RoadBackwardLanes[i] : 0;
                float median = i < reference.RoadMedianWidth.Count ? reference.RoadMedianWidth[i] : 0f;

                float halfLen = 8f;
                if (i + 1 < reference.Path.Count)
                    halfLen = Math.Max(
                        halfLen,
                        RaceMath.FlatDistance(reference.Path[i], reference.Path[i + 1]) * 0.8f + 5f);

                float grade = GradeAt(reference.Path, i);
                float flowConfidence = forward > 0 && backward > 0
                    ? Math.Min(conf, 0.92f)
                    : (lanes >= 3 ? Math.Min(conf, 0.35f) : 0.15f);

                Road.Add(new RoadSupport
                {
                    Center = center,
                    HeadingDeg = heading,
                    PreferredHeadingDeg = preferredHeading,
                    Grade = grade,
                    HalfLengthM = halfLen,
                    LeftM = RaceMath.Clamp(left, 1.7f, 16f),
                    RightM = RaceMath.Clamp(right, 1.7f, 16f),
                    Confidence = RaceMath.Clamp(conf, 0f, 1f),
                    Lanes = Math.Max(1, lanes),
                    ForwardLanes = Math.Max(0, forward),
                    BackwardLanes = Math.Max(0, backward),
                    MedianWidth = RaceMath.Clamp(median, 0f, 8f),
                    FlowConfidence = flowConfidence,
                    Source = "Reference",
                });
            }
        }

        private void AddPositiveRoadProbes(
            DrivingReference.Result reference, Vector3 egoPos, float egoHeading)
        {
            int now = 0;
            try { now = Game.GameTime; } catch { }
            TrimPositiveRoadProbeCache(egoPos, now);

            // Roughly 30-45 native tests per plan at normal lookahead. This is
            // intentionally sparse: cached positive hits turn them into a local
            // free-space memory rather than a dense per-frame width scan.
            if (reference != null && reference.Path != null && reference.Path.Count >= 2)
            {
                float[] lateralOffsets = { -12f, -8f, -4f, 4f, 8f, 12f };
                for (int i = 0; i < reference.Path.Count; i += 3)
                {
                    Vector3 center = reference.Path[i];
                    Vector3 dir = DirectionAt(reference.Path, i);
                    float preferredHeading = RaceMath.HeadingFromVector(dir);
                    float roadHeading = i < reference.RoadHeadingDeg.Count
                        ? reference.RoadHeadingDeg[i]
                        : preferredHeading;
                    Vector3 axis = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(roadHeading));
                    Vector3 left = new Vector3(-axis.Y, axis.X, 0f);

                    for (int j = 0; j < lateralOffsets.Length; j++)
                    {
                        float lat = lateralOffsets[j];
                        Vector3 p = new Vector3(
                            center.X + left.X * lat,
                            center.Y + left.Y * lat,
                            center.Z);
                        ProbePositiveRoadPoint(p, roadHeading, preferredHeading, now);
                    }
                }
            }
            else
            {
                // Terminal/reference-loss fallback: still learn positive space
                // immediately around the car, but never infer negative space.
                Vector3 fwd = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(egoHeading));
                Vector3 left = new Vector3(-fwd.Y, fwd.X, 0f);
                float[] forwards = { 0f, 10f, 20f, 30f, 40f };
                float[] laterals = { -10f, -5f, 5f, 10f };
                for (int i = 0; i < forwards.Length; i++)
                {
                    for (int j = 0; j < laterals.Length; j++)
                    {
                        Vector3 p = new Vector3(
                            egoPos.X + fwd.X * forwards[i] + left.X * laterals[j],
                            egoPos.Y + fwd.Y * forwards[i] + left.Y * laterals[j],
                            egoPos.Z);
                        ProbePositiveRoadPoint(p, egoHeading, egoHeading, now);
                    }
                }
            }
        }

        private void ProbePositiveRoadPoint(
            Vector3 p, float roadHeading, float preferredHeading, int now)
        {
            bool onRoad = false;
            positiveProbeTests++;
            try
            {
                onRoad = Function.Call<bool>(
                    Hash.IS_POINT_ON_ROAD, p.X, p.Y, p.Z + 0.5f, 0);
            }
            catch { }
            if (!onRoad) return;

            positiveProbeHits++;
            for (int i = 0; i < positiveRoadProbeCache.Count; i++)
            {
                CachedRoadProbe old = positiveRoadProbeCache[i];
                if (RaceMath.FlatDistance(old.Center, p) > 2.75f) continue;
                if (AxisHeadingError(old.HeadingDeg, roadHeading) > 30f) continue;
                old.Center = p;
                old.HeadingDeg = roadHeading;
                old.PreferredHeadingDeg = preferredHeading;
                old.SeenMs = now;
                positiveRoadProbeCache[i] = old;
                return;
            }

            positiveRoadProbeCache.Add(new CachedRoadProbe
            {
                Center = p,
                HeadingDeg = roadHeading,
                PreferredHeadingDeg = preferredHeading,
                SeenMs = now,
            });
            if (positiveRoadProbeCache.Count > 180)
                positiveRoadProbeCache.RemoveAt(0);
        }

        private void TrimPositiveRoadProbeCache(Vector3 egoPos, int now)
        {
            for (int i = positiveRoadProbeCache.Count - 1; i >= 0; i--)
            {
                CachedRoadProbe p = positiveRoadProbeCache[i];
                bool stale = now > 0 && p.SeenMs > 0 && now - p.SeenMs > 12000;
                bool far = RaceMath.FlatDistance(egoPos, p.Center) > 140f;
                if (stale || far)
                    positiveRoadProbeCache.RemoveAt(i);
            }
        }

        private void AddCachedRoadProbes(Vector3 egoPos)
        {
            for (int i = 0; i < positiveRoadProbeCache.Count; i++)
            {
                CachedRoadProbe p = positiveRoadProbeCache[i];
                if (RaceMath.FlatDistance(egoPos, p.Center) > 125f) continue;
                Road.Add(new RoadSupport
                {
                    Center = p.Center,
                    HeadingDeg = p.HeadingDeg,
                    PreferredHeadingDeg = p.PreferredHeadingDeg,
                    Grade = 0f,
                    HalfLengthM = 5.5f,
                    LeftM = 2.6f,
                    RightM = 2.6f,
                    Confidence = 0.52f,
                    Lanes = 1,
                    ForwardLanes = 0,
                    BackwardLanes = 0,
                    MedianWidth = 0f,
                    FlowConfidence = 0f,
                    Source = "RoadProbe",
                });
            }
        }

        private void AddNearbyNodeSupports(Vector3 egoPos, float egoHeading)
        {
            for (int nth = 1; nth <= 24; nth++)
            {
                try
                {
                    Vector3 p;
                    float h;
                    int lanes;
                    bool ok = PathFind.GetNthClosestVehicleNodePositionWithHeading(
                        egoPos, nth, out p, out h, out lanes);
                    if (!ok) continue;

                    float d = RaceMath.FlatDistance(egoPos, p);
                    if (d > 95f || Math.Abs(p.Z - egoPos.Z) > 12f) continue;

                    lanes = Math.Max(1, Math.Min(8, lanes));
                    float half = RaceMath.Clamp(lanes * 3.25f * 0.5f + 0.55f, 2.0f, 14f);
                    float conf = 0.52f - RaceMath.Clamp(d / 95f, 0f, 1f) * 0.17f;
                    float axis = AxisHeadingError(h, egoHeading);
                    if (axis < 40f) conf += 0.08f;

                    Road.Add(new RoadSupport
                    {
                        Center = p,
                        HeadingDeg = h,
                        PreferredHeadingDeg = AlignHeadingAxis(h, egoHeading),
                        Grade = 0f,
                        HalfLengthM = 13f,
                        LeftM = half,
                        RightM = half,
                        Confidence = RaceMath.Clamp(conf, 0.25f, 0.62f),
                        Lanes = lanes,
                        ForwardLanes = 0,
                        BackwardLanes = 0,
                        MedianWidth = 0f,
                        FlowConfidence = 0.08f,
                        Source = "NearbyNode",
                    });
                }
                catch { }
            }
        }

        private void MergeRedundantSupports()
        {
            var merged = new List<RoadSupport>();
            for (int i = 0; i < Road.Count; i++)
            {
                var s = Road[i];
                bool redundant = false;
                for (int j = 0; j < merged.Count; j++)
                {
                    var m = merged[j];
                    bool probePair = s.Source == "RoadProbe" || m.Source == "RoadProbe";
                    float mergeDistance = probePair ? 2.2f : 5.5f;
                    if (RaceMath.FlatDistance(s.Center, m.Center) > mergeDistance) continue;
                    if (AxisHeadingError(s.HeadingDeg, m.HeadingDeg) > 22f) continue;
                    if (s.Confidence <= m.Confidence + 0.10f)
                    {
                        redundant = true;
                        break;
                    }
                }
                if (!redundant) merged.Add(s);
            }
            Road.Clear();
            Road.AddRange(merged);
        }

        private int AssignSurfaceComponents()
        {
            int n = Road.Count;
            if (n == 0) return 0;
            var component = new int[n];
            for (int i = 0; i < n; i++) component[i] = -1;
            var queue = new Queue<int>();
            int nextId = 0;

            for (int seed = 0; seed < n; seed++)
            {
                if (component[seed] >= 0) continue;
                component[seed] = nextId;
                queue.Enqueue(seed);

                while (queue.Count > 0)
                {
                    int a = queue.Dequeue();
                    for (int b = 0; b < n; b++)
                    {
                        if (component[b] >= 0 || a == b) continue;
                        if (!SupportsConnect(Road[a], Road[b])) continue;
                        component[b] = nextId;
                        queue.Enqueue(b);
                    }
                }
                nextId++;
            }

            for (int i = 0; i < n; i++)
            {
                var s = Road[i];
                s.SurfaceComponentId = component[i];
                Road[i] = s;
            }
            return nextId;
        }

        private static bool SupportsConnect(RoadSupport a, RoadSupport b)
        {
            float flat = RaceMath.FlatDistance(a.Center, b.Center);
            float maxReach = Math.Min(34f, a.HalfLengthM + b.HalfLengthM + 5f);
            if (flat > maxReach) return false;

            // Estimate each support's surface height at the other's center.
            Vector3 af = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(a.HeadingDeg));
            Vector3 bf = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(b.HeadingDeg));
            Vector3 ab = new Vector3(b.Center.X - a.Center.X, b.Center.Y - a.Center.Y, 0f);
            float za = a.Center.Z + RaceMath.FlatDot(ab, af) * a.Grade;
            float zb = b.Center.Z + RaceMath.FlatDot(
                new Vector3(a.Center.X - b.Center.X, a.Center.Y - b.Center.Y, 0f), bf) * b.Grade;
            float dz = Math.Min(Math.Abs(za - b.Center.Z), Math.Abs(zb - a.Center.Z));
            float allowedDz = 1.8f + flat * 0.12f;
            if (dz > allowedDz) return false;

            // Sequential/overlapping rectangles on the same physical surface
            // connect. Parallel streets separated laterally do not. Crossing
            // supports connect only when their actual layers are vertically
            // compatible (checked above), which preserves real intersections.
            float outAB = OutsideSupportXY(a, b.Center);
            float outBA = OutsideSupportXY(b, a.Center);
            if (outAB <= 3.5f || outBA <= 3.5f) return true;

            // At a junction, centers may sit just outside both short support
            // rectangles. Allow a small endpoint gap when axes differ.
            float axis = AxisHeadingError(a.HeadingDeg, b.HeadingDeg);
            return axis > 25f && flat < 12f;
        }

        private static float OutsideSupportXY(RoadSupport s, Vector3 p)
        {
            Vector3 f = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(s.HeadingDeg));
            Vector3 l = new Vector3(-f.Y, f.X, 0f);
            Vector3 rel = new Vector3(p.X - s.Center.X, p.Y - s.Center.Y, 0f);
            float along = Math.Abs(RaceMath.FlatDot(rel, f)) - s.HalfLengthM;
            float latSigned = RaceMath.FlatDot(rel, l);
            float lat = latSigned >= 0f ? latSigned - s.LeftM : -latSigned - s.RightM;
            return Math.Max(along, lat);
        }

        private SurfaceQuery QuerySurface(
            Vector3 p,
            float headingDeg,
            float zHint,
            bool projectionMode,
            int surfaceComponentHint)
        {
            var best = new SurfaceQuery
            {
                Found = false,
                Index = -1,
                SignedOutside = 999f,
                Confidence = 0f,
                SurfaceZ = p.Z,
                Grade = 0f,
                FlowCost = 0f,
                OpposingSide = false,
                SurfaceComponentId = -1,
            };
            float bestScore = float.MaxValue;

            for (int i = 0; i < Road.Count; i++)
            {
                var s = Road[i];
                if (surfaceComponentHint >= 0 && s.SurfaceComponentId != surfaceComponentHint)
                    continue;
                float broadReach = s.HalfLengthM + Math.Max(s.LeftM, s.RightM) + 3f;
                float dx = p.X - s.Center.X;
                float dy = p.Y - s.Center.Y;
                if (dx * dx + dy * dy > broadReach * broadReach) continue;

                Vector3 f = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(s.HeadingDeg));
                Vector3 l = new Vector3(-f.Y, f.X, 0f);
                Vector3 rel = new Vector3(p.X - s.Center.X, p.Y - s.Center.Y, 0f);
                float alongSigned = RaceMath.FlatDot(rel, f);
                float along = Math.Abs(alongSigned);
                float lat = RaceMath.FlatDot(rel, l);

                float outsideAlong = along - s.HalfLengthM;
                float outsideLat = lat >= 0f ? lat - s.LeftM : -lat - s.RightM;
                float outside = Math.Max(outsideAlong, outsideLat);
                bool inside = outside <= 0f;

                float surfaceZ = s.Center.Z + alongSigned * s.Grade;
                float dz = Math.Abs(zHint - surfaceZ);
                float maxDz = projectionMode ? 5.5f : 6.5f;
                if (dz > maxDz) continue;

                // Prefer the connected vertical layer first, then high-confidence
                // structural support. This prevents XY crossings from snapping
                // between an overpass and the road below.
                float score = dz * 2.6f
                    + Math.Max(0f, outside) * 3.0f
                    + (1f - s.Confidence) * 2.0f;
                if (!inside) score += 4f;

                // Once an actual containing surface exists, an outside-nearby
                // support can never displace it just because its center/Z is a
                // little closer.
                if (best.Found && !inside) continue;
                if (!best.Found && inside)
                    score -= 100f;
                if (score >= bestScore) continue;

                bestScore = score;
                best = new SurfaceQuery
                {
                    Found = inside,
                    Index = i,
                    SignedOutside = outside,
                    Confidence = s.Confidence,
                    SurfaceZ = surfaceZ,
                    Grade = s.Grade,
                    FlowCost = 0f,
                    OpposingSide = false,
                    SurfaceComponentId = s.SurfaceComponentId,
                };
            }

            // Traversability and traffic flow are deliberately separate.
            // Positive road probes may establish "there is asphalt here", but
            // they never invent lane direction. Flow is annotated from the
            // nearest structural (reference/node) support on the same surface.
            if (best.Found)
            {
                float flowCost;
                bool opposing;
                QueryFlowAnnotation(
                    p, headingDeg, zHint, best.SurfaceComponentId,
                    out flowCost, out opposing);
                best.FlowCost = flowCost;
                best.OpposingSide = opposing;
            }
            return best;
        }

        private void QueryFlowAnnotation(
            Vector3 p,
            float headingDeg,
            float zHint,
            int surfaceComponentHint,
            out float cost,
            out bool opposing)
        {
            cost = 0f;
            opposing = false;
            float bestScore = float.MaxValue;
            int best = -1;
            float bestLat = 0f;

            for (int i = 0; i < Road.Count; i++)
            {
                RoadSupport s = Road[i];
                if (s.Source == "RoadProbe" || s.FlowConfidence <= 0.05f) continue;
                if (surfaceComponentHint >= 0 && s.SurfaceComponentId != surfaceComponentHint)
                    continue;

                Vector3 f = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(s.HeadingDeg));
                Vector3 l = new Vector3(-f.Y, f.X, 0f);
                Vector3 rel = new Vector3(p.X - s.Center.X, p.Y - s.Center.Y, 0f);
                float alongSigned = RaceMath.FlatDot(rel, f);
                float along = Math.Abs(alongSigned);
                float lat = RaceMath.FlatDot(rel, l);
                float outsideAlong = along - s.HalfLengthM;
                float outsideLat = lat >= 0f ? lat - s.LeftM : -lat - s.RightM;
                float outside = Math.Max(outsideAlong, outsideLat);
                if (outside > 8f) continue;

                float surfaceZ = s.Center.Z + alongSigned * s.Grade;
                float dz = Math.Abs(zHint - surfaceZ);
                if (dz > 7f) continue;

                float score = Math.Max(0f, outside) * 2.5f
                    + dz * 1.8f
                    + (1f - s.FlowConfidence) * 1.5f;
                if (outside <= 0f) score -= 4f;
                if (score >= bestScore) continue;
                bestScore = score;
                best = i;
                bestLat = lat;
            }

            if (best >= 0)
                EvaluateFlow(Road[best], bestLat, headingDeg, out cost, out opposing);
        }

        private static void EvaluateFlow(
            RoadSupport s,
            float lateral,
            float headingDeg,
            out float cost,
            out bool opposing)
        {
            cost = 0f;
            opposing = false;
            if (s.FlowConfidence <= 0.05f) return;

            float headErr = Math.Abs(RaceMath.HeadingDiffDeg(s.PreferredHeadingDeg, headingDeg));
            if (headErr > 100f)
            {
                // Driving against route progress should be costly regardless
                // of which half of the road we occupy.
                cost += 3.0f * s.FlowConfidence;
            }

            if (s.ForwardLanes > 0 && s.BackwardLanes > 0)
            {
                // GTA traffic is right-hand. With PreferredHeading aligned to
                // route progress, positive LEFT lateral is the opposing side.
                float medianHalf = s.MedianWidth * 0.5f;
                float intoOpposing = lateral - medianHalf;
                if (intoOpposing > 0f)
                {
                    opposing = true;
                    cost += s.FlowConfidence * (0.9f + Math.Min(3.5f, intoOpposing * 0.28f));
                }
            }
            else if (s.Lanes >= 3)
            {
                // Split is unknown: only a weak side preference.
                if (lateral > 1.0f)
                    cost += s.FlowConfidence * Math.Min(0.65f, lateral * 0.10f);
            }
        }

        private void BuildDebugGrid()
        {
            DebugCells.Clear();
            float egoHeading = RaceMath.HeadingFromVector(egoForward);

            // March each lateral strip forward on the connected surface.
            // Sampling every cell from ego Z makes uphill/downhill road appear
            // unknown once elevation changes beyond the surface-layer gate.
            for (float lat = -18f; lat <= 18f; lat += 4f)
            {
                float zHint = egoOrigin.Z;
                for (float forward = 0f; forward <= 70f; forward += 5f)
                {
                    Vector3 candidate = new Vector3(
                        egoOrigin.X + egoForward.X * forward + egoLeft.X * lat,
                        egoOrigin.Y + egoForward.Y * forward + egoLeft.Y * lat,
                        zHint);

                    Vector3 p;
                    float flow;
                    float conf;
                    bool opposing;
                    bool road = TryProjectToSurface(
                        candidate, egoHeading, zHint,
                        out p, out flow, out conf, out opposing);
                    if (road) zHint = p.Z;
                    else p = candidate;

                    bool occupied = false;
                    for (int i = 0; i < actors.Count; i++)
                    {
                        if (ActorClearance(actors[i], p, egoHeading, 0f) <= 0.2f)
                        {
                            occupied = true;
                            break;
                        }
                    }

                    DebugCells.Add(new DebugCell
                    {
                        Position = p,
                        OnRoad = road,
                        Occupied = occupied,
                        FlowCost = flow,
                        SurfaceCost = road ? (1f - conf) * 2f : 14f,
                    });
                }
            }
        }

        private static Vector3 DirectionAt(IList<Vector3> path, int i)
        {
            Vector3 d;
            if (i <= 0)
                d = new Vector3(path[1].X - path[0].X, path[1].Y - path[0].Y, path[1].Z - path[0].Z);
            else if (i >= path.Count - 1)
                d = new Vector3(path[i].X - path[i - 1].X, path[i].Y - path[i - 1].Y, path[i].Z - path[i - 1].Z);
            else
                d = new Vector3(path[i + 1].X - path[i - 1].X, path[i + 1].Y - path[i - 1].Y, path[i + 1].Z - path[i - 1].Z);

            Vector3 flat = new Vector3(d.X, d.Y, 0f);
            return RaceMath.FlatLength(flat) > 0.2f
                ? RaceMath.FlatNormalize(flat)
                : new Vector3(0f, 1f, 0f);
        }

        private static float GradeAt(IList<Vector3> path, int i)
        {
            if (path == null || path.Count < 2) return 0f;
            int a = Math.Max(0, i - 1);
            int b = Math.Min(path.Count - 1, i + 1);
            Vector3 d = new Vector3(
                path[b].X - path[a].X,
                path[b].Y - path[a].Y,
                path[b].Z - path[a].Z);
            float flat = (float)Math.Sqrt(d.X * d.X + d.Y * d.Y);
            if (flat < 0.5f) return 0f;
            return RaceMath.Clamp(d.Z / flat, -0.45f, 0.45f);
        }

        private static float AxisHeadingError(float a, float b)
        {
            float d = Math.Abs(RaceMath.HeadingDiffDeg(a, b));
            return Math.Min(d, Math.Abs(180f - d));
        }

        private static float AlignHeadingAxis(float h, float reference)
        {
            if (Math.Abs(RaceMath.HeadingDiffDeg(h, reference)) <= 90f) return h;
            float x = h + 180f;
            while (x >= 360f) x -= 360f;
            while (x < 0f) x += 360f;
            return x;
        }
    }
}
