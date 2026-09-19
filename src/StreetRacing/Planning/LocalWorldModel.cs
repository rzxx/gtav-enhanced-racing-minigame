using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;

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
        }

        public readonly List<RoadSupport> Road = new List<RoadSupport>();
        public readonly List<DebugCell> DebugCells = new List<DebugCell>();
        public string Detail { get; private set; } = "";

        private readonly List<TrackedActor> actors = new List<TrackedActor>();
        private Perception perception;
        private Vector3 egoOrigin;
        private Vector3 egoForward;
        private Vector3 egoLeft;
        private float egoHalfLength = 2.3f;
        private float egoHalfWidth = 1.0f;

        public void Reset()
        {
            Road.Clear();
            DebugCells.Clear();
            actors.Clear();
            perception = null;
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
            Reset();
            this.perception = perception;
            egoOrigin = egoPos;
            egoForward = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(egoHeading));
            egoLeft = new Vector3(-egoForward.Y, egoForward.X, 0f);
            this.egoHalfLength = RaceMath.Clamp(egoHalfLength, 1.5f, 4.5f);
            this.egoHalfWidth = RaceMath.Clamp(egoHalfWidth, 0.75f, 1.8f);

            AddReferenceSupports(reference);
            AddNearbyNodeSupports(egoPos, egoHeading);
            MergeRedundantSupports();

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
            Detail = $"roadSupports={Road.Count};actors={actors.Count};debugCells={DebugCells.Count}";
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
            var q = QuerySurface(candidate, headingDeg, previousZ, true);
            if (!q.Found)
            {
                projected = candidate;
                flowCost = 0f;
                confidence = 0f;
                opposingSide = false;
                return false;
            }

            projected = new Vector3(candidate.X, candidate.Y, q.SurfaceZ);
            flowCost = q.FlowCost;
            confidence = q.Confidence;
            opposingSide = q.OpposingSide;
            return true;
        }

        public PoseCost EvaluatePose(Vector3 pos, float headingDeg, float timeS)
        {
            var q = QuerySurface(pos, headingDeg, pos.Z, false);
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

                float hardMargin = a.Kind == ActorKind.Ped ? 0.9f : 0.15f;
                if (clear <= hardMargin)
                {
                    result.HardCollision = true;
                    result.ActorCost += a.Kind == ActorKind.Ped ? 5000f : 1500f;
                }
                else
                {
                    float caution = a.Kind == ActorKind.Ped ? 4.5f : 2.8f;
                    if (clear < caution)
                    {
                        float x = caution - clear;
                        result.ActorCost += x * x * (a.Kind == ActorKind.Ped ? 16f : 5f);
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

            float actorHeading = a.HeadingDeg;
            if (RaceMath.FlatLength(a.Velocity) > 1.2f)
                actorHeading = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(a.Velocity));

            Vector3 af = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(actorHeading));
            Vector3 al = new Vector3(-af.Y, af.X, 0f);
            Vector3 ef = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(egoHeadingDeg));
            Vector3 el = new Vector3(-ef.Y, ef.X, 0f);

            float actorHL = a.HalfLengthM > 0.2f ? a.HalfLengthM : 2.3f;
            float actorHW = a.HalfWidthM > 0.2f ? a.HalfWidthM : 1.0f;
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
                    if (RaceMath.FlatDistance(s.Center, m.Center) > 5.5f) continue;
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

        private SurfaceQuery QuerySurface(
            Vector3 p,
            float headingDeg,
            float zHint,
            bool projectionMode)
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
            };
            float bestScore = float.MaxValue;

            for (int i = 0; i < Road.Count; i++)
            {
                var s = Road[i];
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

                float flowCost;
                bool opposing;
                EvaluateFlow(s, lat, headingDeg, out flowCost, out opposing);

                bestScore = score;
                best = new SurfaceQuery
                {
                    Found = inside,
                    Index = i,
                    SignedOutside = outside,
                    Confidence = s.Confidence,
                    SurfaceZ = surfaceZ,
                    Grade = s.Grade,
                    FlowCost = flowCost,
                    OpposingSide = opposing,
                };
            }
            return best;
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

            for (float forward = 0f; forward <= 70f; forward += 5f)
            {
                for (float lat = -18f; lat <= 18f; lat += 4f)
                {
                    Vector3 p = new Vector3(
                        egoOrigin.X + egoForward.X * forward + egoLeft.X * lat,
                        egoOrigin.Y + egoForward.Y * forward + egoLeft.Y * lat,
                        egoOrigin.Z);

                    Vector3 projected;
                    float flow;
                    float conf;
                    bool opposing;
                    bool road = TryProjectToSurface(
                        p, egoHeading, egoOrigin.Z,
                        out projected, out flow, out conf, out opposing);
                    if (road) p = projected;

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
