using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;

namespace StreetRacing
{
    /// Local 2.5D world representation used by SpatialPlannerV2.
    ///
    /// Unlike LocalPlannerV2, this model does not ask "how far left/right from
    /// the GPS rail may I go?". It represents a union of local road supports
    /// in world XY, plus time-dependent oriented actor occupancy.
    internal sealed class LocalWorldModel
    {
        internal struct RoadSupport
        {
            public Vector3 Center;
            public float HeadingDeg;
            public float HalfLengthM;
            public float LeftM;
            public float RightM;
            public float Confidence;
            public int Lanes;
            public float Grade;          // dz / horizontal meter along HeadingDeg
            public bool RouteAligned;   // reference support: heading aligned with route travel
            public string Source;
        }

        internal struct DebugCell
        {
            public Vector3 Position;
            public float SurfaceCost;
            public float DirectionCost;
            public bool OnRoad;
            public bool Occupied;
        }

        internal struct PoseCost
        {
            public bool HardCollision;
            public float SurfaceCost;
            public float ActorCost;
            public float ClearanceM;
            public int BlockingHandle;
            public bool OnRoad;
            public float RoadConfidence;
            public float DirectionCost;
            public float SurfaceZ;
            public float SurfaceHeadingDeg;
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
            this.egoOrigin = egoPos;
            this.egoForward = RaceMath.VectorFromHeading(egoHeading);
            this.egoForward = RaceMath.FlatNormalize(this.egoForward);
            this.egoLeft = new Vector3(-this.egoForward.Y, this.egoForward.X, 0f);
            this.egoHalfLength = RaceMath.Clamp(egoHalfLength, 1.5f, 4.5f);
            this.egoHalfWidth = RaceMath.Clamp(egoHalfWidth, 0.75f, 1.8f);

            AddReferenceSupports(reference);
            AddNearbyNodeSupports(egoPos, egoHeading);
            MergeRedundantSupports();
            InferGrades();

            if (perception != null)
            {
                for (int i = 0; i < perception.Actors.Count; i++)
                {
                    var a = perception.Actors[i];
                    if (!a.Valid) continue;
                    if (a.Dist > 110f) continue;
                    actors.Add(a);
                }
            }

            if (buildDebugGrid) BuildDebugGrid();
            float maxAbsGrade = 0f;
            int aligned = 0;
            for (int i = 0; i < Road.Count; i++)
            {
                float g = Math.Abs(Road[i].Grade);
                if (g > maxAbsGrade) maxAbsGrade = g;
                if (Road[i].RouteAligned) aligned++;
            }
            Detail = $"roadSupports={Road.Count};aligned={aligned};actors={actors.Count};"
                + $"maxGrade={maxAbsGrade:F2};debugCells={DebugCells.Count}";
        }

        public PoseCost EvaluatePose(Vector3 pos, float headingDeg, float timeS)
        {
            var result = new PoseCost
            {
                HardCollision = false,
                SurfaceCost = 0f,
                ActorCost = 0f,
                ClearanceM = 999f,
                BlockingHandle = -1,
                OnRoad = false,
                RoadConfidence = 0f,
                DirectionCost = 0f,
                SurfaceZ = pos.Z,
                SurfaceHeadingDeg = headingDeg,
            };

            float signedOutside;
            float roadConf;
            float directionCost;
            float surfaceZ;
            float surfaceHeading;
            result.OnRoad = SurfaceAt(pos, headingDeg, out signedOutside, out roadConf,
                out directionCost, out surfaceZ, out surfaceHeading);
            result.RoadConfidence = roadConf;
            result.DirectionCost = directionCost;
            result.SurfaceZ = surfaceZ;
            result.SurfaceHeadingDeg = surfaceHeading;
            if (result.OnRoad)
            {
                result.SurfaceCost = (1f - roadConf) * 2.0f + directionCost;
                // Small preference for not grazing inferred road edges.
                if (signedOutside > -0.7f)
                    result.SurfaceCost += (signedOutside + 0.7f) * 0.9f;
            }
            else
            {
                // Unknown/non-road is expensive but not a wall. This is the
                // hook for future sidewalk/grass classification.
                result.SurfaceCost = 14.0f + Math.Min(24f, Math.Max(0f, signedOutside) * 2.2f);
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

        public bool TryGetActorRelative(
            int handle, Vector3 fromPos, float headingDeg, float timeS,
            out float along, out float lateral, out TrackedActor actor)
        {
            along = 999f;
            lateral = 999f;
            actor = new TrackedActor();
            if (!TryGetActor(handle, out actor)) return false;
            Vector3 ap;
            try
            {
                ap = perception != null
                    ? perception.Predict(actor, RaceMath.Clamp(timeS, 0f, 5f))
                    : new Vector3(actor.Position.X + actor.Velocity.X * timeS,
                        actor.Position.Y + actor.Velocity.Y * timeS,
                        actor.Position.Z + actor.Velocity.Z * timeS);
            }
            catch { ap = actor.Position; }

            Vector3 f = RaceMath.VectorFromHeading(headingDeg);
            f = RaceMath.FlatNormalize(f);
            Vector3 l = new Vector3(-f.Y, f.X, 0f);
            Vector3 rel = new Vector3(ap.X - fromPos.X, ap.Y - fromPos.Y, 0f);
            along = RaceMath.FlatDot(rel, f);
            lateral = RaceMath.FlatDot(rel, l);
            return true;
        }

        public bool TryProjectToSurface(
            Vector3 pos, float previousZ, float headingDeg,
            out Vector3 projected, out float confidence, out float directionCost)
        {
            projected = pos;
            confidence = 0f;
            directionCost = 0f;
            int best = -1;
            float bestScore = float.MaxValue;
            float bestZ = previousZ;

            for (int i = 0; i < Road.Count; i++)
            {
                var s = Road[i];
                Vector3 f = RaceMath.VectorFromHeading(s.HeadingDeg);
                f = RaceMath.FlatNormalize(f);
                Vector3 l = new Vector3(-f.Y, f.X, 0f);
                Vector3 rel = new Vector3(pos.X - s.Center.X, pos.Y - s.Center.Y, 0f);
                float alongSigned = RaceMath.FlatDot(rel, f);
                float along = Math.Abs(alongSigned);
                float lat = RaceMath.FlatDot(rel, l);
                if (along > s.HalfLengthM + 1.0f) continue;
                if (lat > s.LeftM + 0.6f || lat < -s.RightM - 0.6f) continue;

                float z = s.Center.Z + alongSigned * s.Grade;
                float dz = Math.Abs(z - previousZ);
                // Step-to-step surface continuity prevents snapping from a
                // mountain road to an overpass/underpass sharing the same XY.
                if (dz > 3.25f) continue;

                float axis = AxisHeadingError(headingDeg, s.HeadingDeg);
                float score = dz * 2.4f + axis * 0.025f - s.Confidence * 0.8f;
                if (score >= bestScore) continue;
                bestScore = score;
                best = i;
                bestZ = z;
            }

            if (best < 0) return false;
            var hit = Road[best];
            projected = new Vector3(pos.X, pos.Y, bestZ);
            confidence = hit.Confidence;
            directionCost = DirectionPenalty(hit, projected, headingDeg);
            return true;
        }

        public float ActorClearance(TrackedActor a, Vector3 egoPos, float egoHeadingDeg, float timeS)
        {
            Vector3 ap;
            try
            {
                ap = perception != null
                    ? perception.Predict(a, RaceMath.Clamp(timeS, 0f, 5f))
                    : new Vector3(a.Position.X + a.Velocity.X * timeS,
                        a.Position.Y + a.Velocity.Y * timeS,
                        a.Position.Z + a.Velocity.Z * timeS);
            }
            catch { ap = a.Position; }

            float actorHeading = a.HeadingDeg;
            if (RaceMath.FlatLength(a.Velocity) > 1.2f)
                actorHeading = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(a.Velocity));

            if (Math.Abs(ap.Z - egoPos.Z) > 4.5f)
                return 999f; // stacked road / bridge traffic

            Vector3 af = RaceMath.VectorFromHeading(actorHeading);
            af = RaceMath.FlatNormalize(af);
            Vector3 al = new Vector3(-af.Y, af.X, 0f);
            Vector3 ef = RaceMath.VectorFromHeading(egoHeadingDeg);
            ef = RaceMath.FlatNormalize(ef);
            Vector3 el = new Vector3(-ef.Y, ef.X, 0f);

            float actorHL = a.HalfLengthM > 0.2f ? a.HalfLengthM : 2.3f;
            float actorHW = a.HalfWidthM > 0.2f ? a.HalfWidthM : 1.0f;

            // Minkowski inflation of the actor box by the ego box projected
            // onto actor axes. The ego center then becomes a point-vs-OBB test.
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
                return Math.Max(x, y); // negative = overlap depth-ish

            float ox = Math.Max(0f, x);
            float oy = Math.Max(0f, y);
            return (float)Math.Sqrt(ox * ox + oy * oy);
        }

        private void AddReferenceSupports(DrivingReference.Result reference)
        {
            if (reference == null || reference.Path == null || reference.Path.Count < 2) return;
            for (int i = 0; i < reference.Path.Count; i++)
            {
                Vector3 center = i < reference.RoadCenter.Count
                    ? reference.RoadCenter[i]
                    : reference.Path[i];
                float heading = i < reference.RoadHeadingDeg.Count
                    ? reference.RoadHeadingDeg[i]
                    : RaceMath.HeadingFromVector(DirectionAt(reference.Path, i));
                float left = i < reference.LeftRoadM.Count ? reference.LeftRoadM[i] : 3.5f;
                float right = i < reference.RightRoadM.Count ? reference.RightRoadM[i] : 3.5f;
                float conf = i < reference.RoadConfidence.Count ? reference.RoadConfidence[i] : 0.35f;
                int lanes = i < reference.RoadLaneCount.Count ? reference.RoadLaneCount[i] : 2;

                float halfLen = 8f;
                if (i + 1 < reference.Path.Count)
                    halfLen = Math.Max(halfLen, RaceMath.FlatDistance(reference.Path[i], reference.Path[i + 1]) * 0.8f + 5f);

                Road.Add(new RoadSupport
                {
                    Center = center,
                    HeadingDeg = heading,
                    HalfLengthM = halfLen,
                    LeftM = RaceMath.Clamp(left, 1.7f, 16f),
                    RightM = RaceMath.Clamp(right, 1.7f, 16f),
                    Confidence = RaceMath.Clamp(conf, 0f, 1f),
                    Lanes = Math.Max(1, lanes),
                    Grade = 0f,
                    RouteAligned = true,
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
                    if (d > 95f) continue;
                    if (Math.Abs(p.Z - egoPos.Z) > 9f) continue;

                    lanes = Math.Max(1, Math.Min(8, lanes));
                    float half = RaceMath.Clamp(lanes * 3.25f * 0.5f + 0.55f, 2.0f, 14f);
                    float conf = 0.52f - RaceMath.Clamp(d / 95f, 0f, 1f) * 0.17f;
                    float axis = AxisHeadingError(h, egoHeading);
                    if (axis < 40f) conf += 0.08f;

                    Road.Add(new RoadSupport
                    {
                        Center = p,
                        HeadingDeg = h,
                        HalfLengthM = 13f,
                        LeftM = half,
                        RightM = half,
                        Confidence = RaceMath.Clamp(conf, 0.25f, 0.62f),
                        Lanes = lanes,
                        Grade = 0f,
                        RouteAligned = false,
                        Source = "NearbyNode",
                    });
                }
                catch { }
            }
        }

        private void MergeRedundantSupports()
        {
            // Keep reference supports, but avoid dozens of nearly-identical node
            // rectangles on the same piece of road.
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

        private void InferGrades()
        {
            for (int i = 0; i < Road.Count; i++)
            {
                var s = Road[i];
                float bestAlong = float.MaxValue;
                float bestGrade = 0f;
                Vector3 f = RaceMath.VectorFromHeading(s.HeadingDeg);
                f = RaceMath.FlatNormalize(f);

                for (int j = 0; j < Road.Count; j++)
                {
                    if (i == j) continue;
                    var o = Road[j];
                    if (AxisHeadingError(s.HeadingDeg, o.HeadingDeg) > 24f) continue;
                    Vector3 rel = new Vector3(
                        o.Center.X - s.Center.X,
                        o.Center.Y - s.Center.Y, 0f);
                    float along = RaceMath.FlatDot(rel, f);
                    float absAlong = Math.Abs(along);
                    if (absAlong < 4f || absAlong > 28f || absAlong >= bestAlong) continue;
                    float cross = Math.Abs(RaceMath.FlatCross(f, rel));
                    if (cross > 6f) continue;
                    float grade = (o.Center.Z - s.Center.Z) / along;
                    if (Math.Abs(grade) > 0.55f) continue;
                    bestAlong = absAlong;
                    bestGrade = grade;
                }

                s.Grade = RaceMath.Clamp(bestGrade, -0.45f, 0.45f);
                Road[i] = s;
            }
        }

        private static float DirectionPenalty(RoadSupport s, Vector3 p, float headingDeg)
        {
            float axisDiff = Math.Abs(RaceMath.HeadingDiffDeg(s.HeadingDeg, headingDeg));
            float headingPenalty = axisDiff > 90f
                ? 4.0f + (axisDiff - 90f) / 45f
                : axisDiff > 45f ? (axisDiff - 45f) / 45f * 0.6f : 0f;

            // Los Santos uses right-hand traffic. For route-aligned supports,
            // the right half is the preferred same-direction carriageway.
            // The left/opposing half is risky but deliberately NOT forbidden.
            float sidePenalty = 0f;
            if (s.RouteAligned && s.Lanes >= 2)
            {
                Vector3 f = RaceMath.VectorFromHeading(s.HeadingDeg);
                f = RaceMath.FlatNormalize(f);
                Vector3 l = new Vector3(-f.Y, f.X, 0f);
                Vector3 rel = new Vector3(p.X - s.Center.X, p.Y - s.Center.Y, 0f);
                float lat = RaceMath.FlatDot(rel, l);
                float deadBand = Math.Min(1.0f, Math.Min(s.LeftM, s.RightM) * 0.18f);
                if (lat > deadBand)
                {
                    float frac = (lat - deadBand) / Math.Max(1f, s.LeftM - deadBand);
                    sidePenalty = 1.4f + RaceMath.Clamp(frac, 0f, 1f) * 2.2f;
                }
            }
            return headingPenalty + sidePenalty;
        }

        private bool SurfaceAt(
            Vector3 p, float headingDeg,
            out float signedOutside, out float confidence,
            out float directionCost, out float surfaceZ, out float surfaceHeading)
        {
            signedOutside = 999f;
            confidence = 0f;
            directionCost = 0f;
            surfaceZ = p.Z;
            surfaceHeading = headingDeg;
            bool insideAny = false;
            float bestInsideScore = float.MaxValue;

            for (int i = 0; i < Road.Count; i++)
            {
                var s = Road[i];
                Vector3 f = RaceMath.VectorFromHeading(s.HeadingDeg);
                f = RaceMath.FlatNormalize(f);
                Vector3 l = new Vector3(-f.Y, f.X, 0f);
                Vector3 rel = new Vector3(p.X - s.Center.X, p.Y - s.Center.Y, 0f);
                float alongSigned = RaceMath.FlatDot(rel, f);
                float along = Math.Abs(alongSigned);
                float lat = RaceMath.FlatDot(rel, l);

                float outsideAlong = along - s.HalfLengthM;
                float outsideLat = lat >= 0f ? lat - s.LeftM : -lat - s.RightM;
                float outside = Math.Max(outsideAlong, outsideLat);
                float z = s.Center.Z + alongSigned * s.Grade;
                float dz = Math.Abs(p.Z - z);

                if (outside <= 0f && dz <= 3.5f)
                {
                    insideAny = true;
                    float score = dz * 1.5f - s.Confidence;
                    if (score < bestInsideScore)
                    {
                        bestInsideScore = score;
                        signedOutside = outside;
                        confidence = s.Confidence;
                        directionCost = DirectionPenalty(s, p, headingDeg);
                        surfaceZ = z;
                        surfaceHeading = s.HeadingDeg;
                    }
                }
                else if (!insideAny && outside < signedOutside && dz < 8f)
                {
                    signedOutside = outside;
                    confidence = Math.Max(confidence, s.Confidence * 0.5f);
                }
            }

            if (signedOutside == 999f) signedOutside = 20f;
            return insideAny;
        }

        private void BuildDebugGrid()
        {
            DebugCells.Clear();
            // Coarse grid only; planning uses continuous evaluation.
            for (float forward = 0f; forward <= 70f; forward += 5f)
            {
                for (float lat = -18f; lat <= 18f; lat += 4f)
                {
                    Vector3 p = new Vector3(
                        egoOrigin.X + egoForward.X * forward + egoLeft.X * lat,
                        egoOrigin.Y + egoForward.Y * forward + egoLeft.Y * lat,
                        egoOrigin.Z);
                    float outDist;
                    float conf;
                    float dirCost;
                    float surfaceZ;
                    float surfaceHeading;
                    bool road = SurfaceAt(p, RaceMath.HeadingFromVector(egoForward),
                        out outDist, out conf, out dirCost, out surfaceZ, out surfaceHeading);
                    if (road) p = new Vector3(p.X, p.Y, surfaceZ);
                    bool occupied = false;
                    for (int i = 0; i < actors.Count; i++)
                    {
                        if (ActorClearance(actors[i], p, RaceMath.HeadingFromVector(egoForward), 0f) <= 0.2f)
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
                        SurfaceCost = road ? (1f - conf) * 2f + dirCost : 14f + Math.Max(0f, outDist),
                        DirectionCost = dirCost,
                    });
                }
            }
        }

        private static Vector3 DirectionAt(IList<Vector3> path, int i)
        {
            Vector3 d;
            if (i <= 0)
                d = new Vector3(path[1].X - path[0].X, path[1].Y - path[0].Y, 0f);
            else if (i >= path.Count - 1)
                d = new Vector3(path[i].X - path[i - 1].X, path[i].Y - path[i - 1].Y, 0f);
            else
                d = new Vector3(path[i + 1].X - path[i - 1].X, path[i + 1].Y - path[i - 1].Y, 0f);
            return RaceMath.FlatLength(d) > 0.2f ? RaceMath.FlatNormalize(d) : new Vector3(0f, 1f, 0f);
        }

        private static float AxisHeadingError(float a, float b)
        {
            float d = Math.Abs(RaceMath.HeadingDiffDeg(a, b));
            return Math.Min(d, Math.Abs(180f - d));
        }
    }
}
