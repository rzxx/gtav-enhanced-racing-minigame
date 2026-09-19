using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;

namespace StreetRacing
{
    /// Local 2D world representation used by SpatialPlannerV1.
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
            public string Source;
        }

        internal struct DebugCell
        {
            public Vector3 Position;
            public float SurfaceCost;
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
            float egoHalfWidth)
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

            BuildDebugGrid();
            Detail = $"roadSupports={Road.Count};actors={actors.Count};debugCells={DebugCells.Count}";
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
            };

            float signedOutside;
            float roadConf;
            result.OnRoad = SurfaceAt(pos, out signedOutside, out roadConf);
            result.RoadConfidence = roadConf;
            if (result.OnRoad)
            {
                result.SurfaceCost = (1f - roadConf) * 2.0f;
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

        public float ActorClearance(TrackedActor a, Vector3 egoPos, float egoHeadingDeg, float timeS)
        {
            Vector3 ap;
            try
            {
                ap = perception != null
                    ? perception.Predict(a, RaceMath.Clamp(timeS, 0f, 5f))
                    : new Vector3(a.Position.X + a.Velocity.X * timeS, a.Position.Y + a.Velocity.Y * timeS, a.Position.Z);
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

        private bool SurfaceAt(Vector3 p, out float signedOutside, out float confidence)
        {
            signedOutside = 999f;
            confidence = 0f;
            bool insideAny = false;

            for (int i = 0; i < Road.Count; i++)
            {
                var s = Road[i];
                if (Math.Abs(p.Z - s.Center.Z) > 7.5f) continue;
                Vector3 f = RaceMath.VectorFromHeading(s.HeadingDeg);
                f = RaceMath.FlatNormalize(f);
                Vector3 l = new Vector3(-f.Y, f.X, 0f);
                Vector3 rel = new Vector3(p.X - s.Center.X, p.Y - s.Center.Y, 0f);
                float along = Math.Abs(RaceMath.FlatDot(rel, f));
                float lat = RaceMath.FlatDot(rel, l);

                float outsideAlong = along - s.HalfLengthM;
                float outsideLat = lat >= 0f ? lat - s.LeftM : -lat - s.RightM;
                float outside = Math.Max(outsideAlong, outsideLat);

                if (outside <= 0f)
                {
                    insideAny = true;
                    // signedOutside near zero means near an estimated boundary;
                    // more negative means safely inside.
                    if (outside < signedOutside) signedOutside = outside;
                    if (s.Confidence > confidence) confidence = s.Confidence;
                }
                else if (!insideAny && outside < signedOutside)
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
                    bool road = SurfaceAt(p, out outDist, out conf);
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
                        SurfaceCost = road ? (1f - conf) * 2f : 14f + Math.Max(0f, outDist),
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
