using System;
using System.Collections.Generic;
using System.Numerics;

namespace NormalSmith.DataStructure
{
    public class TriangleData
    {
        public Vector3 v0, v1, v2;
        public Vector2 uv0, uv1, uv2;

        public TriangleData(Vector3 a, Vector3 b, Vector3 c, Vector2 uvA, Vector2 uvB, Vector2 uvC)
        {
            v0 = a;
            v1 = b;
            v2 = c;
            uv0 = uvA;
            uv1 = uvB;
            uv2 = uvC;
        }

        public Vector3 Centroid() => (v0 + v1 + v2) / 3.0f;
    }

    public class BVHNode
    {
        public PublicBoundingBox Box;
        public BVHNode Left;
        public BVHNode Right;
        public List<TriangleData> Triangles;
        private const int LeafSize = 4;
        public bool IsLeaf => Triangles != null;

        public BVHNode(List<TriangleData> triangles, int depth)
        {
            Box = PublicBoundingBox.Create(triangles);
            if (triangles.Count <= LeafSize)
            {
                Triangles = triangles;
            }
            else
            {
                Vector3 extent = Box.Max - Box.Min;
                int axis = 0;
                if (extent.Y > extent.X) axis = 1;
                if (extent.Z > extent[axis]) axis = 2;

                triangles.Sort((a, b) => a.Centroid()[axis].CompareTo(b.Centroid()[axis]));
                int mid = triangles.Count / 2;

                List<TriangleData> leftTriangles = triangles.GetRange(0, mid);
                List<TriangleData> rightTriangles = triangles.GetRange(mid, triangles.Count - mid);

                Left = new BVHNode(leftTriangles, depth + 1);
                Right = new BVHNode(rightTriangles, depth + 1);
            }
        }

        /// <summary>
        /// The fully robust approach to alpha intersection:
        /// Returns the *closest* intersection distance if any triangle is opaque
        /// (i.e. alpha >= 0.5), or null if everything is transparent or missed.
        /// </summary>
        /// <param name="origin">Ray origin in world space.</param>
        /// <param name="direction">Normalized ray direction.</param>
        /// <param name="maxDistance">Maximum allowed intersection distance.</param>
        /// <param name="sampleAlpha">Delegate to sample alpha in [0..1] for a given UV.</param>
        /// <returns>
        /// float? - The distance to the nearest opaque intersection or null if none.
        /// </returns>
        public float? IntersectRayWithAlphaNearest(
            Vector3 origin,
            Vector3 direction,
            float maxDistance,
            Func<Vector2, float> sampleAlpha)
        {
            // 1) Quick check: if bounding box is not hit, no intersection in this node.
            if (!Box.IntersectRay(origin, direction, maxDistance))
                return null;

            // 2) Leaf node: check each triangle and pick the closest that passes alpha >= 0.5
            if (IsLeaf)
            {
                float bestT = maxDistance;
                bool foundOpaqueHit = false;

                foreach (var tri in Triangles)
                {
                    if (RayIntersectsHairCard(origin, direction, tri, out float t, sampleAlpha))
                    {
                        if (t < bestT)
                        {
                            bestT = t;
                            foundOpaqueHit = true;
                        }
                    }
                }

                return foundOpaqueHit ? bestT : (float?)null;
            }
            else
            {
                // 3) Internal node: check left child, then right child, pick the nearer intersection
                float? leftHit = Left?.IntersectRayWithAlphaNearest(origin, direction, maxDistance, sampleAlpha);
                float? rightHit = Right?.IntersectRayWithAlphaNearest(origin, direction, maxDistance, sampleAlpha);

                if (leftHit == null && rightHit == null)
                    return null;
                if (leftHit == null)
                    return rightHit;
                if (rightHit == null)
                    return leftHit;

                // Both sides returned a hit; pick the smaller distance
                return Math.Min(leftHit.Value, rightHit.Value);
            }
        }

        /// <summary>
        /// Checks if a ray intersects this "hair card" triangle with alpha. 
        /// If the triangle is physically hit but alpha < 0.5, we treat it as transparent => return false.
        /// </summary>
        public static bool RayIntersectsHairCard(
            Vector3 origin,
            Vector3 direction,
            TriangleData tri,
            out float t,
            Func<Vector2, float> sampleAlpha)
        {
            t = 0f;

            // 1) Standard Möller–Trumbore intersection
            if (!RayIntersectsTriangle(origin, direction, tri.v0, tri.v1, tri.v2, out float hitT))
                return false;

            // 2) Compute barycentric coords to find the local UV
            float area = ComputeTriangleArea(tri.v0, tri.v1, tri.v2);
            if (area <= 0) return false;

            ComputeBarycentrics(tri.v0, tri.v1, tri.v2,
                                origin, direction, hitT, area,
                                out float u, out float v, out float w);

            Vector2 hitUV = u * tri.uv0 + v * tri.uv1 + w * tri.uv2;

            // 3) Sample alpha; if alpha < 0.5, treat as transparent => no intersection
            float alpha = sampleAlpha(hitUV);
            if (alpha < 0.5f)
                return false;

            // Opaque => record the hit distance
            t = hitT;
            return true;
        }

        /// <summary>
        /// Möller–Trumbore test for ray/triangle intersection. Returns true if hit, with distance in t.
        /// </summary>
        private static bool RayIntersectsTriangle(
            Vector3 origin,
            Vector3 direction,
            Vector3 v0,
            Vector3 v1,
            Vector3 v2,
            out float t)
        {
            t = 0f;

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            // Compute a local scale factor using the average edge length of the triangle
            float len1 = edge1.Length();
            float len2 = (v2 - v1).Length();
            float len3 = (v0 - v2).Length();
            float avgEdgeLength = (len1 + len2 + len3) / 3f;
            float epsilon = avgEdgeLength * 1e-6f; // Scaled epsilon based on triangle size

            Vector3 h = Vector3.Cross(direction, edge2);

            float a = Vector3.Dot(edge1, h);
            if (MathF.Abs(a) < epsilon)
                return false; // parallel or degenerate

            float f = 1.0f / a;
            Vector3 s = origin - v0;
            float u = f * Vector3.Dot(s, h);
            if (u < 0.0f || u > 1.0f)
                return false;

            Vector3 q = Vector3.Cross(s, edge1);
            float v = f * Vector3.Dot(direction, q);
            if (v < 0.0f || (u + v) > 1.0f)
                return false;

            float dist = f * Vector3.Dot(edge2, q);
            if (dist > epsilon)
            {
                t = dist;
                return true;
            }
            return false;
        }


        /// <summary>
        /// Computes the signed area of a triangle using the cross product.
        /// </summary>
        private static float ComputeTriangleArea(Vector3 a, Vector3 b, Vector3 c)
        {
            return 0.5f * Vector3.Cross(b - a, c - a).Length();
        }

        /// <summary>
        /// Computes barycentric coords for the intersection point on the triangle.
        /// </summary>
        private static void ComputeBarycentrics(
            Vector3 a, Vector3 b, Vector3 c,
            Vector3 origin, Vector3 dir, float t,
            float area,
            out float u, out float v, out float w)
        {
            Vector3 p = origin + t * dir;
            float areaPBC = ComputeTriangleArea(p, b, c);
            float areaPCA = ComputeTriangleArea(p, c, a);

            u = areaPBC / area;
            v = areaPCA / area;
            w = 1f - u - v;
        }
    }

    /// <summary>
    /// Represents a bounding box used by the BVH.
    /// </summary>
    public struct PublicBoundingBox
    {
        public Vector3 Min;
        public Vector3 Max;

        public static PublicBoundingBox Create(List<TriangleData> triangles)
        {
            Vector3 min = new Vector3(float.MaxValue);
            Vector3 max = new Vector3(float.MinValue);

            foreach (var tri in triangles)
            {
                min = Vector3.Min(min, tri.v0);
                min = Vector3.Min(min, tri.v1);
                min = Vector3.Min(min, tri.v2);

                max = Vector3.Max(max, tri.v0);
                max = Vector3.Max(max, tri.v1);
                max = Vector3.Max(max, tri.v2);
            }

            return new PublicBoundingBox { Min = min, Max = max };
        }

        /// <summary>
        /// Standard AABB vs. ray test. If it fails, there's no intersection in this box.
        /// </summary>
        public bool IntersectRay(Vector3 origin, Vector3 direction, float maxDistance)
        {
            float tmin = 0f;
            float tmax = maxDistance;

            for (int i = 0; i < 3; i++)
            {
                float invD = 1f / direction[i];
                float t0 = (Min[i] - origin[i]) * invD;
                float t1 = (Max[i] - origin[i]) * invD;

                if (invD < 0f)
                {
                    float temp = t0;
                    t0 = t1;
                    t1 = temp;
                }

                tmin = MathF.Max(tmin, t0);
                tmax = MathF.Min(tmax, t1);
                if (tmax < tmin) return false;
            }

            return true;
        }
    }
}
