using System;
using System.Collections.Generic;
using System.Numerics;

namespace NormalSmith.DataStructure
{
    #region BVH Data Structures

    /// <summary>
    /// Represents a triangle with associated UV coordinates.
    /// This type is used for occlusion testing and BVH construction.
    /// </summary>
    public class TriangleData
    {
        public Vector3 v0, v1, v2;
        public Vector2 uv0, uv1, uv2;

        /// <summary>
        /// Initializes a new instance of the TriangleData class.
        /// </summary>
        public TriangleData(Vector3 a, Vector3 b, Vector3 c, Vector2 uvA, Vector2 uvB, Vector2 uvC)
        {
            v0 = a;
            v1 = b;
            v2 = c;
            uv0 = uvA;
            uv1 = uvB;
            uv2 = uvC;
        }

        /// <summary>
        /// Returns the centroid of the triangle.
        /// </summary>
        public Vector3 Centroid() => (v0 + v1 + v2) / 3.0f;
    }

    /// <summary>
    /// Represents a node in a Bounding Volume Hierarchy (BVH) used for accelerating ray intersection tests.
    /// </summary>
    public class BVHNode
    {
        public PublicBoundingBox Box;
        public BVHNode Left;
        public BVHNode Right;
        public List<TriangleData> Triangles;
        private const int LeafSize = 4;
        public bool IsLeaf => Triangles != null;

        /// <summary>
        /// Constructs a BVH node from a list of triangles.
        /// If the number of triangles is less than or equal to the leaf size, this node is a leaf.
        /// Otherwise, the triangles are sorted along the largest axis and split into two child nodes.
        /// </summary>
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
        /// Determines whether a ray (with a given origin and direction) intersects any triangle 
        /// in the BVH, taking into account an alpha mask via the sampleAlpha function.
        /// </summary>
        public bool IntersectRayWithAlpha(Vector3 origin, Vector3 direction, float maxDistance, Func<Vector2, float> sampleAlpha)
        {
            if (!Box.IntersectRay(origin, direction, maxDistance))
                return false;

            if (IsLeaf)
            {
                foreach (var tri in Triangles)
                {
                    if (RayIntersectsHairCard(origin, direction, tri, out float t, sampleAlpha) && t < maxDistance)
                        return true;
                }
                return false;
            }
            else
            {
                return Left.IntersectRayWithAlpha(origin, direction, maxDistance, sampleAlpha) ||
                       Right.IntersectRayWithAlpha(origin, direction, maxDistance, sampleAlpha);
            }
        }

        /// <summary>
        /// Checks whether a ray intersects a triangle (considering an alpha mask).
        /// </summary>
        public static bool RayIntersectsHairCard(Vector3 origin, Vector3 direction, TriangleData tri, out float t, Func<Vector2, float> sampleAlpha)
        {
            if (!RayIntersectsTriangle(origin, direction, tri.v0, tri.v1, tri.v2, out t))
                return false;

            float area = ComputeTriangleArea(tri.v0, tri.v1, tri.v2);
            if (area <= 0)
                return false;

            ComputeBarycentrics(tri.v0, tri.v1, tri.v2, origin, direction, t, area, out float u, out float v, out float w);
            Vector2 hitUV = u * tri.uv0 + v * tri.uv1 + w * tri.uv2;
            float alpha = sampleAlpha(hitUV);
            return alpha >= 0.5f;
        }

        /// <summary>
        /// Computes the area of a triangle defined by three vertices.
        /// </summary>
        private static float ComputeTriangleArea(Vector3 a, Vector3 b, Vector3 c)
        {
            return Vector3.Cross(b - a, c - a).Length() * 0.5f;
        }

        /// <summary>
        /// Computes barycentric coordinates for the point where the ray intersects the triangle.
        /// </summary>
        private static void ComputeBarycentrics(Vector3 a, Vector3 b, Vector3 c, Vector3 origin, Vector3 direction, float t, float area, out float u, out float v, out float w)
        {
            Vector3 p = origin + t * direction;
            float areaPBC = ComputeTriangleArea(p, b, c);
            float areaPCA = ComputeTriangleArea(p, c, a);
            u = areaPBC / area;
            v = areaPCA / area;
            w = 1 - u - v;
        }

        /// <summary>
        /// Determines if a ray intersects a triangle using the Möller–Trumbore algorithm.
        /// </summary>
        private static bool RayIntersectsTriangle(Vector3 origin, Vector3 direction, Vector3 v0, Vector3 v1, Vector3 v2, out float t)
        {
            const float epsilon = 1e-6f;
            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            Vector3 h = Vector3.Cross(direction, edge2);
            float a = Vector3.Dot(edge1, h);
            if (MathF.Abs(a) < epsilon)
            {
                t = 0;
                return false;
            }
            float f = 1.0f / a;
            Vector3 s = origin - v0;
            float u = f * Vector3.Dot(s, h);
            if (u < 0.0f || u > 1.0f)
            {
                t = 0;
                return false;
            }
            Vector3 q = Vector3.Cross(s, edge1);
            float v = f * Vector3.Dot(direction, q);
            if (v < 0.0f || u + v > 1.0f)
            {
                t = 0;
                return false;
            }
            t = f * Vector3.Dot(edge2, q);
            return t > epsilon;
        }
    }

    /// <summary>
    /// Represents a public bounding box used by the BVH.
    /// This type is used to quickly exclude rays that do not intersect the spatial region.
    /// </summary>
    public struct PublicBoundingBox
    {
        public Vector3 Min;
        public Vector3 Max;

        /// <summary>
        /// Creates a bounding box that encloses all the given triangles.
        /// </summary>
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
        /// Determines whether a ray (with a given origin, direction, and maximum distance) intersects this bounding box.
        /// </summary>
        public bool IntersectRay(Vector3 origin, Vector3 direction, float maxDistance)
        {
            float tmin = 0;
            float tmax = maxDistance;
            for (int i = 0; i < 3; i++)
            {
                float invD = 1.0f / direction[i];
                float t0 = (Min[i] - origin[i]) * invD;
                float t1 = (Max[i] - origin[i]) * invD;
                if (invD < 0)
                {
                    float temp = t0;
                    t0 = t1;
                    t1 = temp;
                }
                tmin = MathF.Max(tmin, t0);
                tmax = MathF.Min(tmax, t1);
                if (tmax < tmin)
                    return false;
            }
            return true;
        }
    }

    #endregion
}
