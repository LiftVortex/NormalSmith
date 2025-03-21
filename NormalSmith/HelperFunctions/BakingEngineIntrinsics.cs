using System;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Numerics;
using NormalSmith.DataStructure;  // For Vector3

namespace NormalSmith.HelperFunctions
{
    public static class BakingEngineIntrinsics
    {
        /// <summary>
        /// Generates eight sample directions in parallel using AVX2 intrinsics.
        /// This is a simplified version; adjust the math to match your actual sampling logic.
        /// </summary>
        /// <param name="uVec">Vector of 8 u values.</param>
        /// <param name="vVec">Vector of 8 v values.</param>
        /// <param name="nX">Broadcast of surface normal X.</param>
        /// <param name="nY">Broadcast of surface normal Y.</param>
        /// <param name="nZ">Broadcast of surface normal Z.</param>
        /// <param name="tX">Broadcast of surface tangent X.</param>
        /// <param name="tY">Broadcast of surface tangent Y.</param>
        /// <param name="tZ">Broadcast of surface tangent Z.</param>
        /// <param name="outX">Output vector for sample direction X components.</param>
        /// <param name="outY">Output vector for sample direction Y components.</param>
        /// <param name="outZ">Output vector for sample direction Z components.</param>
        public static void GenerateSampleDirectionsVectorized(
            Vector256<float> uVec, Vector256<float> vVec,
            Vector256<float> nX, Vector256<float> nY, Vector256<float> nZ,
            Vector256<float> tX, Vector256<float> tY, Vector256<float> tZ,
            out Vector256<float> outX, out Vector256<float> outY, out Vector256<float> outZ)
        {
            // Example: sampleDir = normalize( n + (2*u - 1)*t )
            Vector256<float> two = Vector256.Create(2.0f);
            Vector256<float> one = Vector256.Create(1.0f);
            Vector256<float> factor = Avx.Multiply(uVec, two);
            factor = Avx.Subtract(factor, one);

            // Multiply tangent by factor: t * (2*u - 1)
            Vector256<float> tFactorX = Avx.Multiply(tX, factor);
            Vector256<float> tFactorY = Avx.Multiply(tY, factor);
            Vector256<float> tFactorZ = Avx.Multiply(tZ, factor);

            // Add the normal: sampleDir = n + tFactor
            Vector256<float> dirX = Avx.Add(nX, tFactorX);
            Vector256<float> dirY = Avx.Add(nY, tFactorY);
            Vector256<float> dirZ = Avx.Add(nZ, tFactorZ);

            // Compute squared length: dirX^2 + dirY^2 + dirZ^2
            Vector256<float> sqX = Avx.Multiply(dirX, dirX);
            Vector256<float> sqY = Avx.Multiply(dirY, dirY);
            Vector256<float> sqZ = Avx.Multiply(dirZ, dirZ);
            Vector256<float> sumSq = Avx.Add(Avx.Add(sqX, sqY), sqZ);

            // Compute reciprocal square root (approximate normalization)
            Vector256<float> invLen = Avx.ReciprocalSqrt(sumSq);

            // Multiply components by the reciprocal length to normalize
            outX = Avx.Multiply(dirX, invLen);
            outY = Avx.Multiply(dirY, invLen);
            outZ = Avx.Multiply(dirZ, invLen);
        }

        /// <summary>
        /// Processes eight samples at once using AVX2 intrinsics.
        /// This method loads 8 u and 8 v values from the randomFloats array,
        /// computes 8 sample directions in parallel, and then processes each sample
        /// with the scalar BVH intersection test.
        /// </summary>
        /// <param name="randomFloats">Array containing at least 16 floats (u and v values).</param>
        /// <param name="randIndex">Starting index into randomFloats.</param>
        /// <param name="interpNormal">Interpolated surface normal.</param>
        /// <param name="interpTangent">Interpolated surface tangent.</param>
        /// <param name="origin">Ray origin.</param>
        /// <param name="mDistance">Maximum ray distance.</param>
        /// <param name="bvhRoot">BVH for intersection testing.</param>
        /// <param name="sampleAlpha">Delegate for sampling alpha at a given UV.</param>
        /// <param name="unoccluded">Reference counter for unoccluded rays.</param>
        /// <param name="sumDir">Reference accumulator for sample directions.</param>
        public static unsafe void ProcessEightSamples(
            float[] randomFloats, int randIndex,
            Vector3 interpNormal, Vector3 interpTangent,
            Vector3 origin,
            float mDistance,
            BVHNode bvhRoot,
            Func<Vector2, float> sampleAlpha,
            ref int unoccluded,
            ref Vector3 sumDir)
        {
            if (!Avx2.IsSupported)
                throw new PlatformNotSupportedException("AVX2 is not supported on this CPU.");

            Vector256<float> uVec;
            Vector256<float> vVec;
            // Load 8 u values and 8 v values from the randomFloats array
            fixed (float* p = &randomFloats[randIndex])
            {
                uVec = Avx.LoadVector256(p);         // Loads randomFloats[randIndex] to [randIndex+7]
                vVec = Avx.LoadVector256(p + 8);       // Loads randomFloats[randIndex+8] to [randIndex+15]
            }

            // Broadcast interpNormal and interpTangent values into vector registers
            Vector256<float> nX = Vector256.Create(interpNormal.X);
            Vector256<float> nY = Vector256.Create(interpNormal.Y);
            Vector256<float> nZ = Vector256.Create(interpNormal.Z);
            Vector256<float> tX = Vector256.Create(interpTangent.X);
            Vector256<float> tY = Vector256.Create(interpTangent.Y);
            Vector256<float> tZ = Vector256.Create(interpTangent.Z);

            // Compute eight sample directions using the vectorized function
            GenerateSampleDirectionsVectorized(uVec, vVec, nX, nY, nZ, tX, tY, tZ,
                out Vector256<float> sampleDirX,
                out Vector256<float> sampleDirY,
                out Vector256<float> sampleDirZ);

            // Prepare arrays to store the vectorized results for scalar processing
            float[] sampleDirXArr = new float[8];
            float[] sampleDirYArr = new float[8];
            float[] sampleDirZArr = new float[8];

            // Store the vectors into the arrays using fixed pointers
            fixed (float* destX = sampleDirXArr)
            {
                Avx.Store(destX, sampleDirX);
            }
            fixed (float* destY = sampleDirYArr)
            {
                Avx.Store(destY, sampleDirY);
            }
            fixed (float* destZ = sampleDirZArr)
            {
                Avx.Store(destZ, sampleDirZ);
            }

            // Process each of the 8 samples individually for BVH intersection
            for (int i = 0; i < 8; i++)
            {
                Vector3 sampleDir = new Vector3(sampleDirXArr[i], sampleDirYArr[i], sampleDirZArr[i]);
                float? tHit = bvhRoot.IntersectRayWithAlphaNearest(origin, sampleDir, mDistance, sampleAlpha);
                if (!(tHit.HasValue && tHit.Value < mDistance))
                {
                    unoccluded++;
                    sumDir += sampleDir;
                }
            }
        }
    }
}
