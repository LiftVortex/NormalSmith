using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Interop;
using System.Windows.Media.Imaging;
using Assimp;
using System.Drawing;

// Alias to resolve ambiguity between Assimp and System.Numerics:
using NumericsMatrix = System.Numerics.Matrix4x4;
using NormalSmith.DataStructure;
using NormalSmith.HelperFunctions;
using System.Windows.Forms;
using System.Windows.Media;

namespace NormalSmith.Engine
{
    public class BakeResult
    {
        public Bitmap PreviewBmp { get; set; }
        public Bitmap BentMap { get; set; }
        public Bitmap OccMap { get; set; }
    }


    /// <summary>
    /// Provides functionality for baking bent normal maps and occlusion maps from a 3D model.
    /// </summary>
    public static class BakingEngine
    {
        #region Public Properties

        /// <summary>
        /// Stores the final bent normal map, if one is generated.
        /// </summary>
        public static Bitmap FinalBentMap { get; private set; }

        /// <summary>
        /// Stores the final occlusion map, if one is generated.
        /// </summary>
        public static Bitmap FinalOccMap { get; private set; }

        #endregion

        #region Thread-Local Random

        /// <summary>
        /// A thread-local random number generator used during parallel baking to avoid contention.
        /// </summary>
        public static readonly ThreadLocal<Random> threadRandom =
            new ThreadLocal<Random>(() => new Random(Guid.NewGuid().GetHashCode()));

        #endregion

        #region Main Baking Method

        /// <summary>
        /// Asynchronously bakes a bent normal map and/or an occlusion map based on the provided parameters.
        /// Operates in parallel, building a BVH for efficient ray intersection and sampling alpha where applicable.
        /// </summary>
        /// <param name="modelPath">The path to the model file.</param>
        /// <param name="width">The width of the output texture.</param>
        /// <param name="height">The height of the output texture.</param>
        /// <param name="useTangentSpace">True if the resulting bent normals should be expressed in tangent space.</param>
        /// <param name="useCosineDistribution">True for cosine-weighted sampling, false for uniform hemisphere.</param>
        /// <param name="generateBentNormalMap">Whether to generate a bent normal map.</param>
        /// <param name="generateOcclusionMap">Whether to generate an occlusion map.</param>
        /// <param name="swizzle">A Vector3 controlling channel flips for the final normal output.</param>
        /// <param name="progress">A progress reporter to update baking progress.</param>
        /// <param name="token">A cancellation token to allow the operation to be canceled.</param>
        /// <param name="clampOcclusion">True to clamp occlusion to a threshold.</param>
        /// <param name="bentUVIndex">Which UV channel is used for bent normal map coordinates.</param>
        /// <param name="alphaUVIndex">Which UV channel is used for alpha sampling.</param>
        /// <param name="selectedMeshIndex">Which mesh from the scene should be baked.</param>
        /// <param name="scene">The Assimp scene containing the mesh data.</param>
        /// <param name="raySampleCount">Number of rays to sample for each pixel.</param>
        /// <param name="maxRayDistance">Maximum ray distance for intersection checks.</param>
        /// <param name="occlusionThreshold">Occlusion threshold used for optional clamping.</param>
        /// <param name="rayOriginBias">Small bias value to offset rays at the surface to avoid self-intersections.</param>
        /// <param name="useEnhancedTangentProcessing">If true, blends the bent normal with the original normal.</param>
        /// <param name="previewInterval">Interval in milliseconds at which to update the preview.</param>
        /// <param name="sampleAlpha">A delegate function to sample alpha values from a texture.</param>
        /// <param name="updatePreview">A delegate function to provide a preview bitmap to the UI.</param>
        /// <param name="updateTitle">A delegate function to update the UI window title.</param>
        /// <param name="invokeOnDispatcher">A delegate to marshal actions onto the UI thread.</param>
        /// <returns>A task that completes with the final preview bitmap.</returns>
        public static async Task<BakeResult> BakeBentNormalMapAsync(
            string modelPath, int texWidth, int texHeight,
            bool useTangentSpace, bool useCosineDistribution,
            bool generateBentNormalMap, bool generateOcclusionMap,
            Vector3 swizzle, IProgress<double> progress, CancellationToken token, bool clampOcclusion,
            int bentUVIndex, int alphaUVIndex, int selectedMeshIndex, Scene scene,
            int raySampleCount, float maxRayDistance, float occlusionThreshold, float rayOriginBias, bool useEnhancedTangentProcessing,
            int previewInterval,
            Func<Vector2, float> sampleAlpha,
            Action<Bitmap> updatePreview,
            Action<string> updateTitle,
            Action<Action> invokeOnDispatcher,
            int supersampleFactor, int uvPadding)
        {
            return await Task.Run(() =>
            {
                // Lower the baking thread's priority
                Thread.CurrentThread.Priority = ThreadPriority.BelowNormal;

                // Record the start time for progress calculations and throw if canceled.
                DateTime startTime = DateTime.UtcNow;
                token.ThrowIfCancellationRequested();

                // Determine if we are generating both bent normal and occlusion maps.
                bool dualMode = generateBentNormalMap && generateOcclusionMap;

                // Compute high-resolution dimensions based on the supersample factor.
                int width = texWidth * supersampleFactor;
                int height = texHeight * supersampleFactor;

                // Allocate buffers for the maps.
                int[] singleBuffer = null;
                int[] bentBuffer = null;
                int[] occBuffer = null;
                if (dualMode)
                {
                    bentBuffer = new int[width * height];
                    occBuffer = new int[width * height];
                    for (int i = 0; i < width * height; i++)
                    {
                        // Set background for bent normal map to 7A7FFF (with full alpha: 0xFF7A7FFF)
                        bentBuffer[i] = unchecked((int)0xFF7A7FFF);
                        // Set background for occlusion map to B9B9B9 (with full alpha: 0xFFB9B9B9)
                        occBuffer[i] = unchecked((int)0xFFB9B9B9);
                    }
                }
                else
                {
                    singleBuffer = new int[width * height];
                    // In single mode, determine which map is being generated and set the background accordingly.
                    if (generateBentNormalMap && !generateOcclusionMap)
                    {
                        for (int i = 0; i < singleBuffer.Length; i++)
                            singleBuffer[i] = unchecked((int)0xFF7A7FFF);
                    }
                    else if (generateOcclusionMap && !generateBentNormalMap)
                    {
                        for (int i = 0; i < singleBuffer.Length; i++)
                            singleBuffer[i] = unchecked((int)0xFFB9B9B9);
                    }
                }

                // Allocate an island mask buffer (initialize to -1 indicating background)
                int[] islandBuffer = new int[width * height];
                for (int i = 0; i < islandBuffer.Length; i++)
                    islandBuffer[i] = -1;


                // Ensure there is at least one mesh in the scene.
                if (scene == null)
                    throw new Exception("No mesh file loaded.");
                if (scene.MeshCount == 0)
                    throw new Exception("No mesh found in file.");


                // Retrieve the chosen mesh.
                var baseMesh = scene.Meshes[selectedMeshIndex];
                var meshNode = FindMeshNode(scene.RootNode, selectedMeshIndex);

                // If no node references the mesh, use identity transforms.
                NumericsMatrix nodeTransform = NumericsMatrix.Identity;
                NumericsMatrix nodeTransformInvT = NumericsMatrix.Identity;
                if (meshNode != null)
                {
                    nodeTransform = GetNodeGlobalTransform(meshNode);
                    if (NumericsMatrix.Invert(nodeTransform, out var inv))
                    {
                        nodeTransformInvT = NumericsMatrix.Transpose(inv);
                    }
                }

                // Gather triangles in world space, applying alpha UV where present.
                var occluderTriangles = new List<TriangleData>();
                if (baseMesh.HasTextureCoords(alphaUVIndex))
                {
                    for (int i = 0; i < baseMesh.FaceCount; i++)
                    {
                        Face face = baseMesh.Faces[i];
                        if (face.IndexCount != 3)
                            continue;
                        int i0 = face.Indices[0];
                        int i1 = face.Indices[1];
                        int i2 = face.Indices[2];

                        // Convert vertices to world space.
                        Vector3D av0 = baseMesh.Vertices[i0];
                        var localV0 = new Vector4(av0.X, av0.Y, av0.Z, 1);
                        var worldV0 = Vector4.Transform(localV0, nodeTransform);
                        Vector3 v0 = new Vector3(worldV0.X, worldV0.Y, worldV0.Z);

                        Vector3D av1 = baseMesh.Vertices[i1];
                        var localV1 = new Vector4(av1.X, av1.Y, av1.Z, 1);
                        var worldV1 = Vector4.Transform(localV1, nodeTransform);
                        Vector3 v1 = new Vector3(worldV1.X, worldV1.Y, worldV1.Z);

                        Vector3D av2 = baseMesh.Vertices[i2];
                        var localV2 = new Vector4(av2.X, av2.Y, av2.Z, 1);
                        var worldV2 = Vector4.Transform(localV2, nodeTransform);
                        Vector3 v2 = new Vector3(worldV2.X, worldV2.Y, worldV2.Z);

                        Vector2 uv0 = new Vector2(
                            baseMesh.TextureCoordinateChannels[alphaUVIndex][i0].X,
                            baseMesh.TextureCoordinateChannels[alphaUVIndex][i0].Y
                        );
                        Vector2 uv1 = new Vector2(
                            baseMesh.TextureCoordinateChannels[alphaUVIndex][i1].X,
                            baseMesh.TextureCoordinateChannels[alphaUVIndex][i1].Y
                        );
                        Vector2 uv2 = new Vector2(
                            baseMesh.TextureCoordinateChannels[alphaUVIndex][i2].X,
                            baseMesh.TextureCoordinateChannels[alphaUVIndex][i2].Y
                        );

                        occluderTriangles.Add(new TriangleData(v0, v1, v2, uv0, uv1, uv2));
                    }
                }

                // Build the BVH for occlusion testing.
                BVHNode bvhRoot = new BVHNode(occluderTriangles, 0);
                // If rayOriginBias is not provided (<= 0), compute a recommended value.
                // This computes the diagonal of the BVH's bounding box and sets the bias to 0.1% of that diagonal.
                if (rayOriginBias <= 0)
                {
                    Vector3 diag = bvhRoot.Box.Max - bvhRoot.Box.Min;
                    // For a normalized model, diag.Length() is about 1.73, so 0.00173 is a good start.
                    // You can also choose a fixed value like 0.002f if your models are normalized.
                    rayOriginBias = diag.Length() * 0.001f;
                    // Alternatively, you could use:
                    // rayOriginBias = 0.002f;
                }

                // Check if the selected bent UV channel is valid.
                if (!baseMesh.HasTextureCoords(bentUVIndex))
                    throw new Exception("Mesh does not have the selected UV channel for map baking.");

                var bentUVChannelMap = baseMesh.TextureCoordinateChannels[bentUVIndex];
                int vertCount = baseMesh.VertexCount;

                // Transform the mesh vertices, normals, tangents, and bitangents into world space.
                Vector3[] vertexPositions = new Vector3[vertCount];
                Vector3[] vertexNormals = new Vector3[vertCount];
                Vector3[] vertexTangents = new Vector3[vertCount];
                Vector3[] vertexBitangents = new Vector3[vertCount];

                for (int i = 0; i < vertCount; i++)
                {
                    Vector3D pos = baseMesh.Vertices[i];
                    var localPos = new Vector4(pos.X, pos.Y, pos.Z, 1f);
                    var worldPos = Vector4.Transform(localPos, nodeTransform);
                    vertexPositions[i] = new Vector3(worldPos.X, worldPos.Y, worldPos.Z);

                    Vector3D nrm = baseMesh.Normals[i];
                    var localNrm = new Vector4(nrm.X, nrm.Y, nrm.Z, 0f);
                    var worldNrm = Vector4.Transform(localNrm, nodeTransformInvT);
                    vertexNormals[i] = Vector3.Normalize(new Vector3(worldNrm.X, worldNrm.Y, worldNrm.Z));

                    if (baseMesh.HasTangentBasis)
                    {
                        Vector3D tan = baseMesh.Tangents[i];
                        var localTan = new Vector4(tan.X, tan.Y, tan.Z, 0f);
                        var worldTan = Vector4.Transform(localTan, nodeTransformInvT);
                        vertexTangents[i] = Vector3.Normalize(new Vector3(worldTan.X, worldTan.Y, worldTan.Z));

                        Vector3D bit = baseMesh.BiTangents[i];
                        var localBiT = new Vector4(bit.X, bit.Y, bit.Z, 0f);
                        var worldBiT = Vector4.Transform(localBiT, nodeTransformInvT);
                        vertexBitangents[i] = Vector3.Normalize(new Vector3(worldBiT.X, worldBiT.Y, worldBiT.Z));
                    }
                }

                // Set up progress tracking and preview updates.
                int processedTriangles = 0;
                int totalTriangles = baseMesh.FaceCount;
                // Compute per–triangle UV island IDs using the bent UV channel.
                int[] triangleIsland = ComputeUVIslands(baseMesh, bentUVIndex);

                double smoothingFactor = 0.8; // Adjust between 0 and 1; lower values smooth more.
                double lastProgress = 0.0;
                DateTime lastUpdateTime = startTime;
                TimeSpan smoothedETA = TimeSpan.Zero;
                // Tracks a smoothed “items per second” throughput.
                double smoothedThroughput = 0.0;
                // The last time we did a throughput check.
                DateTime lastCheckTime = startTime;
                // How many triangles we had processed at the last check.
                int lastProcessedTriangles = 0;

                // Pre-allocate the preview bitmap once.
                Bitmap previewBmp = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                // At the beginning of the Task.Run delegate, declare a lock object:
                object previewBmpLock = new object();

                // Start a timer to periodically update the UI and compute ETA.
                var previewTimer = new System.Threading.Timer(_ =>
                {
                    // Wrap the entire timer callback in a lock to prevent overlapping accesses.
                    lock (previewBmpLock)
                    {
                        // 1) Measure how many triangles were processed since last time
                        DateTime now = DateTime.UtcNow;
                        TimeSpan interval = now - lastCheckTime;

                        // The total number of triangles processed so far:
                        int currentProcessed = Interlocked.CompareExchange(ref processedTriangles, 0, 0);
                        // Triangles processed *this* interval:
                        int processedThisInterval = currentProcessed - lastProcessedTriangles;

                        // Update smoothed throughput (items/sec)
                        if (processedThisInterval > 0 && interval.TotalSeconds > 0)
                        {
                            double rawThroughput = processedThisInterval / interval.TotalSeconds;
                            if (smoothedThroughput < 1e-6)
                            {
                                // First measurement
                                smoothedThroughput = rawThroughput;
                            }
                            else
                            {
                                // Exponential smoothing
                                smoothedThroughput = smoothingFactor * smoothedThroughput + (1.0 - smoothingFactor) * rawThroughput;
                            }
                        }
                        // 2) Compute fraction of work done (0 to 1)
                        double fractionComplete = (double)currentProcessed / totalTriangles;
                        // Update "last check" for the next callback
                        lastProcessedTriangles = currentProcessed;
                        lastCheckTime = now;

                        if(fractionComplete < 1)
                        {

                            // 3) Compute an ETA string (if needed)
                            string remainingTimeText = "";
                            if (fractionComplete >= 0.1 && fractionComplete < 1.0 && smoothedThroughput > 0.01)
                            {
                                int itemsLeft = totalTriangles - currentProcessed;
                                double secondsLeft = itemsLeft / smoothedThroughput;
                                TimeSpan newEta = TimeSpan.FromSeconds(secondsLeft);
                                // Optionally, assign a formatted string to remainingTimeText.
                            }

                            float progressMultiplier = 1;

                            if(uvPadding > 0)
                            {
                                progressMultiplier = 0.5f;
                            }
                            
                            fractionComplete = fractionComplete * progressMultiplier;
                            progress.Report(fractionComplete);
                            // 4) Update title with progress and ETA
                            invokeOnDispatcher(() => updateTitle($"Baking... {fractionComplete:P0} {remainingTimeText}"));
                        }

                        // 5) Provide a preview image to the UI 
                        // Note: All bitmap access is inside the lock.
                        // Single-mode: update preview bitmap using unsafe code for memory copy.
                        if (!dualMode)
                        {
                            // Lock the preview bitmap.
                            var bmpData = previewBmp.LockBits(
                                new Rectangle(0, 0, width, height),
                                ImageLockMode.WriteOnly,
                                previewBmp.PixelFormat);

                            unsafe
                            {
                                // Pin the managed buffer.
                                fixed (int* src = singleBuffer)
                                {
                                    // Copy the entire buffer in one go.
                                    Buffer.MemoryCopy(
                                        src,
                                        bmpData.Scan0.ToPointer(),
                                        singleBuffer.Length * sizeof(int),  // destination size in bytes
                                        singleBuffer.Length * sizeof(int)); // number of bytes to copy
                                }
                            }

                            previewBmp.UnlockBits(bmpData);

                            // Downscale the high resolution previewBmp using the nearest neighbor algorithm
                            Bitmap downscaledPreview = NearestNeighborDownscale(previewBmp, 1024, 1024);

                            // Dispatch the updated bitmap to the UI.
                            invokeOnDispatcher(() => updatePreview(downscaledPreview));
                        }
                        else
                        {
                            // Dual-mode: update preview using the bentBuffer.
                            var bmpData = previewBmp.LockBits(
                                new Rectangle(0, 0, width, height),
                                ImageLockMode.WriteOnly,
                                previewBmp.PixelFormat);

                            unsafe
                            {
                                fixed (int* src = bentBuffer)
                                {
                                    Buffer.MemoryCopy(
                                        src,
                                        bmpData.Scan0.ToPointer(),
                                        bentBuffer.Length * sizeof(int),
                                        bentBuffer.Length * sizeof(int));
                                }
                            }

                            previewBmp.UnlockBits(bmpData);

                            // Downscale the high resolution previewBmp using the nearest neighbor algorithm
                            Bitmap downscaledPreview = NearestNeighborDownscale(previewBmp, 1024, 1024);

                            // Dispatch the downscaled preview to the UI.
                            invokeOnDispatcher(() => updatePreview(downscaledPreview));

                        }

                    } // End of lock (previewBmpLock)
                },
                null,
                previewInterval,  // due time
                previewInterval   // period
                );

                try
                {
                    // Process each triangle in parallel.
                    Parallel.For(0, totalTriangles, new ParallelOptions
                    {
                        CancellationToken = token,
                        MaxDegreeOfParallelism = Math.Max(1, Environment.ProcessorCount - 2)
                    },
                    triIndex =>
                    {
                        token.ThrowIfCancellationRequested();
                        Face face = baseMesh.Faces[triIndex];
                        if (face.IndexCount != 3)
                        {
                            Interlocked.Increment(ref processedTriangles);
                            return;
                        }

                        int i0 = face.Indices[0];
                        int i1 = face.Indices[1];
                        int i2 = face.Indices[2];

                        PointF uv0 = new PointF(bentUVChannelMap[i0].X, bentUVChannelMap[i0].Y);
                        PointF uv1 = new PointF(bentUVChannelMap[i1].X, bentUVChannelMap[i1].Y);
                        PointF uv2 = new PointF(bentUVChannelMap[i2].X, bentUVChannelMap[i2].Y);
                        PointF p0 = new PointF(uv0.X * width, (1 - uv0.Y) * height);
                        PointF p1 = new PointF(uv1.X * width, (1 - uv1.Y) * height);
                        PointF p2 = new PointF(uv2.X * width, (1 - uv2.Y) * height);

                        Vector3 pos0 = vertexPositions[i0];
                        Vector3 pos1 = vertexPositions[i1];
                        Vector3 pos2 = vertexPositions[i2];

                        Vector3 nor0 = vertexNormals[i0];
                        Vector3 nor1 = vertexNormals[i1];
                        Vector3 nor2 = vertexNormals[i2];

                        Vector3 tan0 = baseMesh.HasTangentBasis ? vertexTangents[i0] : Vector3.Zero;
                        Vector3 tan1 = baseMesh.HasTangentBasis ? vertexTangents[i1] : Vector3.Zero;
                        Vector3 tan2 = baseMesh.HasTangentBasis ? vertexTangents[i2] : Vector3.Zero;

                        // Depending on dual-mode, we rasterize and compute differently.
                        if (dualMode)
                        {
                            int currentIsland = triangleIsland[triIndex];

                            AARasterizer.RasterizeTriangleToDualBufferAA(
                                bentBuffer,
                                occBuffer,
                                width,
                                height,
                                p0,
                                p1,
                                p2,
                                (int x, int y, Vector3 bary, PointF uvInterp, out int bentColor, out int occColor) =>
                                {
                                    // Record the island ID for this pixel.
                                    int index = x + y * width;
                                    islandBuffer[index] = currentIsland;

                                    Vector3 interpNormal = Vector3.Normalize(
                                        bary.X * nor0 + bary.Y * nor1 + bary.Z * nor2);
                                    Vector3 posInterp = bary.X * pos0 + bary.Y * pos1 + bary.Z * pos2;
                                    Vector3 origin = posInterp + interpNormal * rayOriginBias;

                                    int sampleCountLocal = raySampleCount;
                                    float maxDistanceLocal = maxRayDistance;
                                    int unoccluded = 0;
                                    Vector3 sumDir = Vector3.Zero;
                                    Random rand = threadRandom.Value;
                                    Vector3 interpTangent = Vector3.Normalize(
                                        bary.X * tan0 + bary.Y * tan1 + bary.Z * tan2);

                                    for (int s = 0; s < sampleCountLocal; s++)
                                    {
                                        Vector3 sampleDir = GenerateSampleDirection(
                                            rand, interpNormal, interpTangent,
                                            useCosineDistribution, useEnhancedTangentProcessing);
                                        float? tHit = bvhRoot.IntersectRayWithAlphaNearest(origin, sampleDir, maxDistanceLocal, sampleAlpha);
                                        bool blocked = tHit.HasValue && tHit.Value < maxDistanceLocal;
                                        if (!blocked)
                                        {
                                            unoccluded++;
                                            sumDir += sampleDir;
                                        }
                                    }

                                    float occ = unoccluded / (float)sampleCountLocal;
                                    float finalOcc = clampOcclusion
                                        ? Math.Max(occ, occlusionThreshold)
                                        : occ < occlusionThreshold
                                            ? Lerp(occ, occlusionThreshold,
                                                (1 - occ / occlusionThreshold) * (1 - occ / occlusionThreshold))
                                            : occ;
                                    int gray = (int)Math.Clamp(finalOcc * 255, 0, 255);
                                    occColor = (255 << 24) | (gray << 16) | (gray << 8) | gray;

                                    float blendT = unoccluded / (float)sampleCountLocal;
                                    Vector3 rawBent = unoccluded > 0
                                        ? Vector3.Normalize(sumDir)
                                        : interpNormal;
                                    if (useEnhancedTangentProcessing)
                                    {
                                        rawBent = Vector3.Normalize(Vector3.Lerp(interpNormal, rawBent, blendT));
                                    }
                                    if (useTangentSpace)
                                    {
                                        Vector3 T = Vector3.Normalize(interpTangent);
                                        Vector3 N = interpNormal;
                                        Vector3 B = Vector3.Normalize(Vector3.Cross(N, T));
                                        rawBent = new Vector3(
                                            Vector3.Dot(rawBent, T),
                                            Vector3.Dot(rawBent, B),
                                            Vector3.Dot(rawBent, N));
                                    }
                                    bentColor = ColorToInt(rawBent, swizzle);
                                    // Do not write directly to bentBuffer/occBuffer here.
                                });
                        }
                        else
                        {
                            int currentIsland = triangleIsland[triIndex];

                            AARasterizer.RasterizeTriangleToBufferAA(
                                singleBuffer,
                                width,
                                height,
                                p0,
                                p1,
                                p2,
                                (x, y, bary, uvInterp) =>
                                {
                                    int index = x + y * width;
                                    islandBuffer[index] = currentIsland;

                                    Vector3 interpNormal = Vector3.Normalize(
                                        bary.X * nor0 + bary.Y * nor1 + bary.Z * nor2);
                                    Vector3 posInterp = bary.X * pos0 + bary.Y * pos1 + bary.Z * pos2;
                                    Vector3 origin = posInterp + interpNormal * rayOriginBias;

                                    int sampleCountLocal = raySampleCount;
                                    float maxDistanceLocal = maxRayDistance;
                                    int unoccluded = 0;
                                    Vector3 sumDir = Vector3.Zero;
                                    Random rand = threadRandom.Value;
                                    Vector3 interpTangent = Vector3.Zero;
                                    if (baseMesh.HasTangentBasis)
                                    {
                                        interpTangent = Vector3.Normalize(
                                            bary.X * tan0 + bary.Y * tan1 + bary.Z * tan2);
                                    }

                                    for (int s = 0; s < sampleCountLocal; s++)
                                    {
                                        Vector3 sampleDir = GenerateSampleDirection(
                                            rand, interpNormal, interpTangent,
                                            useCosineDistribution, useEnhancedTangentProcessing);
                                        float? tHit = bvhRoot.IntersectRayWithAlphaNearest(origin, sampleDir, maxDistanceLocal, sampleAlpha);
                                        bool blocked = tHit.HasValue && tHit.Value < maxDistanceLocal;
                                        if (!blocked)
                                        {
                                            unoccluded++;
                                            sumDir += sampleDir;
                                        }
                                    }

                                    float occ = unoccluded / (float)sampleCountLocal;
                                    float finalOcc = clampOcclusion
                                        ? Math.Max(occ, occlusionThreshold)
                                        : occ < occlusionThreshold
                                            ? Lerp(occ, occlusionThreshold,
                                                (1 - occ / occlusionThreshold) * (1 - occ / occlusionThreshold))
                                            : occ;
                                    int gray = (int)Math.Clamp(finalOcc * 255, 0, 255);
                                    int occColor = (255 << 24) | (gray << 16) | (gray << 8) | gray;

                                    float blendT = unoccluded / (float)sampleCountLocal;
                                    Vector3 rawBent = unoccluded > 0
                                        ? Vector3.Normalize(sumDir)
                                        : interpNormal;
                                    if (useEnhancedTangentProcessing)
                                    {
                                        rawBent = Vector3.Normalize(Vector3.Lerp(interpNormal, rawBent, blendT));
                                    }
                                    if (useTangentSpace)
                                    {
                                        Vector3 T = Vector3.Normalize(interpTangent);
                                        Vector3 N = interpNormal;
                                        Vector3 B = Vector3.Normalize(Vector3.Cross(N, T));
                                        rawBent = new Vector3(
                                            Vector3.Dot(rawBent, T),
                                            Vector3.Dot(rawBent, B),
                                            Vector3.Dot(rawBent, N));
                                    }
                                    int colorInt = generateOcclusionMap && !generateBentNormalMap
                                        ? occColor
                                        : ColorToInt(rawBent, swizzle);
                                    return colorInt;
                                });
                        }

                        Interlocked.Increment(ref processedTriangles);

                    });
                }
                finally
                {
                    // Dispose preview timer and restore title if canceled.
                    previewTimer.Dispose();
                    invokeOnDispatcher(() =>
                    {
                        if (token.IsCancellationRequested)
                        {
                            updateTitle("Normal Smith");
                        }
                    });
                }

                progress.Report(1);
                previewTimer.Dispose();
                progress.Report(1);

                // After processing, create the high-resolution bitmap:
                // After processing, create the final output bitmap(s)
                if (!dualMode)
                {

                    void ReportBfsProgress(double localFrac, double start, double chunk)
                    {
                        double overall = start + localFrac * chunk;
                        //progress.Report(overall);

                        // Also update the window title on the UI thread
                        invokeOnDispatcher(() =>
                        {
                            updateTitle($"Island Padding: {overall * 100:0.0}% done");
                        });
                    }

                    invokeOnDispatcher(() => updateTitle("Copying from buffer... 50%"));
                    // Single mode: only one map is generated.
                    Bitmap highResBmp = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                    var bmpData = highResBmp.LockBits(
                        new Rectangle(0, 0, width, height),
                        ImageLockMode.WriteOnly,
                        highResBmp.PixelFormat);
                    Marshal.Copy(singleBuffer, 0, bmpData.Scan0, singleBuffer.Length);
                    highResBmp.UnlockBits(bmpData);

                    // Choose the appropriate background color (for example, use the bent map background if generating bent normals).
                    int bgColor = generateBentNormalMap ? unchecked((int)0xFF7A7FFF) : unchecked((int)0xFFB9B9B9);
                    int[] dilatedIslandMask;
                    // BFS #1 => go from 90%..95%
                    if (uvPadding > 0)
                    {
                        double start1 = 0.50;
                        double chunk1 = 0.25;
                        dilatedIslandMask = DilateIslandMask(islandBuffer, width, height, uvPadding, token, frac =>
                        {
                            // Map local BFS fraction (0..1) => overall progress slice, if you want
                            // e.g. 90..95%
                            double overall = 0.50 + frac * 0.25;
                            progress.Report(overall);

                            // Update window title on UI thread
                            invokeOnDispatcher(() =>
                            {
                                updateTitle($"Island Padding (Mask)... {overall * 100:0}%");
                            });
                        });

                        // BFS #2 => go from 95%..100%
                        double start2 = 0.75;
                        double chunk2 = 0.25;
                        highResBmp = ApplyIslandColorDilation(
                            highResBmp,
                            dilatedIslandMask,
                            width,
                            height,
                            uvPadding,
                            bgColor,
                            token,
                            frac =>
                            {
                                // e.g. 95..100%
                                double overall = 0.75 + frac * 0.25;
                                progress.Report(overall);
                                invokeOnDispatcher(() => {
                                    updateTitle($"Island Padding (Map)... {overall * 100:0}%");
                                });
                            }
                        );
                    }
                    else
                    {
                        // If no padding, BFS doesn't run => just jump to 100%
                        progress.Report(1.0);
                    }

                    // At this point, BFS is done => we must be at 100%
                    progress.Report(1.0);

                    if(supersampleFactor>1)
                    {
                        invokeOnDispatcher(() => updateTitle("Downscaling Image..."));
                    }

                    // Downscale the high-res bitmap to the desired output dimensions:
                    Bitmap finalBmp = new Bitmap(texWidth, texHeight, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                    using (Graphics g = Graphics.FromImage(finalBmp))
                    {
                        g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                        g.DrawImage(highResBmp, new Rectangle(0, 0, texWidth, texHeight));
                    }
                    highResBmp.Dispose();

                    invokeOnDispatcher(() => updateTitle("Creating Final Bake Images..."));

                    // Create the BakeResult and assign the appropriate map based on which flag is true.
                    BakeResult bakeResult = new BakeResult();
                    if (generateBentNormalMap && !generateOcclusionMap)
                    {
                        bakeResult.BentMap = finalBmp;
                    }
                    else if (generateOcclusionMap && !generateBentNormalMap)
                    {
                        bakeResult.OccMap = finalBmp;
                    }
                    bakeResult.PreviewBmp = finalBmp;
                    return bakeResult;
                }
                else
                {
                    void ReportBfsProgress(double localFrac, double start, double chunk)
                    {
                        double overall = start + localFrac * chunk;
                        //progress.Report(overall);

                        // Also update the window title on the UI thread
                        invokeOnDispatcher(() =>
                        {
                            updateTitle($"Island Padding: {overall * 100:0.0}% done");
                        });
                    }

                    // Dual mode: both Bent and Occlusion maps are generated.
                    invokeOnDispatcher(() => updateTitle("Copying from buffer... 50%"));
                    // Create separate high-resolution bitmaps for bent and occlusion maps.
                    Bitmap highResBentBmp = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                    Bitmap highResOccBmp = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                    // Copy the bentBuffer into its bitmap.
                    var bentData = highResBentBmp.LockBits(
                        new Rectangle(0, 0, width, height),
                        ImageLockMode.WriteOnly,
                        highResBentBmp.PixelFormat);
                    Marshal.Copy(bentBuffer, 0, bentData.Scan0, bentBuffer.Length);
                    highResBentBmp.UnlockBits(bentData);

                    // Copy the occBuffer into its bitmap.
                    var occData = highResOccBmp.LockBits(
                        new Rectangle(0, 0, width, height),
                        ImageLockMode.WriteOnly,
                        highResOccBmp.PixelFormat);
                    Marshal.Copy(occBuffer, 0, occData.Scan0, occBuffer.Length);
                    highResOccBmp.UnlockBits(occData);

                    // Choose the appropriate background color (for example, use the bent map background if generating bent normals).
                    int bgColor = generateBentNormalMap ? unchecked((int)0xFF7A7FFF) : unchecked((int)0xFFB9B9B9);
                    // Choose the occlusion background color.
                    int bgColor2 = unchecked((int)0xFFB9B9B9);

                    int[] dilatedIslandMask;
                    // BFS #1 => go from 90%..95%
                    if (uvPadding > 0)
                    {
                        double start1 = 0.50;
                        double chunk1 = 0.17;
                        dilatedIslandMask = DilateIslandMask(islandBuffer, width, height, uvPadding, token, frac =>
                        {
                            // Map local BFS fraction (0..1) => overall progress slice, if you want
                            // e.g. 90..95%
                            double overall = 0.50 + frac * 0.17;
                            progress.Report(overall);

                            // Update window title on UI thread
                            invokeOnDispatcher(() =>
                            {
                                updateTitle($"Island Padding (Mask)... {overall * 100:0}%");
                            });
                        });

                        // BFS #2 => go from 95%..100%
                        double start2 = 0.67;
                        double chunk2 = 0.16;
                        highResBentBmp = ApplyIslandColorDilation(
                            highResBentBmp,
                            dilatedIslandMask,
                            width,
                            height,
                            uvPadding,
                            bgColor,
                            token,
                            frac =>
                            {
                                // e.g. 95..100%
                                double overall = 0.67 + frac * 0.16;
                                progress.Report(overall);
                                invokeOnDispatcher(() => {
                                    updateTitle($"Island Padding (Bent Map)... {overall * 100:0}%");
                                });
                            }
                        );

                        // BFS #2 => go from 95%..100%
                        double start3 = 0.83;
                        double chunk3 = 0.17;
                        highResOccBmp = ApplyIslandColorDilation(
                            highResOccBmp,
                            dilatedIslandMask,
                            width,
                            height,
                            uvPadding,
                            bgColor2,
                            token,
                            frac =>
                            {
                                // e.g. 95..100%
                                double overall = 0.83 + frac * 0.17;
                                progress.Report(overall);
                                invokeOnDispatcher(() => {
                                    updateTitle($"Island Padding (Occ Map) {overall * 100:0}%");
                                });
                            }
                        );
                    }
                    else
                    {
                        // If no padding, BFS doesn't run => just jump to 100%
                        progress.Report(1.0);
                    }

                    if (supersampleFactor > 1)
                    {
                        invokeOnDispatcher(() => updateTitle("Downscaling Images..."));
                    }

                    // Downscale the bent bitmap.
                    Bitmap finalBentBmp = new Bitmap(texWidth, texHeight, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                    using (Graphics g = Graphics.FromImage(finalBentBmp))
                    {
                        g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                        g.DrawImage(highResBentBmp, new Rectangle(0, 0, texWidth, texHeight));
                    }
                    highResBentBmp.Dispose();

                    // Downscale the occlusion bitmap.
                    Bitmap finalOccBmp = new Bitmap(texWidth, texHeight, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                    using (Graphics g = Graphics.FromImage(finalOccBmp))
                    {
                        g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                        g.DrawImage(highResOccBmp, new Rectangle(0, 0, texWidth, texHeight));
                    }
                    highResOccBmp.Dispose();

                    invokeOnDispatcher(() => updateTitle("Creating Final Bake Images..."));

                    // Create the BakeResult with both maps.
                    BakeResult bakeResult = new BakeResult
                    {
                        BentMap = finalBentBmp,    // Bent normal map.
                        OccMap = finalOccBmp,      // Occlusion map.
                        PreviewBmp = finalBentBmp  // Preview uses the bent map (or you can combine them as needed).
                    };
                    return bakeResult;
                }

            }, token);
        }

        #endregion

        #region Helper Methods

        private static Bitmap ParallelDownscaleHighQuality(
            Bitmap highResBmp,
            int finalWidth,
            int finalHeight,
            IProgress<double> progress,
            CancellationToken token)
        {
            // Create the final output bitmap.
            Bitmap finalBmp = new Bitmap(finalWidth, finalHeight, highResBmp.PixelFormat);

            // For simplicity, we split the image vertically into a number of sections.
            // You can adjust numSections based on processor count or a fixed value.
            int numSections = 4;  // For example, split into 4 vertical sections.
            int sectionWidth = finalWidth / numSections;

            // Determine scale factors from the high-res image to the final image.
            double scaleX = (double)highResBmp.Width / finalWidth;
            double scaleY = (double)highResBmp.Height / finalHeight;

            // Lock object to protect finalBmp while merging sections.
            object finalBmpLock = new object();

            // Create tasks for each section.
            List<Task> tasks = new List<Task>();
            for (int i = 0; i < numSections; i++)
            {
                // Calculate the destination rectangle for this section.
                int destX = i * sectionWidth;
                int destWidth = (i == numSections - 1) ? (finalWidth - destX) : sectionWidth;
                Rectangle destRect = new Rectangle(destX, 0, destWidth, finalHeight);

                // Calculate the corresponding source rectangle.
                // We assume a uniform scale in X and Y.
                Rectangle sourceRect = new Rectangle(
                    (int)(destRect.X * scaleX),
                    0,
                    (int)(destRect.Width * scaleX),
                    highResBmp.Height);  // Using the full height of the high-res image

                // Create a task for processing this section.
                tasks.Add(Task.Run(() =>
                {
                    token.ThrowIfCancellationRequested();

                    // Create a temporary bitmap for this section.
                    using (Bitmap sectionBmp = new Bitmap(destRect.Width, destRect.Height, highResBmp.PixelFormat))
                    {
                        // Use Graphics to perform high-quality bicubic downscaling on this section.
                        using (Graphics g = Graphics.FromImage(sectionBmp))
                        {
                            g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                            g.DrawImage(highResBmp, new Rectangle(0, 0, destRect.Width, destRect.Height), sourceRect, GraphicsUnit.Pixel);
                        }

                        // Copy the downscaled section into the final bitmap.
                        lock (finalBmpLock)
                        {
                            using (Graphics gFinal = Graphics.FromImage(finalBmp))
                            {
                                gFinal.DrawImage(sectionBmp, destRect);
                            }
                        }
                    }
                    // Report progress for this section.
                    progress?.Report((double)(i + 1) / numSections);
                }, token));
            }

            // Wait for all sections to finish.
            Task.WaitAll(tasks.ToArray());

            return finalBmp;
        }

        private static Bitmap NearestNeighborDownscale(Bitmap source, int newWidth, int newHeight)
        {
            // Create the destination bitmap with the target dimensions.
            Bitmap dest = new Bitmap(newWidth, newHeight, source.PixelFormat);

            // Lock both source and destination bitmaps for efficient memory access.
            var sourceData = source.LockBits(new Rectangle(0, 0, source.Width, source.Height),
                                             ImageLockMode.ReadOnly, source.PixelFormat);
            var destData = dest.LockBits(new Rectangle(0, 0, newWidth, newHeight),
                                         ImageLockMode.WriteOnly, source.PixelFormat);

            int bytesPerPixel = Image.GetPixelFormatSize(source.PixelFormat) / 8;

            unsafe
            {
                byte* srcPtr = (byte*)sourceData.Scan0;
                byte* destPtr = (byte*)destData.Scan0;

                // For each destination pixel, map to the corresponding source pixel.
                for (int y = 0; y < newHeight; y++)
                {
                    // Compute source y-coordinate via nearest neighbor.
                    int srcY = y * source.Height / newHeight;

                    for (int x = 0; x < newWidth; x++)
                    {
                        // Compute source x-coordinate via nearest neighbor.
                        int srcX = x * source.Width / newWidth;

                        // Get pointers to the source and destination pixels.
                        byte* srcPixel = srcPtr + (srcY * sourceData.Stride) + (srcX * bytesPerPixel);
                        byte* destPixel = destPtr + (y * destData.Stride) + (x * bytesPerPixel);

                        // Copy the pixel data (for each channel).
                        for (int i = 0; i < bytesPerPixel; i++)
                        {
                            destPixel[i] = srcPixel[i];
                        }
                    }
                }
            }

            // Unlock the bitmaps.
            source.UnlockBits(sourceData);
            dest.UnlockBits(destData);

            return dest;
        }


        private static Bitmap ApplyUvPadding(Bitmap source, int padding)
        {
            Bitmap padded = (Bitmap)source.Clone();
            int width = padded.Width;
            int height = padded.Height;

            // Perform iterative dilation for the specified padding amount.
            for (int iter = 0; iter < padding; iter++)
            {
                Bitmap temp = (Bitmap)padded.Clone();
                for (int x = 1; x < width - 1; x++)
                {
                    for (int y = 1; y < height - 1; y++)
                    {
                        System.Drawing.Color current = padded.GetPixel(x, y);
                        // Assume the background is pure black (0xFF000000). Adjust if necessary.
                        if (current.ToArgb() == unchecked((int)0xFF000000))
                        {
                            // Check neighboring pixels.
                            System.Drawing.Color left = padded.GetPixel(x - 1, y);
                            System.Drawing.Color right = padded.GetPixel(x + 1, y);
                            System.Drawing.Color up = padded.GetPixel(x, y - 1);
                            System.Drawing.Color down = padded.GetPixel(x, y + 1);
                            // If any neighbor is non-background, set this pixel to that color.
                            if (left.ToArgb() != unchecked((int)0xFF000000))
                                temp.SetPixel(x, y, left);
                            else if (right.ToArgb() != unchecked((int)0xFF000000))
                                temp.SetPixel(x, y, right);
                            else if (up.ToArgb() != unchecked((int)0xFF000000))
                                temp.SetPixel(x, y, up);
                            else if (down.ToArgb() != unchecked((int)0xFF000000))
                                temp.SetPixel(x, y, down);
                        }
                    }
                }
                padded.Dispose();
                padded = temp;
            }
            return padded;
        }

        /// <summary>
        /// Expands each island ID in the given 'mask' array by 'uvPadding' pixels.
        /// Morphologically dilates any pixel whose mask ID is >= 0, so that any background pixel (-1)
        /// that touches an island becomes that island's ID. This is a replacement for the BFS mask-expansion.
        /// </summary>
        /// <param name="mask">An integer array of length width*height, where -1=background, or an integer island ID.</param>
        /// <param name="width">Image width.</param>
        /// <param name="height">Image height.</param>
        /// <param name="uvPadding">Distance (in pixels) to expand island IDs outward.</param>
        /// <param name="token">Cancellation token for aborting.</param>
        /// <param name="bfsProgress">Optional callback for progress, from 0..1.</param>
        /// <returns>An expanded integer array, same size, with islands grown outward by 'uvPadding' pixels.</returns>
        private static int[] DilateIslandMask(
            int[] mask,
            int width,
            int height,
            int uvPadding,
            CancellationToken token,
            Action<double> bfsProgress = null
        )
        {
            // 1) If no padding, just return a copy (no changes).
            if (uvPadding <= 0)
            {
                bfsProgress?.Invoke(1.0);
                return (int[])mask.Clone();
            }

            // We'll do 'uvPadding' passes; each pass grows the mask out by 1 pixel in all directions.
            int[] result = (int[])mask.Clone();

            // We only do 4-neighbor expansions here, to match your BFS code's up/down/left/right logic.
            // If you'd like 8 neighbors, just add the diagonals.
            int[] offsets = { -1, 1, -width, width };

            // 2) Morphologically expand in layers.
            for (int iteration = 1; iteration <= uvPadding; iteration++)
            {
                token.ThrowIfCancellationRequested();

                // We'll create a new array for the next "layer" so we don't partially overwrite ourselves.
                int[] nextMask = (int[])result.Clone();

                // For each pixel that is currently -1, see if any neighbor has a valid island ID.
                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        token.ThrowIfCancellationRequested();

                        int idx = x + y * width;
                        if (result[idx] == -1) // background
                        {
                            // Check neighbors
                            foreach (int off in offsets)
                            {
                                int nx = x, ny = y;
                                if (off == -1) nx = x - 1;
                                else if (off == 1) nx = x + 1;
                                else if (off == -width) ny = y - 1;
                                else if (off == width) ny = y + 1;

                                // Bounds check
                                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                                    continue;

                                int nIdx = nx + ny * width;
                                int neighborIslandId = result[nIdx];

                                // If neighbor has a valid island, adopt it
                                if (neighborIslandId != -1)
                                {
                                    nextMask[idx] = neighborIslandId;
                                    break;  // We found an island neighbor, so we can stop checking
                                }
                            }
                        }
                    }
                }

                // Copy the newly expanded mask for the next iteration
                result = nextMask;

                // (Optional) Provide iteration-based progress
                double frac = (double)iteration / uvPadding;
                if (frac > 1.0) frac = 1.0;
                bfsProgress?.Invoke(frac);
            }

            // 3) Return the final grown mask
            bfsProgress?.Invoke(1.0);
            return result;
        }

        /// <summary>
        /// Morphologically dilates each UV island by 'uvPadding' pixels, 
        /// filling the background color around each island with the island's color.
        /// This replaces the BFS-based color dilation, but yields the same final result
        /// in a simpler and often faster way.
        /// </summary>
        /// <param name="source">The source bitmap (ARGB).</param>
        /// <param name="islandMask">
        /// An array of length (width*height) that gives the island ID for each pixel, or -1 for background.
        /// </param>
        /// <param name="width">Bitmap width.</param>
        /// <param name="height">Bitmap height.</param>
        /// <param name="uvPadding">Distance in pixels to expand around the island.</param>
        /// <param name="backgroundColor">ARGB integer indicating background color (e.g. 0xFF000000).</param>
        /// <param name="token">Cancellation token to allow user to abort.</param>
        /// <param name="bfsProgress">
        /// Optional progress callback, reporting fraction from 0..1 across all iterations.
        /// </param>
        /// <returns>
        /// A new Bitmap in which each island’s color has been expanded by `uvPadding` pixels 
        /// into the background.
        /// </returns>
        private static Bitmap ApplyIslandColorDilation(
            Bitmap source,
            int[] islandMask,
            int width,
            int height,
            int uvPadding,
            int backgroundColor,
            CancellationToken token,
            Action<double> bfsProgress = null
        )
        {
            // 1) If no padding, just return a clone
            if (uvPadding <= 0)
            {
                bfsProgress?.Invoke(1.0);
                return (Bitmap)source.Clone();
            }

            // 2) Clone and lock the bitmap
            Bitmap result = (Bitmap)source.Clone();
            var rect = new Rectangle(0, 0, width, height);
            var data = result.LockBits(rect, System.Drawing.Imaging.ImageLockMode.ReadWrite, result.PixelFormat);

            int[] pixelData = new int[width * height];
            System.Runtime.InteropServices.Marshal.Copy(data.Scan0, pixelData, 0, pixelData.Length);

            // For 8 neighbors, add diagonals
            int[] offsets = {
                            -1, +1,             // left, right
                            -width, +width,     // up, down
                            -width-1, -width+1,
                            +width-1, +width+1
                        };


            for (int iteration = 1; iteration <= uvPadding; iteration++)
            {
                token.ThrowIfCancellationRequested();

                // We’ll build a new array for the “next” state after this iteration
                int[] newPixelData = (int[])pixelData.Clone();

                // For each pixel that is background, check neighbors
                for (int yPos = 0; yPos < height; yPos++)
                {
                    for (int xPos = 0; xPos < width; xPos++)
                    {
                        token.ThrowIfCancellationRequested();

                        int idx = xPos + yPos * width;

                        // If this pixel is background, see if it has a neighbor with the same island
                        if (pixelData[idx] == backgroundColor)
                        {
                            int cid = islandMask[idx];
                            if (cid == -1)
                            {
                                // It's truly outside any island => skip
                                continue;
                            }

                            // We have a background pixel that belongs to an island in mask,
                            // so let's see if we can get a neighbor's color from the same island.
                            bool foundColor = false;
                            int neighborColor = backgroundColor;

                            foreach (int off in offsets)
                            {
                                int nx = xPos, ny = yPos;
                                if (off == -1) nx = xPos - 1;
                                else if (off == 1) nx = xPos + 1;
                                else if (off == -width) ny = yPos - 1;
                                else if (off == width) ny = yPos + 1;

                                // Bounds check
                                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                                    continue;

                                int nIdx = nx + ny * width;
                                int neighborIsland = islandMask[nIdx];
                                // neighbor is the same island & not background => we can “borrow” that color
                                if (neighborIsland == cid && pixelData[nIdx] != backgroundColor)
                                {
                                    foundColor = true;
                                    neighborColor = pixelData[nIdx];
                                    break;
                                }
                            }

                            // If found a same-island neighbor with real color => fill it
                            if (foundColor)
                            {
                                newPixelData[idx] = neighborColor;
                            }
                        }
                    }
                }

                // Copy the new state for next iteration
                pixelData = newPixelData;

                // (Optional) Provide progress for each iteration
                double frac = (double)iteration / uvPadding;
                if (frac > 1.0) frac = 1.0;
                bfsProgress?.Invoke(frac);
            }

            // 4) Copy final pixel data back and unlock
            System.Runtime.InteropServices.Marshal.Copy(pixelData, 0, data.Scan0, pixelData.Length);
            result.UnlockBits(data);

            // 5) Return the final dilated bitmap
            return result;
        }
        /// <summary>
        /// Computes a UV island ID for each triangle using the given UV channel.
        /// Two triangles are grouped together if they share an edge in UV space
        /// (within the provided floating-point tolerance).
        /// 
        /// By default, tolerance=1e-4 means we treat points that differ by less
        /// than 0.0001 as "the same," but you can choose a smaller or larger
        /// tolerance to control whether nearly adjacent UV edges are merged.
        /// </summary>
        /// <param name="mesh">The mesh to analyze.</param>
        /// <param name="uvChannel">The UV channel index to read from.</param>
        /// <param name="tolerance">
        /// Any two UVs that differ by less than 'tolerance' are considered identical
        /// when building edges. e.g. 1e-5 => very tight matching.
        /// </param>
        /// <returns>
        /// An array of length=mesh.FaceCount, giving an island ID for each face.
        /// Faces that share edges in UV space (within tolerance) end up in the same island.
        /// </returns>
        private static int[] ComputeUVIslands(Assimp.Mesh mesh, int uvChannel, float tolerance = 1e-4f)
        {
            int triangleCount = mesh.FaceCount;
            int[] parent = new int[triangleCount];
            for (int i = 0; i < triangleCount; i++)
                parent[i] = i;

            // Union–find helpers
            int Find(int i)
            {
                while (parent[i] != i)
                    i = parent[i];
                return i;
            }
            void Union(int i, int j)
            {
                int rootI = Find(i);
                int rootJ = Find(j);
                if (rootI != rootJ)
                    parent[rootJ] = rootI;
            }

            // Convert a UV coordinate to an integer pair, using tolerance as the "snap" size.
            // Example: tolerance=1e-5 => multiply by 100000, then Round().
            (int, int) Quantize(Vector3D uv)
            {
                float scale = 1f / tolerance;
                int qx = (int)Math.Round(uv.X * scale);
                int qy = (int)Math.Round(uv.Y * scale);
                return (qx, qy);
            }

            // Build a dictionary mapping an "edge key" => list of triangle indices sharing that edge.
            var edgeDict = new Dictionary<(int, int, int, int), List<int>>();

            for (int tri = 0; tri < triangleCount; tri++)
            {
                Assimp.Face face = mesh.Faces[tri];
                if (face.IndexCount != 3)
                    continue;

                // Grab the three UVs for this face
                Vector3D uv0 = mesh.TextureCoordinateChannels[uvChannel][face.Indices[0]];
                Vector3D uv1 = mesh.TextureCoordinateChannels[uvChannel][face.Indices[1]];
                Vector3D uv2 = mesh.TextureCoordinateChannels[uvChannel][face.Indices[2]];

                // Quantize to integer pairs
                var q0 = Quantize(uv0);
                var q1 = Quantize(uv1);
                var q2 = Quantize(uv2);

                // Define each edge as an order–independent tuple of (minX, minY, maxX, maxY).
                var edges = new List<(int, int, int, int)>
        {
            (
                Math.Min(q0.Item1, q1.Item1), Math.Min(q0.Item2, q1.Item2),
                Math.Max(q0.Item1, q1.Item1), Math.Max(q0.Item2, q1.Item2)
            ),
            (
                Math.Min(q1.Item1, q2.Item1), Math.Min(q1.Item2, q2.Item2),
                Math.Max(q1.Item1, q2.Item1), Math.Max(q1.Item2, q2.Item2)
            ),
            (
                Math.Min(q2.Item1, q0.Item1), Math.Min(q2.Item2, q0.Item2),
                Math.Max(q2.Item1, q0.Item1), Math.Max(q2.Item2, q0.Item2)
            )
        };

                // Add each edge to the dictionary
                foreach (var edge in edges)
                {
                    if (!edgeDict.TryGetValue(edge, out var list))
                    {
                        list = new List<int>();
                        edgeDict[edge] = list;
                    }
                    list.Add(tri);
                }
            }

            // Union triangles that share an edge in the dictionary
            foreach (var kvp in edgeDict)
            {
                List<int> tris = kvp.Value;
                if (tris.Count > 1)
                {
                    for (int i = 1; i < tris.Count; i++)
                        Union(tris[0], tris[i]);
                }
            }

            // Flatten the union–find structure
            int[] islandIDs = new int[triangleCount];
            for (int i = 0; i < triangleCount; i++)
                islandIDs[i] = Find(i);

            return islandIDs;
        }


        /// <summary>
        /// Locates the node that references the specified mesh index in the Assimp scene hierarchy.
        /// Returns null if no node references the mesh.
        /// </summary>
        private static Node FindMeshNode(Node node, int meshIndex)
        {
            if (node.MeshIndices.Contains(meshIndex))
                return node;
            foreach (var child in node.Children)
            {
                var result = FindMeshNode(child, meshIndex);
                if (result != null)
                    return result;
            }
            return null;
        }

        /// <summary>
        /// Traverses parent nodes to compute the global transform for the specified node.
        /// </summary>
        private static NumericsMatrix GetNodeGlobalTransform(Node node)
        {
            NumericsMatrix transform = node.Transform.ToMatrix4x4();
            var current = node.Parent;
            while (current != null)
            {
                transform = NumericsMatrix.Multiply(current.Transform.ToMatrix4x4(), transform);
                current = current.Parent;
            }
            return transform;
        }

        /// <summary>
        /// Performs a linear interpolation between a and b by factor t.
        /// </summary>
        private static float Lerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        /// <summary>
        /// Converts a normal vector in the -1..1 range to an ARGB int, applying a swizzle factor.
        /// </summary>
        private static int ColorToInt(Vector3 normal, Vector3 swizzle)
        {
            normal.X *= swizzle.X;
            normal.Y *= swizzle.Y;
            normal.Z *= swizzle.Z;

            int r = (int)Math.Clamp((normal.X * 0.5f + 0.5f) * 255, 0, 255);
            int g = (int)Math.Clamp((normal.Y * 0.5f + 0.5f) * 255, 0, 255);
            int b = (int)Math.Clamp((normal.Z * 0.5f + 0.5f) * 255, 0, 255);
            return 255 << 24 | r << 16 | g << 8 | b;
        }

        /// <summary>
        /// Generates a sample direction for ray casting. Supports both cosine-weighted and uniform hemisphere sampling.
        /// Optionally applies an enhanced tangent approach for better blending.
        /// </summary>
        private static Vector3 GenerateSampleDirection(
            Random rand,
            Vector3 normal,
            Vector3 tangent,
            bool useCosine,
            bool enhancedTangent)
        {
            if (useCosine)
            {
                float ru = (float)rand.NextDouble();
                float rv = (float)rand.NextDouble();
                float rVal = MathF.Sqrt(ru);
                float theta = 2 * MathF.PI * rv;

                Vector3 N = normal;
                Vector3 T = tangent;
                if (enhancedTangent && T.LengthSquared() > 0.0001f)
                {
                    Vector3 B = Vector3.Normalize(Vector3.Cross(N, T));
                    Vector3 sampleLocal = new Vector3(
                        rVal * MathF.Cos(theta),
                        rVal * MathF.Sin(theta),
                        MathF.Sqrt(1 - ru)
                    );
                    return sampleLocal.X * T + sampleLocal.Y * B + sampleLocal.Z * N;
                }
                else
                {
                    Vector3 helper = MathF.Abs(N.X) > 0.9f ? new Vector3(0, 1, 0) : new Vector3(1, 0, 0);
                    Vector3 T2 = Vector3.Normalize(Vector3.Cross(helper, N));
                    Vector3 B2 = Vector3.Normalize(Vector3.Cross(N, T2));

                    Vector3 sampleLocal = new Vector3(
                        rVal * MathF.Cos(theta),
                        rVal * MathF.Sin(theta),
                        MathF.Sqrt(1 - ru)
                    );
                    return sampleLocal.X * T2 + sampleLocal.Y * B2 + sampleLocal.Z * N;
                }
            }
            else
            {
                float u = (float)rand.NextDouble();
                float v = (float)rand.NextDouble();
                float phi = 2f * MathF.PI * u;
                float cosTheta = 1f - 2f * v;
                float sinTheta = MathF.Sqrt(1f - cosTheta * cosTheta);

                Vector3 sampleLocal = new Vector3(
                    sinTheta * MathF.Cos(phi),
                    sinTheta * MathF.Sin(phi),
                    cosTheta
                );

                Vector3 N = normal;
                Vector3 T = tangent;
                if (enhancedTangent && T.LengthSquared() > 0.0001f)
                {
                    Vector3 B = Vector3.Normalize(Vector3.Cross(N, T));
                    return sampleLocal.X * T + sampleLocal.Y * B + sampleLocal.Z * N;
                }
                else
                {
                    Vector3 helper = MathF.Abs(N.X) > 0.9f ? new Vector3(0, 1, 0) : new Vector3(1, 0, 0);
                    Vector3 T2 = Vector3.Normalize(Vector3.Cross(helper, N));
                    Vector3 B2 = Vector3.Normalize(Vector3.Cross(N, T2));
                    return sampleLocal.X * T2 + sampleLocal.Y * B2 + sampleLocal.Z * N;
                }
            }
        }

        /// <summary>
        /// Deletes a GDI object by pointer, used after converting a System.Drawing.Bitmap to a WPF BitmapSource.
        /// </summary>
        [DllImport("gdi32.dll")]
        private static extern bool DeleteObject(nint hObject);

        #endregion
    }
}
