using System;
using System.Drawing;
using System.Numerics;

namespace NormalSmith.HelperFunctions
{
    /// <summary>
    /// Custom delegate for dual-buffer sub-sampling:
    /// We want to pass in (x, y, bary, uvInterp) and get *two* ARGB colors via out parameters.
    /// </summary>
    public delegate void DualSampleFunc(int x, int y, Vector3 bary, PointF uvInterp, out int bentColor, out int occColor);

    public static class AARasterizer
    {
        /// <summary>
        /// Single-buffer 2x2 AA rasterizer:
        /// sampleFunc: (x, y, bary, uv) => returns ARGB int
        /// </summary>
        public static void RasterizeTriangleToBufferAA(
            int[] pixelBuffer,
            int width,
            int height,
            PointF p0,
            PointF p1,
            PointF p2,
            Func<int, int, Vector3, PointF, int> sampleFunc)
        {
            float[] sampleOffsets = { 0.25f, 0.75f };

            float minX = MathF.Min(p0.X, MathF.Min(p1.X, p2.X));
            float minY = MathF.Min(p0.Y, MathF.Min(p1.Y, p2.Y));
            float maxX = MathF.Max(p0.X, MathF.Max(p1.X, p2.X));
            float maxY = MathF.Max(p0.Y, MathF.Max(p1.Y, p2.Y));

            int x0 = Math.Max(0, (int)MathF.Floor(minX));
            int y0 = Math.Max(0, (int)MathF.Floor(minY));
            int x1 = Math.Min(width, (int)MathF.Ceiling(maxX));
            int y1 = Math.Min(height, (int)MathF.Ceiling(maxY));

            float area = EdgeFunction(p0, p1, p2);
            if (MathF.Abs(area) < 1e-8f)
                return; // Degenerate triangle => skip

            for (int y = y0; y < y1; y++)
            {
                for (int x = x0; x < x1; x++)
                {
                    float totalA = 0, totalR = 0, totalG = 0, totalB = 0;
                    int sampleCount = 0;

                    foreach (float sy in sampleOffsets)
                    {
                        foreach (float sx in sampleOffsets)
                        {
                            float sampleX = x + sx;
                            float sampleY = y + sy;

                            float w0 = EdgeFunction(p1, p2, new PointF(sampleX, sampleY));
                            float w1 = EdgeFunction(p2, p0, new PointF(sampleX, sampleY));
                            float w2 = EdgeFunction(p0, p1, new PointF(sampleX, sampleY));

                            if (w0 >= 0 && w1 >= 0 && w2 >= 0)
                            {
                                float invArea = 1f / area;
                                float alpha = w0 * invArea;
                                float beta = w1 * invArea;
                                float gamma = w2 * invArea;

                                Vector3 bary = new Vector3(alpha, beta, gamma);

                                // If you do p0.X = uv0.X*width, p0.Y=(1-uv0.Y)*height,
                                // you might invert that here. For simplicity, do:
                                float u = alpha * p0.X + beta * p1.X + gamma * p2.X;
                                float v = alpha * p0.Y + beta * p1.Y + gamma * p2.Y;

                                float uvX = u / width;
                                float uvY = 1f - (v / height);

                                PointF uvInterp = new PointF(uvX, uvY);

                                int color = sampleFunc(x, y, bary, uvInterp);

                                byte A = (byte)(color >> 24 & 0xFF);
                                byte R = (byte)(color >> 16 & 0xFF);
                                byte G = (byte)(color >> 8 & 0xFF);
                                byte B = (byte)(color & 0xFF);

                                totalA += A;
                                totalR += R;
                                totalG += G;
                                totalB += B;
                                sampleCount++;
                            }
                        }
                    }

                    if (sampleCount > 0)
                    {
                        float invCount = 1f / sampleCount;
                        byte finalA = (byte)Math.Clamp(totalA * invCount, 0, 255);
                        byte finalR = (byte)Math.Clamp(totalR * invCount, 0, 255);
                        byte finalG = (byte)Math.Clamp(totalG * invCount, 0, 255);
                        byte finalB = (byte)Math.Clamp(totalB * invCount, 0, 255);

                        int finalColor = (finalA << 24) | (finalR << 16) | (finalG << 8) | finalB;
                        pixelBuffer[y * width + x] = finalColor;
                    }
                }
            }
        }

        /// <summary>
        /// Dual-buffer 2x2 AA rasterizer:
        /// 
        /// We define a custom delegate 'DualSampleFunc' so we can return two ARGB colors via 'out' params.
        /// sampleAction:
        ///    (x, y, bary, uv, out int bentCol, out int occCol)
        /// </summary>
        public static void RasterizeTriangleToDualBufferAA(
            int[] bentBuffer,
            int[] occBuffer,
            int width,
            int height,
            PointF p0,
            PointF p1,
            PointF p2,
            DualSampleFunc sampleAction)
        {
            float[] sampleOffsets = { 0.25f, 0.75f };

            float minX = MathF.Min(p0.X, MathF.Min(p1.X, p2.X));
            float minY = MathF.Min(p0.Y, MathF.Min(p1.Y, p2.Y));
            float maxX = MathF.Max(p0.X, MathF.Max(p1.X, p2.X));
            float maxY = MathF.Max(p0.Y, MathF.Max(p1.Y, p2.Y));

            int x0 = Math.Max(0, (int)MathF.Floor(minX));
            int y0 = Math.Max(0, (int)MathF.Floor(minY));
            int x1 = Math.Min(width, (int)MathF.Ceiling(maxX));
            int y1 = Math.Min(height, (int)MathF.Ceiling(maxY));

            float area = EdgeFunction(p0, p1, p2);
            if (MathF.Abs(area) < 1e-8f)
                return;

            for (int y = y0; y < y1; y++)
            {
                for (int x = x0; x < x1; x++)
                {
                    // We'll accumulate TWO colors: bent & occ
                    float bentA = 0, bentR = 0, bentG = 0, bentB = 0;
                    float occA = 0, occR = 0, occG = 0, occB = 0;
                    int sampleCount = 0;

                    foreach (float sy in sampleOffsets)
                    {
                        foreach (float sx in sampleOffsets)
                        {
                            float sampleX = x + sx;
                            float sampleY = y + sy;

                            float w0 = EdgeFunction(p1, p2, new PointF(sampleX, sampleY));
                            float w1 = EdgeFunction(p2, p0, new PointF(sampleX, sampleY));
                            float w2 = EdgeFunction(p0, p1, new PointF(sampleX, sampleY));

                            if (w0 >= 0 && w1 >= 0 && w2 >= 0)
                            {
                                float invA = 1f / area;
                                float alpha = w0 * invA;
                                float beta = w1 * invA;
                                float gamma = w2 * invA;

                                Vector3 bary = new Vector3(alpha, beta, gamma);

                                float u = alpha * p0.X + beta * p1.X + gamma * p2.X;
                                float v = alpha * p0.Y + beta * p1.Y + gamma * p2.Y;
                                float uvX = u / width;
                                float uvY = 1f - (v / height);

                                PointF uvInterp = new PointF(uvX, uvY);

                                // Call user function to get two ARGB colors
                                sampleAction(x, y, bary, uvInterp, out int bentCol, out int occCol);

                                // Decompose bentCol
                                byte bA = (byte)(bentCol >> 24 & 0xFF);
                                byte bR = (byte)(bentCol >> 16 & 0xFF);
                                byte bG = (byte)(bentCol >> 8 & 0xFF);
                                byte bB = (byte)(bentCol & 0xFF);

                                bentA += bA;
                                bentR += bR;
                                bentG += bG;
                                bentB += bB;

                                // Decompose occCol
                                byte oA = (byte)(occCol >> 24 & 0xFF);
                                byte oR = (byte)(occCol >> 16 & 0xFF);
                                byte oG = (byte)(occCol >> 8 & 0xFF);
                                byte oB = (byte)(occCol & 0xFF);

                                occA += oA;
                                occR += oR;
                                occG += oG;
                                occB += oB;

                                sampleCount++;
                            }
                        }
                    }

                    if (sampleCount > 0)
                    {
                        float invCount = 1f / sampleCount;

                        // Final bent color
                        byte fbA = (byte)Math.Clamp(bentA * invCount, 0, 255);
                        byte fbR = (byte)Math.Clamp(bentR * invCount, 0, 255);
                        byte fbG = (byte)Math.Clamp(bentG * invCount, 0, 255);
                        byte fbB = (byte)Math.Clamp(bentB * invCount, 0, 255);
                        int finalBent = (fbA << 24) | (fbR << 16) | (fbG << 8) | fbB;

                        // Final occ color
                        byte foA = (byte)Math.Clamp(occA * invCount, 0, 255);
                        byte foR = (byte)Math.Clamp(occR * invCount, 0, 255);
                        byte foG = (byte)Math.Clamp(occG * invCount, 0, 255);
                        byte foB = (byte)Math.Clamp(occB * invCount, 0, 255);
                        int finalOcc = (foA << 24) | (foR << 16) | (foG << 8) | foB;

                        bentBuffer[y * width + x] = finalBent;
                        occBuffer[y * width + x] = finalOcc;
                    }
                }
            }
        }

        private static float EdgeFunction(PointF a, PointF b, PointF c)
        {
            return (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);
        }
    }
}
