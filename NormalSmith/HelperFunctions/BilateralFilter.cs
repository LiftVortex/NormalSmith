using System;
using System.Drawing;

namespace NormalSmith.HelperFunctions
{
    public static class BilateralFilter
    {
        /// <summary>
        /// I've increased the Anti-Aliasing quality, i prefer that look to the BilateralFilter.
        /// Applies a bilateral filter to the input bitmap.
        /// </summary>
        /// <param name="input">The input bitmap (e.g. your baked result).</param>
        /// <param name="radius">The radius (in pixels) of the filter window.</param>
        /// <param name="sigmaSpatial">The sigma for spatial (distance) weighting.</param>
        /// <param name="sigmaRange">The sigma for range (color difference) weighting.</param>
        /// <returns>A new Bitmap with the bilateral filter applied.</returns>
        public static Bitmap ApplyBilateralFilter(Bitmap input, int radius, double sigmaSpatial, double sigmaRange)
        {
            int width = input.Width;
            int height = input.Height;
            Bitmap output = new Bitmap(width, height, input.PixelFormat);

            double twoSigmaSpatial2 = 2 * sigmaSpatial * sigmaSpatial;
            double twoSigmaRange2 = 2 * sigmaRange * sigmaRange;

            // Loop over every pixel in the image.
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    double sumR = 0, sumG = 0, sumB = 0, sumA = 0;
                    double weightSum = 0;

                    Color center = input.GetPixel(x, y);

                    // Loop over the filter window.
                    for (int dy = -radius; dy <= radius; dy++)
                    {
                        int ny = y + dy;
                        if (ny < 0 || ny >= height)
                            continue;

                        for (int dx = -radius; dx <= radius; dx++)
                        {
                            int nx = x + dx;
                            if (nx < 0 || nx >= width)
                                continue;

                            Color neighbor = input.GetPixel(nx, ny);

                            // Spatial weighting: based on the distance from the center pixel.
                            double spatialDist2 = dx * dx + dy * dy;
                            double spatialWeight = Math.Exp(-spatialDist2 / twoSigmaSpatial2);

                            // Range weighting: based on the difference in color (including alpha).
                            double dr = center.R - neighbor.R;
                            double dg = center.G - neighbor.G;
                            double db = center.B - neighbor.B;
                            double da = center.A - neighbor.A;
                            double rangeDist2 = dr * dr + dg * dg + db * db + da * da;
                            double rangeWeight = Math.Exp(-rangeDist2 / twoSigmaRange2);

                            double weight = spatialWeight * rangeWeight;

                            sumR += neighbor.R * weight;
                            sumG += neighbor.G * weight;
                            sumB += neighbor.B * weight;
                            sumA += neighbor.A * weight;
                            weightSum += weight;
                        }
                    }

                    int newR = (int)Math.Round(sumR / weightSum);
                    int newG = (int)Math.Round(sumG / weightSum);
                    int newB = (int)Math.Round(sumB / weightSum);
                    int newA = (int)Math.Round(sumA / weightSum);

                    Color newColor = Color.FromArgb(newA, newR, newG, newB);
                    output.SetPixel(x, y, newColor);
                }
            }

            return output;
        }
    }
}
