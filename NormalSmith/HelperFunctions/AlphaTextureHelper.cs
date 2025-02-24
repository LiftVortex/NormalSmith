using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using DrawingPixelFormat = System.Drawing.Imaging.PixelFormat;

namespace NormalSmith.HelperFunctions
{
    public static class AlphaTextureHelper
    {
        // Holds the loaded alpha bitmap.
        private static Bitmap alphaBitmap;
        // Lock object for thread safety.
        private static readonly object alphaLock = new object();
        // Cached pixel data.
        private static byte[] cachedAlphaData;
        private static int cachedAlphaWidth, cachedAlphaHeight;
        // Bytes per pixel determined by the image’s pixel format.
        private static int cachedBytesPerPixel;

        /// <summary>
        /// Clears the loaded alpha texture and its cached data.
        /// </summary>
        public static void ClearAlphaTexture()
        {
            lock (alphaLock)
            {
                alphaBitmap?.Dispose();
                alphaBitmap = null;
                cachedAlphaData = null;
                cachedAlphaWidth = 0;
                cachedAlphaHeight = 0;
                cachedBytesPerPixel = 0;
            }
        }

        /// <summary>
        /// Loads an alpha texture from the given file path.
        /// For PNG images the code reads the original file’s pixel format via a PngBitmapDecoder,
        /// which preserves the original format (for example, grayscale "L" images will be kept as Gray8).
        /// For other formats, it falls back to System.Drawing.
        /// </summary>
        public static void LoadAlphaTexture(string filePath)
        {
            lock (alphaLock)
            {
                string ext = Path.GetExtension(filePath).ToLowerInvariant();
                if (ext == ".png")
                {
                    // Use a PNG decoder to load the PNG while preserving its pixel format.
                    BitmapSource bitmapSource = LoadPngBitmapSource(filePath);
                    cachedAlphaWidth = bitmapSource.PixelWidth;
                    cachedAlphaHeight = bitmapSource.PixelHeight;

                    // Determine bytes per pixel from the BitmapSource.Format.
                    // Common cases:
                    // Gray8: 8 bits per pixel (1 byte).
                    // Bgr24: 24 bits per pixel (3 bytes).
                    // Bgra32: 32 bits per pixel (4 bytes).
                    if (bitmapSource.Format == PixelFormats.Gray8)
                    {
                        cachedBytesPerPixel = 1;
                    }
                    else if (bitmapSource.Format == PixelFormats.Bgr24)
                    {
                        cachedBytesPerPixel = 3;
                    }
                    else if (bitmapSource.Format == PixelFormats.Bgra32)
                    {
                        cachedBytesPerPixel = 4;
                    }
                    else
                    {
                        // Fallback: use the bits per pixel value.
                        cachedBytesPerPixel = bitmapSource.Format.BitsPerPixel / 8;
                        if (cachedBytesPerPixel < 1)
                            cachedBytesPerPixel = 4;
                    }

                    // Compute the stride. Note: for simplicity we assume stride equals width * bytesPerPixel.
                    int stride = cachedAlphaWidth * cachedBytesPerPixel;
                    cachedAlphaData = new byte[stride * cachedAlphaHeight];
                    bitmapSource.CopyPixels(cachedAlphaData, stride, 0);
                }
                else
                {
                    // For non-PNG images (like JPEG), assume JPEG is always RGB.
                    if (ext == ".jpg" || ext == ".jpeg")
                    {
                        cachedBytesPerPixel = 3;
                    }
                    else
                    {
                        // Load a temporary Bitmap to check its PixelFormat.
                        using (var tempBitmap = new Bitmap(filePath))
                        {
                            switch (tempBitmap.PixelFormat)
                            {
                                case DrawingPixelFormat.Format32bppArgb:
                                case DrawingPixelFormat.Format32bppPArgb:
                                case DrawingPixelFormat.Format32bppRgb:
                                    cachedBytesPerPixel = 4;
                                    break;
                                case DrawingPixelFormat.Format24bppRgb:
                                    cachedBytesPerPixel = 3;
                                    break;
                                case DrawingPixelFormat.Format8bppIndexed:
                                    cachedBytesPerPixel = 1;
                                    break;
                                default:
                                    cachedBytesPerPixel = 4;
                                    break;
                            }
                        }
                        // Now load the actual Bitmap.
                        alphaBitmap = new Bitmap(filePath);
                        CacheAlphaData();
                    }
                }
            }
        }

        /// <summary>
        /// Loads a PNG image using WPF’s PngBitmapDecoder to preserve its original pixel format.
        /// </summary>
        private static BitmapSource LoadPngBitmapSource(string filePath)
        {
            using (FileStream stream = new FileStream(filePath, FileMode.Open, FileAccess.Read))
            {
                PngBitmapDecoder decoder = new PngBitmapDecoder(
                    stream,
                    BitmapCreateOptions.PreservePixelFormat,
                    BitmapCacheOption.OnLoad);
                return decoder.Frames[0];
            }
        }

        /// <summary>
        /// For non-PNG images, caches the pixel data of the loaded Bitmap.
        /// </summary>
        private static void CacheAlphaData()
        {
            if (alphaBitmap == null)
                return;

            cachedAlphaWidth = alphaBitmap.Width;
            cachedAlphaHeight = alphaBitmap.Height;
            Rectangle rect = new Rectangle(0, 0, cachedAlphaWidth, cachedAlphaHeight);
            BitmapData bmpData = alphaBitmap.LockBits(rect, ImageLockMode.ReadOnly, alphaBitmap.PixelFormat);
            int byteCount = Math.Abs(bmpData.Stride) * cachedAlphaHeight;
            cachedAlphaData = new byte[byteCount];
            Marshal.Copy(bmpData.Scan0, cachedAlphaData, 0, byteCount);
            alphaBitmap.UnlockBits(bmpData);
        }

        /// <summary>
        /// Samples the alpha value at a given UV coordinate.
        /// Returns 1.0f if no alpha texture is loaded.
        /// For 32bpp images the A channel is used;
        /// for 24bpp images a grayscale luminance is computed;
        /// for 8bpp images the single channel is used.
        /// </summary>
        public static float SampleAlpha(Vector2 uv)
        {
            if (cachedAlphaData == null)
                return 1.0f;

            // Compute pixel coordinates based on UV.
            int x = (int)(uv.X * (cachedAlphaWidth - 1));
            int y = (int)((1 - uv.Y) * (cachedAlphaHeight - 1));
            x = Math.Clamp(x, 0, cachedAlphaWidth - 1);
            y = Math.Clamp(y, 0, cachedAlphaHeight - 1);

            // For PNGs loaded via WPF, stride is assumed to be width * bytesPerPixel.
            int stride = cachedAlphaWidth * cachedBytesPerPixel;
            int index = y * stride + x * cachedBytesPerPixel;

            if (cachedBytesPerPixel == 4)
            {
                // For 32bpp images, assume the order is [B, G, R, A].
                byte a = cachedAlphaData[index + 3];
                return a / 255f;
            }
            else if (cachedBytesPerPixel == 3)
            {
                // For 24bpp images, compute a luminance from RGB.
                byte r = cachedAlphaData[index];
                byte g = cachedAlphaData[index + 1];
                byte b = cachedAlphaData[index + 2];
                float luminance = (0.299f * r + 0.587f * g + 0.114f * b);
                return luminance / 255f;
            }
            else if (cachedBytesPerPixel == 1)
            {
                // For 8bpp images, the single channel is used.
                byte val = cachedAlphaData[index];
                return val / 255f;
            }
            else
            {
                return 1.0f;
            }
        }
    }




}
