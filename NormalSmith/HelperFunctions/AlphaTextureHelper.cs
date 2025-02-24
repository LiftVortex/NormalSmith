using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Numerics;
using System.Runtime.InteropServices;

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
        public static void ClearAlphaTexture()
        {
            lock (alphaLock)
            {
                alphaBitmap?.Dispose();
                alphaBitmap = null;
                cachedAlphaData = null;
                cachedAlphaWidth = 0;
                cachedAlphaHeight = 0;
            }
        }


        /// <summary>
        /// Loads an alpha texture from the given file path and caches its pixel data.
        /// </summary>
        public static void LoadAlphaTexture(string filePath)
        {
            lock (alphaLock)
            {
                alphaBitmap = new Bitmap(filePath);
                CacheAlphaData();
            }
        }

        /// <summary>
        /// Caches the pixel data of the loaded alpha texture.
        /// </summary>
        private static void CacheAlphaData()
        {
            if (alphaBitmap == null)
                return;

            cachedAlphaWidth = alphaBitmap.Width;
            cachedAlphaHeight = alphaBitmap.Height;
            var rect = new Rectangle(0, 0, cachedAlphaWidth, cachedAlphaHeight);
            BitmapData bmpData = alphaBitmap.LockBits(rect, ImageLockMode.ReadOnly, alphaBitmap.PixelFormat);

            // Assuming 32bppArgb for simplicity.
            int byteCount = Math.Abs(bmpData.Stride) * cachedAlphaHeight;
            cachedAlphaData = new byte[byteCount];
            Marshal.Copy(bmpData.Scan0, cachedAlphaData, 0, byteCount);
            alphaBitmap.UnlockBits(bmpData);
        }

        /// <summary>
        /// Samples the alpha value at a given UV coordinate.
        /// Returns 1.0f if no alpha texture is loaded.
        /// </summary>
        public static float SampleAlpha(Vector2 uv)
        {
            if (cachedAlphaData == null)
                return 1.0f;

            int x = (int)(uv.X * (cachedAlphaWidth - 1));
            int y = (int)((1 - uv.Y) * (cachedAlphaHeight - 1));
            x = Math.Clamp(x, 0, cachedAlphaWidth - 1);
            y = Math.Clamp(y, 0, cachedAlphaHeight - 1);

            // For 32bppArgb: byte order is [B, G, R, A]
            int stride = cachedAlphaWidth * 4;
            int index = y * stride + x * 4;
            byte a = cachedAlphaData[index + 3];
            return a / 255f;
        }
    }
}
