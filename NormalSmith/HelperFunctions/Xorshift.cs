using System;

// Simple fast PRNG - Xorshift
public class Xorshift
{
    private uint x, y, z, w;

    public Xorshift(uint seed = 123456789)
    {
        x = seed;
        y = 362436069;
        z = 521288629;
        w = 88675123;
    }

    // Returns a float in [0..1).
    public float NextFloat()
    {
        uint t = x ^ (x << 11);
        x = y; y = z; z = w;
        w = w ^ (w >> 19) ^ (t ^ (t >> 8));

        // Take lower 24 bits => fraction
        // (can also do / (1 << 24) if you prefer).
        return (w & 0x00FFFFFF) / (float)0x01000000;
    }
}
