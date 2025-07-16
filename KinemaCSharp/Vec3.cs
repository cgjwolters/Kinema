using System;
using System.Reflection.Metadata.Ecma335;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public struct Vec3
  {
    public double x, y, z;
    public bool isDerivative = false;

    public const double IdentDist = 1.0E-04;
    public const double IdentDir = 1.0E-03;

    public const double Pi = 3.141592653589793238462643383279502884197169;
    public const double Pi2 = 2.0 * Pi;

    public const double NumAccuracy = 1.0e-16;
    public const double InchInMm = 25.4;

    public static double Sqr(double v) { return v * v; }

    public Vec3(double x, double y, double z)
    {
      this.x = x; this.y = y; this.z = z;
      isDerivative = false;
    }

    public Vec3(Vec3 cp)
    {
      this = cp;
    }

    public double UnitLen3()        // Make unitlength, return old length
    {
      double ln = Len3();

      if (ln > 0.0) {
        x /= ln; y /= ln; z /= ln;
      }

      return ln;
    }

    public readonly double Len3() { return Math.Sqrt(x * x + y * y + z * z); }
    public readonly double LenSq3() { return x * x + y * y + z * z; }

    public double Len3(double newLen)    // Set new length return old length;
    {
      double oldLen = Len3();

      if (oldLen > 0.0) {
        double fact = newLen / oldLen;
        x *= fact; y *= fact; z *= fact;
      }

      return oldLen;
    }

    public readonly double DistTo3(Vec3 v)
    {
      Vec3 lv = new(v.x - x, v.y - y, v.z - z);

      return lv.Len3();
    }
    public readonly double SqDistTo3(Vec3 v) // Square of distance
    {
      Vec3 lv = new(v.x - x, v.y - y, v.z - z);

      return lv.x * lv.x + lv.y * lv.y + lv.z * lv.z; ;
    }

    public readonly double AngleTo3(Vec3 v)
    {
      Vec3 lvout = Outer(v);

      double lsq1 = Sqr(x) + Sqr(y) + Sqr(z);
      double lsq2 = Sqr(v.x) + Sqr(v.y) + Sqr(v.z);

      double lp = lsq1 * lsq2;
      double lo = Sqr(lvout.x) + Sqr(lvout.y) + Sqr(lvout.z);

      if (lp < 1e-30 || lo < 1e-30 * lp) {
        if (this * v >= 0.0) return 0.0;
        else return Pi;
      }
      else if (lo >= lp) return Pi / 2.0;
      else {
        double a = Math.Asin(Math.Sqrt(lo / lp));
        if (this * v < 0.0) a = Pi - a;
        if (a > Pi) a -= Pi;
        if (a < 0.0) a += Pi;

        return a;
      }
    }

    public static double operator *(Vec3 v1, Vec3 v2) // Inner product
    {
      return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    public readonly Vec3 Outer(Vec3 b)
    {
      return new(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    } // Outer product (*this x b)

    public static Vec3 operator *(Vec3 v, double fact)
    {
      Vec3 lv = new(v.x, v.y, v.z);

      lv.x *= fact; lv.y *= fact; lv.z *= fact;

      return lv;
    }
    public static Vec3 operator /(Vec3 v, double fact)
    {
      Vec3 lv = new(v.x, v.y, v.z);

      lv.x /= fact; lv.y /= fact; lv.z /= fact;

      return lv;
    }

    public static Vec3 operator +(Vec3 v1, Vec3 v2)
    {
      return new(v1.x + v2.x, v1.y + v2.y, v1.z + v2.y);
    }

    public static Vec3 operator -(Vec3 v1, Vec3 v2)
    {
      return new(v1.x - v2.x, v1.y - v2.y, v1.z - v2.y);
    }

    public override readonly bool Equals(Object? obj)
    {
      if (obj is null) return false;
      return DistTo3((Vec3)obj) <= IdentDist;
    }

    public override readonly int GetHashCode() { return 0; }

    public static bool operator ==(Vec3 v1, Vec3 v2)
    {
      return v1.DistTo3(v2) <= IdentDist;
    }

    public static bool operator !=(Vec3 v1, Vec3 v2)
    {
      return v1.DistTo3(v2) > IdentDist;
    }

    //public void Transform3(Trf3 trf)
    //{
    //  double nx = trf.m[0,0] * x + trf.m[0,1] * y + trf.m[0,2] * z;
    //  double ny = trf.m[1,0] * x + trf.m[1,1] * y + trf.m[1,2] * z;
    //  z = trf.m[2,0] * x + trf.m[2,1] * y + trf.m[2,2] * z;

    //  x = nx; y = ny;

    //  if (!isDerivative) {
    //    x += trf.m[0,3];
    //    y += trf.m[1,3];
    //    z += trf.m[2,3];

    //    isDerivative = trf.isDerivative;
    //  }
    //}
  }
}
