using System;

namespace KinemaLibCs
{
  public struct Vec3
  {
    public double x, y, z;
    bool isDerivative;

    const double IdentDist = 1.0E-04;
    const double IdentDir = 1.0E-03;

    const double Pi = 3.141592653589793238462643383279502884197169;
    const double Pi2 = 2.0 * Pi;

    public Vec3(double x, double y, double z)
    {
      this.x = x; this.y = y; this.z = z;
      isDerivative = false;
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

    readonly double DistTo3(Vec3 v)
    {
      Vec3 lv = new(v.x - x, v.y - y, v.z - z);

      return lv.Len3();
    }
    readonly double SqDistTo3(Vec3 v) // Square of distance
    {
      Vec3 lv = new(v.x - x, v.y - y, v.z - z);

      return lv.x * lv.x + lv.y * lv.y + lv.z * lv.z; ;
    }

    readonly double AngleTo3(Vec3 v)
    {
      Vec3 out = outer(v);

      double lsq1 = sqr(x) + sqr(y) + sqr(z);
      double lsq2 = sqr(v.x) + sqr(v.y) + sqr(v.z);

      double lp = lsq1 * lsq2;
      double lo = sqr(out.x) + sqr(out.y) + sqr(out.z);

      if (lp < 1e-30 || lo < 1e-30 * lp) {
        if (operator*(v) >= 0.0) return 0.0;
    else return Vec2::Pi;
      }
      else if (lo >= lp) return Vec2::Pi / 2.0;
      else {
        double a = asin(sqrt(lo / lp));
        if (operator*(v) < 0.0) a = Vec2::Pi - a;
        if (a > Vec2::Pi) a -= Vec2::Pi;
        if (a < 0.0) a += Vec2::Pi;

        return a;
      }
    }

    public static double operator *(Vec3 v1, Vec3 v2) // Inner product
    {
      return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    public readonly Vec3 outer(Vec3 b)
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

    public override bool Equals(Object obj) { return true; }
    public override int GetHashCode() { return 0; }

    public static bool operator ==(Vec3 v1, Vec3 v2)
    {
      return v1.DistTo3(v2) <= IdentDist;
    }

    public static bool operator !=(Vec3 v1, Vec3 v2)
    {
      return v1.DistTo3(v2) > IdentDist;
    }

    void transform3(Trf3 trf)
    {
      double nx = trf.m[0,0] * x + trf.m[0,1] * y + trf.m[0,2] * z;
      double ny = trf.m[1,0] * x + trf.m[1,1] * y + trf.m[1,2] * z;
      z = trf.m[2,0] * x + trf.m[2,1] * y + trf.m[2,2] * z;

      x = nx; y = ny;

      if (!isDerivative) {
        x += trf.m[0,3];
        y += trf.m[1,3];
        z += trf.m[2,3];

        isDerivative = trf.isDerivative;
      }
    }
  }
}
