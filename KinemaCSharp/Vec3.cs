using System;

namespace KinemaLibCs
{
  public struct Vec3
  {
    public double x, y, z;
    bool isDerivative;

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
    
    public double Len3() { return Math.Sqrt(x*x + y*y + z*z); }
    public double LenSq3() { return x * x + y * y + z * z; }

    public double Len3(double newLen)    // Set new length return old length;
    {
      double oldLen = Len3();

      if (oldLen > 0.0) {
        double fact = newLen / oldLen;
        x *= fact; y *= fact; z *= fact;
      }

      return oldLen;
    }

    double DistTo3(Vec3 v)
    {
      Vec3 lv = new(v.x-x,v.y - y,v.z - z);

      return lv.Len3();
    }
    double SqDistTo3(Vec3 v) // Square of distance
    {
      Vec3 lv = new(v.x - x, v.y - y, v.z - z);

      return lv.x*lv.x + lv.y * lv.y + lv.z * lv.z; ;
    }

    double AngleTo3(Vec3 v) { return 0.0; }

    double operator *(Vec3 v) { return x * v.x + y * v.y + z * v.z; }  // Inner product

    Vec3 outer(Vec3 b) { } // Outer product (*this x b)

    Vec3 operator *(double fact)
    {
      Vec3 lv = new(x,y,z);

      lv.x *= fact; lv.y *= fact; lv.z *= fact;

      return lv;
    }
    Vec3 operator /(double fact)
    {
      Vec3 lv = new(x, y, z);

      lv.x /= fact; lv.y /= fact; lv.z /= fact;

      return lv;
    }

    Vec3 operator +(Vec3 v) { }

    Vec3 operator -(Vec3 v) { }

    void operator *= (double fact) { x *= fact; y *= fact; z *= fact; }

    void operator /= (double fact) { x /= fact; y /= fact; z /= fact; }

    void operator += (Vec3 v) { x += v.x; y += v.y; z += v.z; }

    void operator -= (Vec3 v) { x -= v.x; y -= v.y; z -= v.z; }

  public override bool Equals(Object obj) { return true; }
  public override int GetHashCode() { return 0; }

  bool operator ==(const Vec3& v)
  {
    return distTo3(v) <= IdentDist;
  }
  bool operator !=(const Vec3& v)
  {
    return distTo3(v) > IdentDist;
  }

   void transform3(Trf3 trf)
  {

  }
}
