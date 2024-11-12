namespace KinemaLibCs
{

  public struct Trf3
  {
    internal double[,] m = new double[3, 4];

    public bool isDerivative;

    public void Init()
    {
      m[0, 0] = 1.0; m[0, 1] = 0.0; m[0, 2] = 0.0;
      m[1, 0] = 0.0; m[1, 1] = 1.0; m[1, 2] = 0.0;

      isDerivative = false;
    }

    public void Zero()                           // All elements to zero
    {
      m[0, 0] = 0.0; m[0, 1] = 0.0; m[0, 2] = 0.0;
      m[1, 0] = 0.0; m[1, 1] = 0.0; m[1, 2] = 0.0;

      isDerivative = false;
    }

    public Trf3() { Init(); }                      // Initialize to unitmat

    public Trf3(Trf3 cp)
    {
      this = cp;
    }

    public Trf3(double m00, double m01, double m02, double m03,
                double m10, double m11, double m12, double m13,
                double m20, double m21, double m22, double m23)
    {
      m[0,0] = m00; m[0,1] = m01; m[0,2] = m02; m[0,3] = m03;
      m[1,0] = m10; m[1,1] = m11; m[1,2] = m12; m[1,3] = m13;
      m[2,0] = m20; m[2,1] = m21; m[2,2] = m22; m[2,3] = m23;

      isDerivative = false;
    }

    public Trf3(ref readonly Vec3 org, ref readonly Vec3 zDir, ref readonly Vec3 xDir)
    {
      Vec3 lz = zDir;
      lz.UnitLen3();

      Vec3 ly = lz.Outer(xDir);
      ly.UnitLen3();

      Vec3 lx = ly.Outer(lz);

      m[0, 0] = lx.x; m[0, 1] = lx.y; m[0, 2] = lx.z;
      m[1, 0] = ly.x; m[1, 1] = ly.y; m[1, 2] = ly.z;
      m[2, 0] = lz.x; m[2, 1] = lz.y; m[2, 2] = lz.z;

      m[0, 3] = -lx.x * org.x - lx.y * org.y - lx.z * org.z;
      m[1, 3] = -ly.x * org.x - ly.y * org.y - ly.z * org.z;
      m[2, 3] = -lz.x * org.x - lz.y * org.y - lz.z * org.z;

      isDerivative = false;
    }

    public readonly double Determinant()
    {
      return m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) -
             m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) +
             m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
    }

    public readonly double ScaleX()
    {
      return Math.Sqrt(Vec3.Sqr(m[0, 0]) + Vec3.Sqr(m[0, 1]) + Vec3.Sqr(m[0, 2]));
    }

    public readonly double ScaleY()
    {
      return Math.Sqrt(Vec3.Sqr(m[1, 0]) + Vec3.Sqr(m[1, 1]) + Vec3.Sqr(m[1, 2]));

    }

    public readonly double ScaleZ() {
      return Math.Sqrt(Vec3.Sqr(m[2, 0]) + Vec3.Sqr(m[2, 1]) + Vec3.Sqr(m[2, 2]));
    }

    public readonly bool Mirrors() { return Determinant() < 0.0; } // Does it mirror?

    private static void swap_3(double [,]mt, int i1, int i2)
    {
      if (i1 == i2) return;

      for (int j = 0; j < 4; j++) {
        double h = mt[i1,j]; mt[i1,j] = mt[i2,j]; mt[i2,j] = h;
      }
    }

    public void Mirror(ref readonly Vec3 org, ref readonly Vec3 mirrorAxis) // Mirror along
                                                                     // axis
    {
      Vec3 xDir = new(1, 0, 0), yDir = new(0, 1, 0);

      if (Math.Abs(mirrorAxis * yDir) < Math.Abs(mirrorAxis * xDir)) xDir = yDir;

      Trf3 trf = new(in org, in mirrorAxis, in xDir);

      Trf3 mirr = new(); mirr.m[2, 2] = -1.0;

      trf.InvertInto(this);

      this *= mirr;
      this *= trf;

      isDerivative = false;
    }

    public bool Invert() { return InvertInto(this); } // Invert transform

    public readonly bool InvertInto(Trf3 invMat) // Invert transform
    {
      return false;
    }

    public static Trf3 operator *(Trf3 trf, double fact)            // Scaling
    {
      Trf3 lmt = new(trf);

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) lmt.m[i,j] *= fact;
      }

      return lmt;
    }

    public static Trf3 operator * (Trf3 trf, Trf3 trf2)         // Post multiply with mt
    {
      Trf3 lmt = new(trf);

      int i, j, k;
      double [,]cpm = new double[3,4];

      // Copy trf.m to cpm;

      for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) cpm[i,j] = trf.m[i,j];
      }

      for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
          double sum = 0.0;
          for (k = 0; k < 3; k++) sum += cpm[i,k] * trf2.m[k,j];

          lmt.m[i,j] = sum;
        }

        if (!trf2.isDerivative) lmt.m[i,3] += cpm[i,3];
      }

      if (!trf.isDerivative) lmt.isDerivative = trf2.isDerivative;

      return lmt;
    }
    public Trf3 PreMultWith(ref readonly Trf3 mt)          // Pre multiply with mt
    {
      double [,]cpm = new double[3,4];

      // Copy this.m to cpm;

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) cpm[i,j] = m[i,j];
      }

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
          double sum = 0.0;
          for (int k = 0; k < 3; k++) sum += mt.m[i,k] * cpm[k,j];

          m[i,j] = sum;
        }

        if (!isDerivative) m[i,3] += mt.m[i,3];
      }

      if (!isDerivative) isDerivative = mt.isDerivative;

      return this;
    }

    public static Vec3 operator *(Trf3 trf, Vec3 p)       // Transform Vec3
    {
      Vec3 v;

      v.x = trf.m[0,0] * p.x + trf.m[0,1] * p.y + trf.m[0,2] * p.z;
      v.y = trf.m[1,0] * p.x + trf.m[1,1] * p.y + trf.m[1,2] * p.z;
      v.z = trf.m[2,0] * p.x + trf.m[2,1] * p.y + trf.m[2,2] * p.z;

      if (!p.isDerivative) {
        v.x += trf.m[0,3];
        v.y += trf.m[1,3];
        v.z += trf.m[2,3];
      }

      v.isDerivative = trf.isDerivative || p.isDerivative;

      return v;

    }

    indexer
    //double operator()(int ix, int iy);   // Return matrix element
    //double& operator() (int ix, int iy);         // Return matrix el. ref.

  }
}