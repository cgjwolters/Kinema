using System.IO.IsolatedStorage;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public struct Trf3
  {
    internal IntPtr cppTrf;

    public double[,] GetTrf()
    {
      double[,] mat = new double[3,4];

      for (int i=0; i<3; ++i) {
        for (int j=0; j<4; ++j) {
          mat[i, j] = GetElementTrf3(cppTrf, i, j);
        }
      }
      return mat;
    }
    
    public bool IsDerivative
    {
      get {
        return GetDerivativeTrf3(cppTrf);
      }

      set
      {
        SetDerivativeTrf3(cppTrf, value);
      }
    }

    public Trf3()
    {
      cppTrf = Trf3New();
    }

    public void Init()
    {
      InitTrf3();
    }

    public void Zero()                           // All elements to zero
    {
      ZeroTrf3();
    }
    
    public void SetElement(IntPtr cppTrf, int row, int col, double value)
    {
      SetElementTrf3(cppTrf, row, col, value);
    }

    public Trf3(Trf3 cp)
    {
      this = new Trf3();
      cppTrf = Trf3Copy(cp.cppTrf);
    }

    public Trf3(double m00, double m01, double m02, double m03,
                double m10, double m11, double m12, double m13,
                double m20, double m21, double m22, double m23)
    {
      cppTrf = Trf3New();

      SetElement(cppTrf, 0, 0, m00);
      SetElement(cppTrf, 0, 1, m01);
      SetElement(cppTrf, 0, 2, m02);
      SetElement(cppTrf, 0, 3, m03);

      SetElement(cppTrf, 1, 0, m10);
      SetElement(cppTrf, 1, 1, m11);
      SetElement(cppTrf, 1, 2, m12);
      SetElement(cppTrf, 1, 3, m13);

      SetElement(cppTrf, 2, 0, m20);
      SetElement(cppTrf, 2, 1, m21);
      SetElement(cppTrf, 2, 2, m22);
      SetElement(cppTrf, 2, 3, m23);

      SetDerivativeTrf3(cppTrf, false);
    }

    public Trf3(in Vec3 org, in Vec3 zDir, in Vec3 xDir)
    {
      cppTrf = Trf3New();

      Vec3 lz = zDir; 
      lz.UnitLen3();

      Vec3 ly = lz.Outer(xDir);
      ly.UnitLen3();

      Vec3 lx = ly.Outer(lz);

      double m03 = -lx.x * org.x - lx.y * org.y - lx.z * org.z;
      double m13 = -ly.x * org.x - ly.y * org.y - ly.z * org.z;
      double m23 = -lz.x * org.x - lz.y * org.y - lz.z * org.z;

      this = new Trf3(lx.x, lx.y, lx.z, m03, ly.x, ly.y, ly.z, m13, lz.x, lz.y, lz.z, m23);
    }

    public readonly double Determinant()
    {
      return DeterminantTrf3(cppTrf);
    }

    public double ScaleX()
    {
      double [,]m = GetTrf();

      return Math.Sqrt(Vec3.Sqr(m[0 ,0]) + Vec3.Sqr(m[0, 1]) + Vec3.Sqr(m[0, 2]));
    }

    public double ScaleY()
    {
      double[,] m = GetTrf();

      return Math.Sqrt(Vec3.Sqr(m[1, 0]) + Vec3.Sqr(m[1, 1]) + Vec3.Sqr(m[1, 2]));

    }

    public double ScaleZ() {
      double[,] m = GetTrf();

      return Math.Sqrt(Vec3.Sqr(m[2, 0]) + Vec3.Sqr(m[2, 1]) + Vec3.Sqr(m[2, 2]));
    }

    public readonly bool Mirrors() { return Determinant() < 0.0; } // Does it mirror?

    public void Mirror(ref readonly Vec3 org, ref readonly Vec3 mirrorAxis)
    {
      MirrorTrf3(cppTrf, in org, in mirrorAxis);
    }

    public readonly bool Invert() { return InvertInto(this); } // Invert transform

    public readonly bool InvertInto(Trf3 invMat) // Invert transform
    {
      return InvertIntoTrf3(cppTrf, invMat.cppTrf);
    }

    //public static Trf3 operator *(Trf3 trf, double fact)            // Scaling
    //{
    //  Trf3 lmt = new(trf);

    //  for (int i = 0; i < 3; i++) {
    //    for (int j = 0; j < 4; j++) lmt.m[i,j] *= fact;
    //  }

    //  return lmt;
    //}

    //public static Trf3 operator * (Trf3 trf, Trf3 trf2)         // Post multiply with mt
    //{
    //  Trf3 lmt = new(trf);

    //  int i, j, k;
    //  double [,]cpm = new double[3,4];

    //  // Copy trf.m to cpm;

    //  for (i = 0; i < 3; i++) {
    //    for (j = 0; j < 4; j++) cpm[i,j] = trf.m[i,j];
    //  }

    //  for (i = 0; i < 3; i++) {
    //    for (j = 0; j < 4; j++) {
    //      double sum = 0.0;
    //      for (k = 0; k < 3; k++) sum += cpm[i,k] * trf2.m[k,j];

    //      lmt.m[i,j] = sum;
    //    }

    //    if (!trf2.isDerivative) lmt.m[i,3] += cpm[i,3];
    //  }

    //  if (!trf.isDerivative) lmt.isDerivative = trf2.isDerivative;

    //  return lmt;
    //}
    //public Trf3 PreMultWith(ref readonly Trf3 mt)          // Pre multiply with mt
    //{
    //  double [,]cpm = new double[3,4];

    //  // Copy this.m to cpm;

    //  for (int i = 0; i < 3; i++) {
    //    for (int j = 0; j < 4; j++) cpm[i,j] = m[i,j];
    //  }

    //  for (int i = 0; i < 3; i++) {
    //    for (int j = 0; j < 4; j++) {
    //      double sum = 0.0;
    //      for (int k = 0; k < 3; k++) sum += mt.m[i,k] * cpm[k,j];

    //      m[i,j] = sum;
    //    }

    //    if (!isDerivative) m[i,3] += mt.m[i,3];
    //  }

    //  if (!isDerivative) isDerivative = mt.isDerivative;

    //  return this;
    //}

    //public static Vec3 operator *(Trf3 trf, Vec3 p)       // Transform Vec3
    //{
    //  Vec3 v;

    //  v.x = trf.m[0,0] * p.x + trf.m[0,1] * p.y + trf.m[0,2] * p.z;
    //  v.y = trf.m[1,0] * p.x + trf.m[1,1] * p.y + trf.m[1,2] * p.z;
    //  v.z = trf.m[2,0] * p.x + trf.m[2,1] * p.y + trf.m[2,2] * p.z;

    //  if (!p.isDerivative) {
    //    v.x += trf.m[0,3];
    //    v.y += trf.m[1,3];
    //    v.z += trf.m[2,3];
    //  }

    //  v.isDerivative = trf.isDerivative || p.isDerivative;

    //  return v;

    //}

    //public readonly ref double this[int ix, int iy]   // Return matrix element
    //{
    //  get
    //  {
    //    if (ix < 0 || ix > 2 || iy < 0 || iy > 3) throw new IndexOutOfRangeException();
    //    return ref m[ix, iy];
    //  }
    //}

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr Trf3New();

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr Trf3Copy(IntPtr cppTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetElementTrf3(IntPtr cppTrf, int row, int col, double value);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static double GetElementTrf3(IntPtr cppTrf, int row, int col);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static bool GetDerivativeTrf3(IntPtr cppTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetDerivativeTrf3(IntPtr cppTrf, bool isDer);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void InitTrf3();

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void ZeroTrf3();                          // All elements to zero

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static double DeterminantTrf3(IntPtr cppTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void MirrorTrf3(IntPtr cppTrf, ref readonly Vec3 org, ref readonly Vec3 mirrorAxis); // Mirror along

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static bool InvertTrf3(IntPtr cppTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static bool InvertIntoTrf3(IntPtr cppTrf, IntPtr cppInvTrf);
  }
}