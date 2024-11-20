using KinemaLibCs;
using System;
using System.IO;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public partial class ArcLinTrack
  {
    private readonly IntPtr track;

    // Import Section

    [DllImport("KinemaLib.dll")]
    extern private static IntPtr ArcLinTrackNew(bool trkClosed = false, double trackPipeDiameter = 0.0);

    [DllImport("KinemaLib.dll")]
    extern private static IntPtr ArcLinTrackSetCoTrack(IntPtr track, IntPtr coTrack, bool trkClosed = false, double trackPipeDiameter = 0.0);

    [DllImport("KinemaLib.dll")]
    extern private static int ArcLinTrackGetSize(IntPtr track);

    [DllImport("KinemaLib.dll")]
    extern private static double ArcLinTrackGetMaxS(IntPtr track);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackClear(IntPtr track);

    [DllImport("KinemaLib.dll")]
    extern private static bool ArcLinTrackIsClosed(IntPtr track);

    [DllImport("KinemaLib.dll")]
    extern private static bool ArcLinTrackSetClosed(IntPtr track, bool closed);

    [DllImport("KinemaLib.dll")]
    extern private static double ArcLinTrackGetPipeRadius(IntPtr track);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackSetPipeRadius(IntPtr track, double r);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackGetPoint(IntPtr track, int idx, out Vec3 v);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackSetPoint(IntPtr track, int idx, Vec3 v);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackCalcCentroid(IntPtr track, out Vec3 centroid);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackTranslate(IntPtr track, Vec3 offset);

    [DllImport("KinemaLib.dll")]
    extern private static double ArcLinTrackGetLength(IntPtr track);

    // End Import Section

    public ArcLinTrack(bool trkClosed = false, double trackPipeDiameter = 0.0) {
        track = (IntPtr)ArcLinTrackNew(trkClosed, trackPipeDiameter);
    }

    public bool LoadTrackData(string trackFile)
    {
      if (!File.Exists(trackFile)) {
      }

      return true;
    }

    public void SetTrack(List<Vec3> ptList)
    {
      Clear();

      //void setTrack(const Ino::Vec3* ptLst, int ptSz, bool trackClosed = true,
      //                                         double trackPipeDiameter = 0.0);
    }

    public void SetCoTrack(ref readonly ArcLinTrack coTrack, bool reverseDir, double maxSDiff)
    {
      ArcLinTrackSetCoTrack(track, coTrack.track, reverseDir, maxSDiff);
    }

    public int Size
    {
      get { return ArcLinTrackGetSize(track); }
    }
    
    public bool IsClosed
    {
      get { return ArcLinTrackIsClosed(track); }
      set { ArcLinTrackSetClosed(track,value); }
    }

    public double PipeRadius
    {
      get { return ArcLinTrackGetPipeRadius(track);}
      set { ArcLinTrackSetPipeRadius(track, value); }
    }
    public double PipeDiameter
    {
      get { return PipeRadius * 2.0; }
      set { PipeRadius = value / 2.0; }
    }

    public void Clear()
    {
      ArcLinTrackClear(track);
    }

    public Vec3 GetPoint(int idx)
    {
      if (idx < 0 || idx >= Size) throw new IndexOutOfRangeException();
      Vec3 v; ArcLinTrackGetPoint(track, idx, out v); return v;
    }

    public void GetPoint(int idx, Vec3 v)
    {
      if (idx < 0 || idx >= Size) throw new IndexOutOfRangeException();
      ArcLinTrackSetPoint(track, idx, v);
    }

    public Vec3 CalcCentroid()
    {
      Vec3 c; ArcLinTrackCalcCentroid(track, out c);

      return c;
    }

    public void Translate(Vec3 offset)
    {
      ArcLinTrackTranslate(track, offset);
    }

    public double Length {
      get { return ArcLinTrackGetLength(track); }
    }

    public double MaxS {
      get { return ArcLinTrackGetMaxS(track); }
    }

  double getPipeDiameter() const { return trackPipeRadius * 2.0; }

  void setPipeRadius(double radius) { trackPipeRadius = radius; }
void setPipeDiameter(double diameter) { trackPipeRadius = diameter / 2.0; }

virtual void getPoint(double at_s, Ino::Vec3& p) const;
virtual void getPointAndDir(double at_s, Ino::Vec3& pnt, Ino::Vec3& dir) const;
virtual void getDir(double at_s, Ino::Vec3& v) const;
virtual void getAcc(double at_s, Ino::Vec3& acc) const;
virtual void getJerk(double at_s, Ino::Vec3& jerk) const;

virtual void getXDir(double at_s, Ino::Vec3& x, Ino::Vec3& xDer) const;

double findPoint(const Ino::Vec3& p) const;
double findPoint(const Ino::Vec3& p, Ino::Vec3& trkPt) const;
double findPoint(const Ino::Vec3& p, double minS, double maxS,
                                              Ino::Vec3& trkPt) const;

  }
}

