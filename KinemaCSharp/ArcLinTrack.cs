﻿using KinemaLibCs;
using System;
using System.IO;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public partial class ArcLinTrack
  {
    private readonly IntPtr track;

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

    public Vec3 GetPoint(double at_s)
    {
      Vec3 pnt;
      Vec3 dir;

      ArcLinTrackGetPointAndDir(track, at_s, out pnt, out dir);

      return pnt;
    }

    public void GetPointAndDir(double at_s, out Vec3 pnt, out Vec3 dir)
    {
      ArcLinTrackGetPointAndDir(track, at_s, out pnt, out dir);
    }

    public Vec3 GetDir(double at_s)
    {
      Vec3 pnt;
      Vec3 dir;

      ArcLinTrackGetPointAndDir(track, at_s, out pnt, out dir);

      return dir;
    }

    public Vec3 GetAcc(double at_s)
    {
      Vec3 acc;
      Vec3 jerk;

      ArcLinTrackGetAccAndJerk(track, at_s, out acc, out jerk);

      return acc;
    }

    public Vec3 GetJerk(double at_s)
    {
      Vec3 acc;
      Vec3 jerk;

      ArcLinTrackGetAccAndJerk(track, at_s, out acc, out jerk);

      return jerk;
    }

    public void getXDir(double at_s, out Vec3 x, out Vec3 xDer)
    {
      ArcLinTrackGetXDir(track, at_s, out x, out xDer);
    }

    public double FindPoint(Vec3 p)
    {
      Vec3 trkPt;
      return ArcLinTrackFindPoint(track, p,out trkPt);
    }

    public double FindPoint(Vec3 p, out Vec3 trkPt)
    {
      return ArcLinTrackFindPoint(track, p, out trkPt);
    }

    public double FindPoint(Vec3 p, double minS, double maxS,
                                              out Vec3 trkPt)
    {
      return ArcLinTrackFindPoint(track, p, minS, maxS, out trkPt);
    }

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

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackGetPointAndDir(IntPtr track, double at_s, out Vec3 pnt, out Vec3 dir);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackGetAccAndJerk(IntPtr track, double at_s, out Vec3 acc, out Vec3 jerk);

    [DllImport("KinemaLib.dll")]
    extern private static void ArcLinTrackGetXDir(IntPtr track, double at_s, out Vec3 x, out Vec3 xDir);

    [DllImport("KinemaLib.dll")]
    extern private static double ArcLinTrackFindPoint(IntPtr trrack, Vec3 p, out Vec3 trkPt);

    [DllImport("KinemaLib.dll")]
    extern private static double ArcLinTrackFindPoint(IntPtr trrack, Vec3 p, double minS, double maxS, out Vec3 trkPt);

    // End Import Section
  }
} // namespace

