using System;
using System.IO;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class ArcLinTrack
  {
    private readonly IntPtr track;

    [DllImport("KinemaCInterface.dll", CharSet = CharSet.Unicode)]
    // extern public static IntPtr NewArcLinTrack(bool trkClosed = false, double trackPipeDiameter = 0.0);
    extern public static IntPtr NewArcLinTrack(bool trkClosed = false, double trackPipeDiameter = 0.0);

    public ArcLinTrack(bool trkClosed = false, double trackPipeDiameter = 0.0) {
      track = (IntPtr)NewArcLinTrack(trkClosed, trackPipeDiameter);
    }

    public bool LoadTrackData(string trackFile)
    {
      if (!File.Exists(trackFile)) {
      }

      return true;
    }
    public void SetCoTrack(ArcLinTrack coTrk, bool reverseDir, double maxSDiff)
    {

    }
  }
}

