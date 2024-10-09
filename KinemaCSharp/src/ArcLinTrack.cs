using System;
using System.IO;

public class ArcLinTrack
{
  public ArcLinTrack() {}

  public bool LoadTrackData(string trackFile)
  {
    if (!File.Exists(trackFile))
    {
    }

    return true;
  }
  public void SetCoTrack(ArcLinTrack coTrk, bool reverseDir, double maxSDiff)
  {

  }

}

