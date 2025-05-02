using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  [StructLayoutAttribute(LayoutKind.Sequential)]
  public struct Vec33 {
    double x; double y; double z;

    public Vec33(double xx, double yy, double zz) { x = xx; y = yy; z = zz; }
  }

  public partial class ArcLinTrack
  {
    public static readonly Vec33[] LeftTrackData =  [ new(1, 1, 1), new(2,2,2) ];

    public static readonly Vec33[] RightTrackData = [ new(3, 1, 1), new(4,2,2) ];

  }
}
