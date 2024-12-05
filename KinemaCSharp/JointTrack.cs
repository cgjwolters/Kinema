using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointTrack : AbstractJoint
  {
    public JointTrack(Grip grp, string name, ArcLinTrack trk, double wheelRad)
    {
      cppJoint = JointTrackNew(grp.cppGrip, name, trk.track, wheelRad);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointTrackNew(IntPtr cppGrip, string name, IntPtr cppTrk, double wheelRad);
  }
}
