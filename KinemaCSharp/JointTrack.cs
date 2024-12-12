using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointTrack : AbstractJoint
  {
    public JointTrack(Grip grp, string name, ArcLinTrack trk, double wheelRad) : base(grp, name)
    {
      cppJoint = JointTrackNew(grp.cppGrip, name, trk.track, wheelRad);
    }

    public double GetWheelRad()
    {
      return GetWheelRadAbstractTrack(cppJoint);
    }

    public void SetWheelRad(double newRad)
    {
      SetWheelRadAbstractTrack(cppJoint, newRad);
    }

    //AbstractTrack getTrack()
    //{
    //  GetTrackAbstractTrack(cppJoint, out AbstractTrack trk);
    //}

    //void replaceTrack(AbstractTrack newTrk)
    //{

    //}

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointTrackNew(IntPtr cppGrip, string name, IntPtr cppTrk, double wheelRad);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static double GetWheelRadAbstractTrack(IntPtr cppJoint);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetWheelRadAbstractTrack(IntPtr cppJoint, double newRad);
  }
}