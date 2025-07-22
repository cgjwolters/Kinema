using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointBall2Slide : AbstractJoint
  {
    public JointBall2Slide(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointBall2SlideNew(grp.cppGrip, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointBall2SlideNew(IntPtr grp, string name);
  }
}
