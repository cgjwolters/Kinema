using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointBallSlide : AbstractJoint
  {
    public JointBallSlide(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointBallSlideNew(ref grp, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointBallSlideNew(ref Grip grp, string name);
  }
}
