using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointBall : AbstractJoint
  {
    public JointBall(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointBallNew(grp.cppGrip, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointBallNew(IntPtr cppGrp, string name);
  }
}
