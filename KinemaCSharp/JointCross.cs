using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointCross : AbstractJoint
  {
    public JointCross(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointCrossNew(grp.cppGrip, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointCrossNew(IntPtr cppGrp, string name);
  }
}
