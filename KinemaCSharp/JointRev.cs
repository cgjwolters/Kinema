using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointRev : AbstractJoint
  {
    public JointRev(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointRevNew(grp.cppGrip, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointRevNew(IntPtr grp, string name);

  }
}
