using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointRev : AbstractJoint
  {
    public JointRev(Grip grp, string name)
    {
      cppJoint = JointRevNew(ref grp, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointRevNew(ref Grip grp, string name);

  }
}
