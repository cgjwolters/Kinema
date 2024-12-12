using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointRevSlide : AbstractJoint
  {
    public JointRevSlide(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointRevSlideNew(ref grp, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointRevSlideNew(ref Grip grp, string name);
  }
}
