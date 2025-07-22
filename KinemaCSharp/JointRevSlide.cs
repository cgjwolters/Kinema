using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class JointRevSlide : AbstractJoint
  {
    public JointRevSlide(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointRevSlideNew(grp.cppGrip, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointRevSlideNew(IntPtr cppGrp, string name);
  }
}
