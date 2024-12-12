using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace KinemaLibCs
{
  public class JointSlide : AbstractJoint
  {
    public JointSlide(Grip grp, string name) : base(grp, name)
    {
      cppJoint = JointSlideNew(ref grp, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointSlideNew(ref Grip grp, string name);
  }
}
