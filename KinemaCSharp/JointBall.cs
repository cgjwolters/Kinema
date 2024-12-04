using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace KinemaLibCs
{
  public class JointBall : AbstractJoint
  {
    public JointBall(Grip grp, string name)
    {
      cppJoint = JointBallNew(ref grp, name);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr JointBallNew(ref Grip grp, string name);
  }
}
