using System.Data;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  internal class Body
  {
    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static IntPtr NewBody(Model mdl, string name);

    private readonly IntPtr cppBody;

    Body(Model mdl, string name) {
      cppBody = NewBody(mdl, name);
    }
  }
}
