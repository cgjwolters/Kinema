using System.Data;
using System.Runtime.InteropServices;

namespace KinemaLib
{
  public class Model
  {
    [DllImport("KinInterface.dll")] public static extern IntPtr newModel();
    Model(System.String name)
    {

    }
  }
}
