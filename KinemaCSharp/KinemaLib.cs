using System.Data;
using System.Runtime;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public partial class Model
  {
    [DllImport("C:\\Users\\Clemens\\Documents\\Projects\\Kinema\\x64\\Release_Multithread\\KinemaCInterface.dll",CharSet=CharSet.Unicode)]
               public extern static IntPtr NewModel(string name);


    private readonly IntPtr cppModel;
    Model(string name)
    {
       cppModel = NewModel(name);
    }
    public static void Main()
    {
      Model mdl = new Model("Clemens");

      Console.WriteLine("Have new model");
    }
  }
}
