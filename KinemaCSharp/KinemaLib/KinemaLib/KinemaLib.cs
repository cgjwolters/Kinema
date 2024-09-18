using System.Data;
using System.Runtime.InteropServices;

namespace KinemaLib
{
  public class Model
  {
    [DllImport("C:\\Users\\Clemens\\Documents\\Projects\\Kinema\\x64\\Debug_Multithread\\KinemaCInterface.dll")] public static extern IntPtr NewModel();

    private readonly IntPtr cppModel;
    Model(string name)
    {
      cppModel = NewModel();
    }

    public static void Main()
    {
      Model mdl = new("Clemens");

      Console.WriteLine("Have new model");
    }
  }
}
