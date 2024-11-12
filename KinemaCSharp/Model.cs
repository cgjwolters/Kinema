using System.Data;
using System.Runtime;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Model
  {
    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static IntPtr NewModel(string name);

    private readonly IntPtr cppModel;
    Model(string name)
    {
      try {
        cppModel = NewModel(name);
      }
      catch (Exception e) {
        Console.WriteLine(e.Message);
      }
    }

    public string Name
    {
      get
      {
        return "Clemens";
      }

      set
      {

      }
    } 
 
    bool DefineModel(ArcLinTrack leftTrk, ArcLinTrack rightTrk)
    {
      return false;
    }
    public static void Main()
    {
      // var dllDirectory = @"C:\Users\Clemens\Documents\Projects\KinemaLibCs\bin\x64";
      var dllDirectory = @"C:\Users\Clemens\Documents\Projects\Kinema\lib\1.0";
      Environment.SetEnvironmentVariable("PATH", Environment.GetEnvironmentVariable("PATH") + ";" + dllDirectory);

      ArcLinTrack leftTrk = new (true, 0.1413);

      bool ok = leftTrk.LoadTrackData("3d_rail_left_a1");

      if (!ok)  {
          
      }

      ArcLinTrack rightTrk = new (true, 0.1413);
      ok = rightTrk.LoadTrackData("3d_rail_right_a1");

      if (!ok) {

      }

      leftTrk.SetCoTrack(rightTrk, true, 3.0);
      rightTrk.SetCoTrack(leftTrk, true, 3.0);

      Model carrierModel = new ("Clemens");

      if (!carrierModel.DefineModel(leftTrk, rightTrk)) return;
    }
  }
}
