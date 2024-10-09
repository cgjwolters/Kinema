using System.Data;
using System.Runtime;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Model
  {
    [DllImport("C:\\Users\\Clemens\\Documents\\Projects\\Kinema\\x64\\Debug\\KinemaCInterface.dll",CharSet=CharSet.Unicode)]
      public extern static IntPtr NewModel(string name);


    private readonly IntPtr cppModel;
    Model(string name)
    {
      cppModel = NewModel(name);
    }

    bool defineModel(ArcLinTrack leftTrk, ArcLinTrack rightTrk)
    {
      return false;
    }
    public static void Main()
    {
      ArcLinTrack leftTrk = new ArcLinTrack();
      bool ok = leftTrk.LoadTrackData("3d_rail_left_a1");

      if (!ok)  {
          
      }

      ArcLinTrack rightTrk = new ArcLinTrack();
      ok = rightTrk.LoadTrackData("3d_rail_right_a1");

      if (!ok) {

      }

      leftTrk.SetCoTrack(rightTrk, true, 3.0);
      rightTrk.SetCoTrack(leftTrk, true, 3.0);

      Model carrierModel = new Model("Clemens");

      if (!carrierModel.defineModel(leftTrk, rightTrk)) return;
    }

  }
}
