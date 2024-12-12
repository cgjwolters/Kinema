using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Model
  {
    private readonly IntPtr cppModel;

    // Maps to keep a ref to the various objects,
    // Prevents untimely destruction of these objects.
    internal readonly Dictionary<string, Body> bodyMap = new Dictionary<string, Body>();
    internal readonly Dictionary<string, Grip> gripMap = new Dictionary<string, Grip>();
    internal readonly Dictionary<string, AbstractJoint> jointMap = new Dictionary<string, AbstractJoint>();
    internal readonly Dictionary<string, Probe> probeMap = new Dictionary<string, Probe>();

    public Model(string name)
    {
      try {
        cppModel = ModelNew(name);
      }
      catch (Exception e) {
        Console.WriteLine(e.Message);
      }
    }

    public string Name
    {
      get
      {
        GetNameModel(cppModel, out string name);

        return name;
      }
    }

    public void GetVersion(out char major, out char minor, out char release)
    {
      GetVersionModel(cppModel, out major, out minor, out release);
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

      leftTrk.SetCoTrack(ref rightTrk, true, 3.0);
      rightTrk.SetCoTrack(ref leftTrk, true, 3.0);

      Model carrierModel = new ("Clemens");

      if (!carrierModel.DefineModel(leftTrk, rightTrk)) return;
    }

    // Interface section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr ModelNew(string name);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetVersionModel(IntPtr cppModel, out char major, out char minor, out char release);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetNameModel(IntPtr cppModel, out string name);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void ClearModel(IntPtr cppModel);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static string GetNameModel(IntPtr cppModel);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetNameModel(IntPtr cppModel, string newName);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetTopoModifiedModel(IntPtr cppModel);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetModifiedModel(IntPtr cppModel, bool mod);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static bool IsModifiedModel(IntPtr cppModel);


    // End Interface section
  }
}


