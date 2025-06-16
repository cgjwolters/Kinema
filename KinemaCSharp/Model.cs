using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public partial class Model
  {
    internal readonly IntPtr cppModel;

    // Maps to keep a ref to the various objects,
    // Prevents untimely destruction of these objects.
    internal readonly Dictionary<string, Body> BodyMap = [];
    internal readonly Dictionary<string, Grip> GripMap = [];
    internal readonly Dictionary<string, AbstractJoint> JointMap = [];
    internal readonly Dictionary<string, Probe> ProbeMap = [];

    internal readonly List<Topology> TopoList = [];

    public static string cat(string name, int seq)
    {
      return name + seq;
    }

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
    public List<Topology> GetTopologyList()
    {
      return TopoList;
    }

    public void GetVersion(out char major, out char minor, out char release)
    {
      GetVersionModel(cppModel, out major, out minor, out release);
    }
    public Vec3 GetOffset()
    {
      GetOffsetModel(cppModel, out Vec3 offset);
      return offset;
    }

    public void SetOffset(Vec3 modelOffset)
    {
      SetOffsetModel(cppModel, modelOffset);
    }

    public void ApplyOffset()
    {
      ApplyOffsetModel(cppModel);
    }

    public void Transform(Trf3 trf)
    {
      TransformModel(cppModel, trf);
    }

    public bool BuildTopology()
    {
      return BuildTopologyModel(cppModel);
    }
    public void Clear()
    {
      BodyMap.Clear();
      GripMap.Clear();
      JointMap.Clear();
      ProbeMap.Clear();
      TopoList.Clear();
    }

    public Body GetBody(string name) => BodyMap[name];

    public Grip GetGrip(string name) => GripMap[name];

    public AbstractJoint GetJoint(string name) => JointMap[name];

    public Probe GetProbe(string name) => ProbeMap[name];

    public static void Main()
    {
      // var dllDirectory = @"C:\Users\Clemens\Documents\Projects\KinemaLibCs\bin\x64";
      var dllDirectory = @"C:\Users\Clemens\Documents\Projects\Kinema\lib\1.0";
      Environment.SetEnvironmentVariable("PATH", Environment.GetEnvironmentVariable("PATH") + ";" + dllDirectory);

      ArcLinTrack leftTrk = new();
      var ptList = ArcLinTrack.LeftTrackData;
      for (int i = 0; i < ptList.Length; ++i) leftTrk.AddTrackPoint(ptList[i].x, ptList[i].y, ptList[i].y);
      leftTrk.SetRelations();
      leftTrk.Validate();

      ArcLinTrack rightTrk = new();
      ptList = ArcLinTrack.RightTrackData;
      for (int i = 0; i < ptList.Length; ++i) rightTrk.AddTrackPoint(ptList[i].x, ptList[i].y, ptList[i].y);
      rightTrk.SetRelations();
      rightTrk.Validate();

      leftTrk.SetCoTrack(ref rightTrk, false, 3.0);
      rightTrk.SetCoTrack(ref leftTrk, false, 3.0);

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

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetOffsetModel(IntPtr cppModel, out Vec3 modelOffset);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetOffsetModel(IntPtr cppModel, Vec3 modelOffset);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void ApplyOffsetModel(IntPtr cppModel);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void TransformModel(IntPtr cppModel, Trf3 trf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static bool BuildTopologyModel(IntPtr cppModel);

    // End Interface section
  }
}


