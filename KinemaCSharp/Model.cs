using System.Numerics;
using System.Reflection;
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

    public void SetFixedAll(bool fixd)
    {
      SetFixedAllModel(cppModel, fixd);
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

    //--------------------------------------------------------------------------------------

    void AdvanceJoint(string name, double delta)
    {
      AbstractJoint jnt = JointMap[name];

      double val = jnt.GetVal(0);
      jnt.SetVal(0, val + delta);
    }
    void AdvanceModel(double delta)
    {
      for (int i = 0; i < CoachSz; ++i) {
        if (i > 0) AdvanceJoint("JntLeftR"+i, delta);
        AdvanceJoint("JntLeftF" + i, delta);
        AdvanceJoint("JntRightR" + i, delta);
        AdvanceJoint("JntRightF" + i, delta);
      }
    }

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

      carrierModel.PutOnTrack(3.0, leftTrk, rightTrk);

      if (!carrierModel.BuildTopology()) {
        Logger.WriteMsg("Inconsistent model!!!");
        return;
      }

      carrierModel.SetWheelVars(3.0);

      // Dont forget!:
      carrierModel.ResetFixedVars();

      Topology topo = carrierModel.GetTopology(0);

      int iter = 50;
      double []posVec = new double [topo.GetVarSz()];

      double[] varList = new double[1];

      if (!topo.SolvePos(50, 1e-5, 1e-5, varList, ref iter)) return;

      Sequence mdlSeq = topo.NewSequence("ModelSequence");

      double startOffset = 1.4;
      double deltapos = 0.1;
      double trklen = leftTrk.Length;

      int iters = 0;

      double lpos = startOffset;
//      int idx = 1;
      var jntLeftR = carrierModel.GetJoint("JntLeftR0");

      while (lpos > 0.2) {
        jntLeftR.SetVal(0, lpos);

        if (!topo.SolvePos(50, 1e-5, 1e-5, posVec, ref iters)) {
//          Logger.WriteMsg($"Model convergeert niet (back)! (lpos={"0"}",lpos);

          //          ads_printf(L"\nModel convergeert niet (back)! (lpos=%f)\n", lpos);
          break;
        }

        Logger.Write("");
        //        ads_printf(L"%d, Dist =  %.3f (iters:%3d)        \r", idx++, lpos, iters);

        lpos -= deltapos;
        carrierModel.AdvanceModel(-deltapos);
      }

      iters = 50;

      while (lpos < trklen) {
        jntLeftR.SetVal(0, lpos);

        if (!topo.SolvePos(50, 1e-5, 1e-5, posVec, ref iters)) {
          mdlSeq.AddCurrentTopoState();
          Logger.WriteMsg("Inconsistent model!!!");

          //          ads_printf(L"\nModel convergeert niet! (lpos=%f)\n", lpos);
          break;
        }

        mdlSeq.AddCurrentTopoState();
        Logger.WriteMsg("Inconsistent model!!!");
        //        ads_printf(L"%d, Dist =  %.3f (iters:%3d)        \r", idx++, lpos, iters);

        lpos += deltapos;
        carrierModel.AdvanceModel(deltapos);
      }
    }

    // Interface section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr ModelNew(string name);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetVersionModel(IntPtr cppModel, out char major, out char minor, out char release);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetFixedAllModel(IntPtr cppModel, bool fixd);

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

    // Interface section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetCppTrf(IntPtr cppTrf, int i, int j, double val);

    [DllImport("KinemaLib.dll")]
    extern private static IntPtr GetParentBody(IntPtr cppBody, out Body parent);


    // End Interface section
  }
}


