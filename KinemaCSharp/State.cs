using System.Runtime.InteropServices;
using System.Globalization;

namespace KinemaLibCs
{
  public class State
  {
    private readonly IntPtr cppState;
    private Sequence seq;

    public State(Sequence seq, int index, long tm, Topology cppTopo)
    {
      cppState = StateNew(seq, index, tm, cppTopo);
      this.seq = seq;
    }

    public State(IntPtr cppState, Sequence seq)
    {
      this.cppState = cppState;
      this.seq = seq;
    }

    public Sequence GetSequence()
    {
      return seq;
    }

    public int GetIdx() {
      return GetIdxState(cppState); ;
    }

    public Int64 GetTm()
    {
      return GetTmState(cppState);
    }

    public bool GetPos(AbstractJoint jnt, int varIdx, out double posVal)
    {

      return GetPosState(cppState, jnt.cppJoint, varIdx, out posVal);
    }

    public bool GetSpeed(AbstractJoint jnt, int varIdx, out double speedVal)
    {
      return GetSpeedState(cppState, jnt, varIdx, out speedVal);
    }

    public bool GetAccel(AbstractJoint jnt, int varIdx, out double accVal)
    {
      return GetAccelState(cppState, jnt, varIdx, out accVal);
    }

    public bool GetJerk(AbstractJoint jnt, int varIdx, out double jerkVal)
    {
      return GetJerkState(cppState, jnt, varIdx, out jerkVal);
    }

    // For all other info call this method and interrogate the topology

    public bool SetTopologyToThis()
    {
      return SetTopologyToThisState(this);
    }

    public bool Write(StreamWriter sw)
    {
      var mdl = GetSequence().GetModel();

      if (mdl == null) return false;

      var keys = new List<string>(mdl.JointMap.Keys);

      for (int i=0; i<keys.Count; ++i) {
        var jnt = mdl.JointMap[keys[i]];

        int sz = jnt.GetVarCnt(false);
        bool fst = true;

        for (int j=0; j<sz; ++j) {
          double val;
          if (!GetPos(jnt, j, out val)) return false;

          String s = val.ToString("G9", CultureInfo.InvariantCulture);
          sw.Write(val.ToString("G9",CultureInfo.InvariantCulture));
          if (!fst) sw.Write(";");
          fst = false;
        }
      }

      sw.WriteLine();

      return true;
    }

    // Interface Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr StateNew(Sequence seq, int index, long tm, Topology cppTopo);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private Sequence GetSequenceState(IntPtr cppState);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int GetIdxState(IntPtr cppState);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private Int64 GetTmState(IntPtr cppState);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetPosState(IntPtr cppState, IntPtr cppJnt, int varIdx, out double posVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetSpeedState(IntPtr cppState, AbstractJoint jnt, int varIdx, out double speedVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetAccelState(IntPtr cppState, AbstractJoint jnt, int varIdx, out double accVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetJerkState(IntPtr cppState, AbstractJoint jnt, int varIdx, out double jerkVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool SetTopologyToThisState(State state);

    // End Interface Section
  }
}


