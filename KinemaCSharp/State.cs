using KinemaLibCs;
using System.Net.NetworkInformation;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class State
  {
    private readonly IntPtr cppState;

    public State(Sequence seq, int index, long tm, Topology cppTopo)
    {
      cppState = StateNew(seq, index, tm, cppTopo);
    }

    public Sequence GetSequence()
    {
      return GetSequenceState(cppState);
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
      return GetPosState(this, jnt, varIdx, out posVal);
    }

    public bool GetSpeed(AbstractJoint jnt, int varIdx, out double speedVal)
    {
      return GetSpeedState(this, jnt, varIdx, out speedVal);
    }

    public bool GetAccel(AbstractJoint jnt, int varIdx, out double accVal)
    {
      return GetAccelState(this, jnt, varIdx, out accVal);
    }

    public bool GetJerk(AbstractJoint jnt, int varIdx, out double jerkVal)
    {
      return GetJerkState(this, jnt, varIdx, out jerkVal);
    }

    // For all other info call this method and interrogate the topology

    public bool SetTopologyToThis()
    {
      return SetTopologyToThisState(this);
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
    extern static private bool GetPosState(State state, AbstractJoint jnt, int varIdx, out double posVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetSpeedState(State state, AbstractJoint jnt, int varIdx, out double speedVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetAccelState(State state, AbstractJoint jnt, int varIdx, out double accVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetJerkState(State state, AbstractJoint jnt, int varIdx, out double jerkVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool SetTopologyToThisState(State state);

    // End Interface Section
  }
}


