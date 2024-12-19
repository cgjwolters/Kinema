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
      return GetPosState(, varIdx, out posVal);
    }

    //  bool getPos(const AbstractJoint& jnt, int varIdx, double& posVal) const;
    //bool getSpeed(const AbstractJoint& jnt, int varIdx, double& speedVal) const;
    //bool getAccel(const AbstractJoint& jnt, int varIdx, double& accVal) const;
    //bool getJerk(const AbstractJoint& jnt, int varIdx, double& jerkVal) const;

    //// For all other info call this method and interrogate the topology:
    //bool setTopologyToThis() const;



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
    extern static private bool GetPosState(AbstractJoint jnt, int varIdx, out double posVal);


    // End Interface Section
  }
}


