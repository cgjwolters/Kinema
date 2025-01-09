using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Sequence
  {
    private readonly IntPtr cppSequence;

    public Sequence(Topology topo, string name)
    {
      cppSequence = SequenceNew(topo.cppTopology, name);
    }

    //Topology& getTopology();

    //Model getModel();

    //string getName();

  // Interface Section

  [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr SequenceNew(IntPtr cppTopo, string name);
  }

  [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
  //Topology getTopologySequence()
  //{
  //  // return 
  //}

}
