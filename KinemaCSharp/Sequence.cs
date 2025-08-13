using System.Drawing;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Sequence(Topology topo, string name)
  {
    private readonly IntPtr cppSequence = SequenceNew(topo.cppTopology, name);
    private readonly string name = name;
    public readonly Topology topology = topo;

    private List<State> stateList = [];

    public Topology GetTopology()
    {
      return topology;
    }

    public Model GetModel()
    {
      return topology.GetModel();
    }

    public string GetName()
    {
      return name;
    }

    public void AddCurrentTopoState()
    {
      AddCurrentTopoStateSequence(cppSequence);
    }

    public int GetStateCount()
    {
      int count = 0;
      GetStateCountSequence(cppSequence,ref count);
      return count;
    }

    public State GetState(int index)
    {
      int cppCount = GetStateCount();

      if (stateList.Count != cppCount) {
        stateList = new(cppCount);

        for (int i=0; i<cppCount; ++i) {
          stateList.Add(new State(GetStateSequence(cppSequence, index),this));
        }
      }

      if (index < 0 || index >= cppCount) {
        throw new IndexOutOfRangeException("State count");
      }

      return stateList[index];
    }

    // Interface Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr SequenceNew(IntPtr top, string name);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void AddCurrentTopoStateSequence(IntPtr cppSequence);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetStateCountSequence(IntPtr cppSequence, ref int count);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr GetStateSequence(IntPtr cppSeq, int index);

    // End Interface Section
  }
}