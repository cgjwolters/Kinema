using System.Drawing;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Sequence
  {
    private readonly IntPtr cppSequence;
    private readonly string name;
    public readonly Topology topology;

    private List<State> stateList = [];

    public Sequence(Topology topo, string name)
    {
      topology = topo;
      this.name = name;
      cppSequence = SequenceNew(topo.cppTopology, name);
    }
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
      return GetStateCountSequence(cppSequence);
    }

    public State GetState(int index)
    {
      int cppCount = GetStateCount();

      if (stateList.Count != cppCount) stateList = new(cppCount);

      if (index < 0 || index >= cppCount) {
        throw new IndexOutOfRangeException("State count");
      }

      if (stateList[index] == null) {
        State state = new State(GetStateSequence(cppSequence, index));

        stateList[index] = state;
      }

      return stateList[index];
    }

    // Interface Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr SequenceNew(IntPtr top, string name);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void AddCurrentTopoStateSequence(IntPtr cppSequence);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static int GetStateCountSequence(IntPtr cppSequence);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr GetStateSequence(IntPtr cppSeq, int index);

    // End Interface Section
  }
}