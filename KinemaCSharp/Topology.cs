using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Topology
  {
    internal readonly IntPtr cppTopology;

    public readonly List<Sequence> SeqList = [];

    public Topology(IntPtr cppTopology)
    {
      this.cppTopology = cppTopology;

      Model mdl = GetModel();
      mdl.TopoList.Add(this);
    }

    public Model GetModel()
    {
      return GetModelTopology(cppTopology);
    }

    public Sequence NewSequence(string name)
    {
      Sequence seq = new(this, name);

      SeqList.Add(seq);

      return seq;
    }

    // Interface Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private Model GetModelTopology(IntPtr cppTopo);
  }
}
