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
      return GetModelTopo(cppTopology);
    }

    public Sequence NewSequence(string name)
    {
      Sequence seq = new(this, name);

      SeqList.Add(seq);

      return seq;
    }

    public int GetVarSz()
    {
      return getVarSzTopo(cppTopology); 
    }

    public bool SolvePos(int maxIter, double rotTol, double posTol,
                                          double[] varPosVec, ref int iter)
    {
      return false;
    }

    // Interface Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private Model GetModelTopo(IntPtr cppTopo);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int getVarSzTopo(IntPtr cppTopology);
  }
}
