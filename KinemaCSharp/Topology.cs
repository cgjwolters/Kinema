using System.IO.IsolatedStorage;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Topology
  {
    Model model;
    internal readonly IntPtr cppTopology;

    public readonly List<Sequence> SeqList = [];

    public Topology(Model mdl1, IntPtr cppTopology)
    {
      model = mdl1;
      this.cppTopology = cppTopology;
      Model mdl = GetModel();
      mdl.TopoList.Add(this);
 
    }

    public Model GetModel()
    {
      return model;
    }

    public Sequence NewSequence(string name)
    {
      Sequence seq = new(this, name);

      SeqList.Add(seq);

      return seq;
    }

    public int GetVarSz()
    {
      return GetVarSzTopo(cppTopology); 
    }

    public bool SolvePos(int maxIter, double rotTol, double posTol,
                                          double[] varPosVec, ref int iter)
    {
      return SolvePosTopology(cppTopology, maxIter, rotTol, posTol, varPosVec, ref iter);
    }

    // Interface Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int GetVarSzTopo(IntPtr cppTopology);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool SolvePosTopology(IntPtr cppTopology, int maxIter, double rotTol, double posTol,
                                                double[] varPosVec, ref int iter);
  }
}
