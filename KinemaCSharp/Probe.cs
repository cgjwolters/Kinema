using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Probe
  {
    private readonly IntPtr cppProbe;

    public Probe(Body body, string name, Trf3 initPos)
    {
      cppProbe = ProbeNew(body, name, initPos);
      body.model.probeMap.Add(name, this);
    }

    public Vec3 GetAbsPos()
    {
      GetAbsPosProbe(cppProbe, out Vec3 apos);
      return apos;
    }

    public Trf3 GetAbsPos2()
    {
      GetAbsPosProbe2(cppProbe, out Trf3 apos);
      return apos;
    }

    public void GetAbsPos(out Vec3 apos)
    {
      GetAbsPosProbe(cppProbe, out apos);
    }

    public void GetAbsPos2(out Trf3 apos)
    {
      GetAbsPosProbe2(cppProbe, out apos);
    }

    public void GetAbsSpeed(out Vec3 asp)
    {
      GetAbsSpeedProbe(cppProbe, out asp);
    }

    public void GetAbsSpeed2(out Trf3 asp)
    {
      GetAbsSpeedProbe2(cppProbe, out asp);
    }

    public void GetAbsAccel(out Vec3 acc)
    {
      GetAbsAccelProbe(cppProbe, out acc);
    }

    public void GetAbsAccel2(out Trf3 acc)
    {
      GetAbsAccelProbe2(cppProbe, out acc);
    }

    public void GetPos(out Trf3 pos)
    {
      GetPosProbe(cppProbe, out pos);
    }

    public void SetPos(Trf3 newPos)
    {
      SetPosProbe(cppProbe, newPos);
    }

    // Interface Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static IntPtr ProbeNew(Body body, string name, Trf3 initPos);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetAbsPosProbe(IntPtr cppPrb, out Vec3 apos);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetAbsPosProbe2(IntPtr cppPrb, out Trf3 apos);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetAbsSpeedProbe(IntPtr cppPrb, out Vec3 asp);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetAbsSpeedProbe2(IntPtr cppPrb, out Trf3 asp);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetAbsAccelProbe(IntPtr cppPrb, out Vec3 aacc);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetAbsAccelProbe2(IntPtr cppPrb, out Trf3 aacc);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void GetPosProbe(IntPtr cppPrb, out Trf3 pos);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern private static void SetPosProbe(IntPtr cppPrb, Trf3 newPos);
  }
}
