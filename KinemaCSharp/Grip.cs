using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  internal class Grip
  {
    private readonly Model model;
    private readonly IntPtr cppGrip;

    public Grip(Model mdl, string name,
                Body body1, Trf3 pos1,
                Body body2, Trf3 pos2)
    {
      model = mdl;
      cppGrip = GripNew(model, name, body1, pos1, body2, pos2);
    }

    public Body GetBody1()
    {
      GetBody1Grip(cppGrip,out Body body);
      return body;
    }

    public Body GetBody2()
    {
      GetBody2Grip(cppGrip, out Body body);
      return body;
    }

    public Body GetOtherBody()
    {
      GetOtherBodyGrip(cppGrip, out Body body);
      return body;
    }

    public AbstractJoint GetJoint()
    {
      GetJointGrip(cppGrip, out AbstractJoint joint);
      return joint;
    }

    public bool IsParentRel()
    {
      return IsParentRelGrip(cppGrip);
    }

    public int GetLoopCnt()
    {
      return GetLoopCntGrip(cppGrip);
    }

    public Trf3 GetPos1()
    {
      GetPos1Grip(cppGrip,out Trf3 pos);
      return pos;
    }

    public Trf3 GetInvPos1()
    {
      GetInvPos1Grip(cppGrip, out Trf3 pos);
      return pos;
    }

    public void SetPos1(Trf3 pos)
    {
      SetPos1Grip(cppGrip, pos);
    }

    public Trf3 GetPos2()
    {
      GetPos2Grip(cppGrip, out Trf3 pos);
      return pos;
    }
    public Trf3 GetInvPos2()
    {
      GetInvPos2Grip(cppGrip, out Trf3 pos);
      return pos;
    }
    public void SetPos2(Trf3 pos)
    {
      SetPos2Grip(cppGrip, pos);
    }

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr GripNew(Model mdl, string name,
                                         Body body1, Trf3 pos1,
                                         Body body2, Trf3 pos2);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetBody1Grip(IntPtr cppGrip, out Body body);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetBody2Grip(IntPtr cppGrip, out Body body);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetOtherBodyGrip(IntPtr cppGrip, out Body body);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetJointGrip(IntPtr cppGrip, out AbstractJoint joint);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool IsParentRelGrip(IntPtr cppGrip);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int GetLoopCntGrip(IntPtr cppGrip);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetPos1Grip(IntPtr cppGrip, out Trf3 pos1);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvPos1Grip(IntPtr cppGrip, out Trf3 pos1);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetPos1Grip(IntPtr cppGrip, Trf3 pos);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetPos2Grip(IntPtr cppGrip, out Trf3 pos1);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvPos2Grip(IntPtr cppGrip, out Trf3 pos1);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetPos2Grip(IntPtr cppGrip, Trf3 pos);

  // AbstractJoint* getJoint() const { return joint; }

    // End Import Section
  }
}
