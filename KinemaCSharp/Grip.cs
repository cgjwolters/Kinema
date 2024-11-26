using KinemaLibCs;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  internal class Grip
  {
    private Model model;
    private readonly IntPtr cppGrip;

    public Grip(Model mdl, string name,
                Body body1, Trf3 pos1,
                Body body2, Trf3 pos2)
    {
      model = mdl;
      cppGrip = GripNew(model, name, body1, pos1, body2, pos2);
    }

    Body GetBody1()
    {
      GetBody1Grip(cppGrip,out Body body);
      return body;
    }

    Body GetBody2()
    {
      GetBody2Grip(cppGrip, out Body body);
      return body;
    }

    Body GetOtherBody()
    {
      GetOtherBodyGrip(cppGrip, out Body body);
      return body;
    }

    // AbstractJoint* getJoint() const { return joint; }

    bool IsParentRel()
    {
      return IsParentRelGrip(cppGrip);
    }

    int GetLoopCnt()
    {
      return GetLoopCntGrip(cppGrip);
    }

    Trf3 GetPos1()
    {
      GetPos1Grip(cppGrip,out Trf3 pos);
      return pos;
    }

    Trf3 GetInvPos1()
    {
      GetInvPos1Grip(cppGrip, out Trf3 pos);
      return pos;
    }

    void SetPos1(Trf3 pos)
    {
      SetPos1Grip(cppGrip, pos);
    }

    Trf3 GetPos2()
    {
      GetPos2Grip(cppGrip, out Trf3 pos);
      return pos;
    }
    Trf3 GetInvPos2()
    {
      GetInvPos2Grip(cppGrip, out Trf3 pos);
      return pos;
    }
    void setPos2(Trf3 pos)
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
