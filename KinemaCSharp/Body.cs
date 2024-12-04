using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public class Body
  {
    private readonly Model model;
    private readonly IntPtr cppBody;

    public Body(Model mdl, string name) {
      model = mdl;
      cppBody = BodyNew(model, name);
    }

    public Body GetParent()
    {
      GetParentBody (cppBody,out Body parent);
      return parent;
    }

    public Grip GetParentGrip()
    {
      GetParentGripBody(cppBody, out Grip parentGrip);
      return parentGrip;
    }
    public int GetTreeLevel()
    {
      return GetTreeLevelBody(cppBody);
    }

    public Grip GripTo(Body otherBody)
    {
      GripToBody(cppBody, otherBody, out Grip grip);
      return grip;
    }

    public Trf3 GetPos()
    {
      GetPosBody(cppBody,out Trf3 trf);
      return trf;
    }

    public void SetPos(Trf3 newPos)
    {
      SetPosBody(cppBody,newPos);
    }

    public Trf3 GetInvPos()
    {
      GetInvPosBody(cppBody,out Trf3 trf);
      return trf;
    }

    public Trf3 GetSpeed()
    {
      GetSpeedBody(cppBody, out Trf3 trf);
      return trf;
    }

    public void SetSpeed(Trf3 newSpeed)
    {
      SetSpeedBody(cppBody, newSpeed);
    }

    public Trf3 GetInvSpeed()
    {
      GetInvSpeedBody(cppBody, out Trf3 trf);
      return trf;
    }

    public Trf3 GetAccel()
    {
      GetAccelBody(cppBody, out Trf3 trf);
      return trf;
    }

    public void SetAccel(Trf3 newAccel)
    {
      SetAccelBody(cppBody, newAccel);
    }

    public Trf3 GetInvAccel()
    {
      GetInvAccelBody(cppBody, out Trf3 trf);
      return trf;
    }

    public Trf3 GetJerk()
    {
      GetJerkBody(cppBody, out Trf3 trf);
      return trf;
    }

    public void SetJerk(Trf3 newJerk)
    {
      SetJerkBody(cppBody, newJerk);
    }

    public Trf3 GetInvJerk()
    {
      GetInvJerkBody(cppBody, out Trf3 trf);
      return trf;
    }

    public Vec3 GetAbsPos()
    {
      GetAbsPosBody(cppBody, out Vec3 pos);
      return pos;
    }

    public Trf3 GetAbsPos2()
    {
      GetAbsPosBody2(cppBody, out Trf3 trf);
      return trf;
    }

    public Vec3 GetAbsSpeed()
    {
      GetAbsSpeedBody(cppBody, out Vec3 speed);
      return speed;
    }

    public void Translate(Vec3 offset)
    {
      TranslateBody(cppBody, offset);
    }

    public void Transform(Trf3 trf)
    {
      TransformBody(cppBody, trf);
    }

    public Grip GetGrip(int idx)
    {
      return GetGripBody(cppBody, idx);
    }

    //  void getAbsGripPos(int idx, Ino::Trf3& trf) const;
    //void getAbsGripPos(const wchar_t* name, Ino::Trf3& trf) const;

    // Import Section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private IntPtr BodyNew(Model mdl, string name);

    [DllImport("KinemaLib.dll")]
    extern private static IntPtr GetParentBody(IntPtr cppBody, out Body parent);

    [DllImport("KinemaLib.dll")]
    extern private static IntPtr GetParentGripBody(IntPtr cppBody, out Grip parentGrip);

    [DllImport("KinemaLib.dll")]
    extern private static int GetTreeLevelBody(IntPtr cppBody);

    [DllImport("KinemaLib.dll")]
    extern private static void GripToBody(IntPtr cppBody, Body otherBody, out Grip grip);

    [DllImport("KinemaLib.dll")]
    extern private static void GetPosBody(IntPtr cppBody, out Trf3 trf);

    [DllImport("KinemaLib.dll")]
    extern private static void SetPosBody(IntPtr cppBody, Trf3 newPos);

    [DllImport("KinemaLib.dll")]
    extern private static void GetInvPosBody(IntPtr cppBody, out Trf3 invPos);

    [DllImport("KinemaLib.dll")]
    extern private static void GetSpeedBody(IntPtr cppBody, out Trf3 trf);

    [DllImport("KinemaLib.dll")]
    extern private static void SetSpeedBody(IntPtr cppBody, Trf3 newSpeed);

    [DllImport("KinemaLib.dll")]
    extern private static void GetInvSpeedBody(IntPtr cppBody, out Trf3 invSpeed);

    [DllImport("KinemaLib.dll")]
    extern private static void GetAccelBody(IntPtr ccppBody, out Trf3 accel);

    [DllImport("KinemaLib.dll")]
    extern private static void SetAccelBody(IntPtr cppBody, Trf3 newAccel);

    [DllImport("KinemaLib.dll")]
    extern private static void GetInvAccelBody(IntPtr ccppBody, out Trf3 invAccel);

    [DllImport("KinemaLib.dll")]
    extern private static void GetJerkBody(IntPtr cppBody, out Trf3 trf);

    [DllImport("KinemaLib.dll")]
    extern private static void SetJerkBody(IntPtr cppBody, Trf3 newJerk);

    [DllImport("KinemaLib.dll")]
    extern private static void GetInvJerkBody(IntPtr cppBody, out Trf3 invJerk);

    [DllImport("KinemaLib.dll")]
    extern private static void GetAbsPosBody(IntPtr cppBody, out Vec3 pos);

    [DllImport("KinemaLib.dll")]
    extern private static void GetAbsPosBody2(IntPtr cppBody, out Trf3 trf);

    [DllImport("KinemaLib.dll")]
    extern private static void GetAbsSpeedBody(IntPtr cppBody, out Vec3 speed);

    [DllImport("KinemaLib.dll")]
    extern private static void TranslateBody(IntPtr cppBody, Vec3 offset);

    [DllImport("KinemaLib.dll")]
    extern private static void TransformBody(IntPtr cppBody, Trf3 trf);

    [DllImport("KinemaLib.dll")]
    extern private static int GetGripCountBody(IntPtr cppBody);

    [DllImport("KinemaLib.dll")]
    extern private static Grip GetGripBody(IntPtr cppBody, int idx);

    // End Import Section
  }
}
