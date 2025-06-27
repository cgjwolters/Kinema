using System.Runtime.InteropServices;

namespace KinemaLibCs
{
  public abstract class AbstractJoint
  {
    protected IntPtr cppJoint;

    public AbstractJoint(Grip grp, string name)
    {
      grp.model.JointMap.Add(name, this);
    }
    public void GetPos(out Trf3 pos) {
      GetPosAbstractJoint(cppJoint, out pos);
    }

    public void GetInvPos(out Trf3 invPos) {
      GetInvPosAbstractJoint(cppJoint, out invPos);
    }

    public void GetDer(out Trf3 der) {
      GetDerAbstractJoint(cppJoint, out der);
    }

    public void GetInvDer(out Trf3 invDer)
    {
      GetInvDerAbstractJoint(cppJoint, out invDer);
    }

    public void GetAcc(out Trf3 acc)
    {
      GetAccAbstractJoint(cppJoint, out acc);
    }

    public void GetInvAcc(out Trf3 invAcc)
    {
      GetInvAccAbstractJoint(cppJoint, out invAcc);
    }

    public void GetJerk(out Trf3 jerk)
    {
      GetJerkAbstractJoint(cppJoint, out jerk);
    }

    public void GetInvJerk(out Trf3 invJerk)
    {
      GetInvJerkAbstractJoint(cppJoint, out invJerk);
    }

    public int GetVarCnt()
    {
      return GetVarCntAbstractJoint(cppJoint);
    }

    public int GetVarCnt(bool fixd)
    {
      return GetVarCntAbstractJoint2(cppJoint, fixd);
    }

    public bool GetFixed(int locIdx)
    {
      return GetFixedAbstractJoint(cppJoint, locIdx);
    }

    public void SetFixed(int locIdx, bool isFixed)
    {
      SetFixedAbstractJoint(cppJoint, locIdx, isFixed);
    }

    public void SetFixedAll(bool isFixed)
    {
      SetFixedAllAbstractJoint(cppJoint, isFixed);
    }

    public bool GetIsAngular(int locIdx)
    {
      return GetIsAngularAbstractJoint(cppJoint, locIdx);
    }

    public virtual void InitVarsFromPos(bool fixedAlso)
    {
      InitVarsFromPosAbstractJoint(cppJoint, fixedAlso);
    }

    public void ClearVarIndices()
    {
      ClearVarIndicesAbstractJoint(cppJoint);
    }

    public int GetVarIdx(int locIdx)
    {
      return GetVarIdxAbstractJoint(cppJoint, locIdx);
    }

    public void SetVarIdx(int locIdx, int varIdx)
    {
      SetVarIdxAbstractJoint(cppJoint, locIdx, varIdx);
    }

    public int GetLocIdx(int vIdx)
    {
      return GetLocIdxAbstractJoint(cppJoint, vIdx);
    }

    public double GetVal(int locIdx)
    {
      return GetValAbstractJoint(cppJoint, locIdx);
    }

    public void SetVal(int locIdx, double newVal)
    {
      SetValAbstractJoint(cppJoint, locIdx, newVal);
    }

    //public void GetVars(bool isFixed, Ino::Vector& varVec)
    //{

    //}

    //public void SetVars(bool isFixed, const Ino::Vector& varVec)
    //{

    //}

    //public void SetVars(const Ino::Vector& varVec, const Ino::Vector& fixedVec)
    //{

    //}

    public double GetSpeed(int locIdx)
    {
      return GetSpeedAbstractJoint(cppJoint, locIdx);
    }

    public void SetSpeed(int locIdx, double newSpeed)
    {
      SetSpeedAbstractJoint(cppJoint,locIdx,newSpeed);
    }

    //public void GetSpeeds(bool isFixed, Ino::Vector& speedVec)
    //{

    //}

    //public void SetSpeeds(bool isFixed, readonly Ino::Vector& speedVec)
    //{

    //}

    //public void SetSpeeds(const Ino::Vector& varSpeedVec,
    //               const Ino::Vector& fixedSpeedVec)
    //{

    //}

    public void ClearTrfCaches()
    {
      ClearTrfCachesAbstractJoint(cppJoint);
    }

    public void GetDerivative(int locIdx, out Trf3 derTrf)
    {
      GetDerivativeAbstractJoint(cppJoint, locIdx, out derTrf);
    }

    public void GetInvDerivative(int locIdx, out Trf3 invDerTrf)
    {
      GetInvDerivativeAbstractJoint(cppJoint, locIdx, out invDerTrf);
    }

    public double GetAccel(int locIdx)
    {
      return GetAccelAbstractJoint(cppJoint, locIdx);
    }

    public void SetAccel(int locIdx, double newAccel)
    {
      SetAccelAbstractJoint(cppJoint, locIdx, newAccel);
    }

    //public void GetAccels(bool isFixed, Ino::Vector& accelVec)
    //{

    //}

    //public void SetAccels(bool isFixed, const Ino::Vector& accelVec)
    //{
    //}

    //public void SetAccels(const Ino::Vector& varAccelVec,
    //               const Ino::Vector& fixedAccelVec)
    //{ 
    //}

    public void GetSecDerivative(int locIdx, out Trf3 accTrf)
    {
      GetSecDerivativeAbstractJoint(cppJoint, locIdx, out accTrf);
    }

    public void GetInvSecDerivative(int locIdx, out Trf3 accTrf)
    {
      GetInvSecDerivativeAbstractJoint(cppJoint, locIdx, out accTrf);
    }

    public void GetAccMixed(out Trf3 mixTrf)
    {
      GetAccMixedAbstractJoint(cppJoint, out mixTrf);
    }

    public void GetInvAccMixed(out Trf3 invMixTrf)
    {
      GetInvAccMixedAbstractJoint(cppJoint, out invMixTrf);
    }

    public void GetJerkMixed(out Trf3 mixTrf)
    {
      GetJerkMixedAbstractJoint(cppJoint, out mixTrf);
    }

    public void GetInvJerkMixed(out Trf3 invMixTrf)
    {
      GetInvJerkMixedAbstractJoint(cppJoint, out invMixTrf);
    }

    public double GetJerk(int locIdx)
    {
      return GetJerkAbstractJoint(cppJoint, locIdx);
    }

    public void SetJerk(int locIdx, double newJerk)
    {
      SetJerkAbstractJoint(cppJoint, locIdx, newJerk);
    }

    //public void GetJerks(bool isFixed, Ino::Vector& jerkVec)
    //{

    //}

    //public void SetJerks(bool isFixed, const Ino::Vector& jerkVvec)
    //{

    //}

    //public void SetJerks(const Ino::Vector& varJerkVec,
    //                     const Ino::Vector& fixedJerkVec)
    //{

    //}

    public void GetThirdDerivative(int locIdx, out Trf3 jerkTrf)
    {
      GetThirdDerivativeAbstractJoint(cppJoint, locIdx, out jerkTrf);
    }

    public void GetInvThirdDerivative(int locIdx, out Trf3 jerkTrf)
    {
      GetInvThirdDerivativeAbstractJoint(cppJoint, locIdx, out jerkTrf);
    }

    // Import section

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetPosAbstractJoint(IntPtr cppJoint, out Trf3 pos);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvPosAbstractJoint(IntPtr cppJoint, out Trf3 invPos);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetDerAbstractJoint(IntPtr cppJoint, out Trf3 der);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvDerAbstractJoint(IntPtr cppJoint, out Trf3 invDer);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetAccAbstractJoint(IntPtr cppJoint, out Trf3 acc);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvAccAbstractJoint(IntPtr cppJoint, out Trf3 invAcc);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetJerkAbstractJoint(IntPtr cppJoint, out Trf3 jerk);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvJerkAbstractJoint(IntPtr cppJoint, out Trf3 invJerk);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int GetVarCntAbstractJoint(IntPtr cppJoint);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int GetVarCntAbstractJoint2(IntPtr cppJoint, bool fixd);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetFixedAbstractJoint(IntPtr cppJoint, int locIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetFixedAbstractJoint(IntPtr cppJoint, int locIdx, bool isFixed);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetFixedAllAbstractJoint(IntPtr cppJoint, bool isFixed);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private bool GetIsAngularAbstractJoint(IntPtr cppJoint, int locIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void InitVarsFromPosAbstractJoint(IntPtr cppJoint, bool fixedAlso);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void ClearVarIndicesAbstractJoint(IntPtr cppJoint);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int GetVarIdxAbstractJoint(IntPtr cppJoint, int locIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetVarIdxAbstractJoint(IntPtr cppJoint, int locIdx, int varIdx);
  
    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private int GetLocIdxAbstractJoint(IntPtr cppJoint, int vIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private double GetValAbstractJoint(IntPtr cppJoint, int locIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetValAbstractJoint(IntPtr cppJoint, int locIdx, double newVal);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private double GetSpeedAbstractJoint(IntPtr cppJoint, int locIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetSpeedAbstractJoint(IntPtr cppJoint, int locaIdx, double newSpeed);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void ClearTrfCachesAbstractJoint(IntPtr cppJoint);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetDerivativeAbstractJoint(IntPtr cppJoint, int locIdx, out Trf3 derTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvDerivativeAbstractJoint(IntPtr cppJoint, int locIdx, out Trf3 invDerTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private double GetAccelAbstractJoint(IntPtr cppJoint, int locIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetAccelAbstractJoint(IntPtr cppJoint, int locIdx, double newAccel);
      
    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetSecDerivativeAbstractJoint(IntPtr cppJoint, int locIdx, out Trf3 accTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvSecDerivativeAbstractJoint(IntPtr cppJoint, int locIdx, out Trf3 accTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetAccMixedAbstractJoint(IntPtr cppJoint, out Trf3 mixTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvAccMixedAbstractJoint(IntPtr cppJoint, out Trf3 invMixTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetJerkMixedAbstractJoint(IntPtr cppJoint, out Trf3 mixTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvJerkMixedAbstractJoint(IntPtr cppJoint, out Trf3 invMixTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private double GetJerkAbstractJoint(IntPtr cppJoint, int locIdx);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void SetJerkAbstractJoint(IntPtr cppJoint, int locIdx, double newJerk);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetThirdDerivativeAbstractJoint(IntPtr cppJoint, int locIdx, out Trf3 jerkTrf);

    [DllImport("KinemaLib.dll", CharSet = CharSet.Unicode)]
    extern static private void GetInvThirdDerivativeAbstractJoint(IntPtr cppJoint, int locIdx, out Trf3 jerkTrf);
  }

  // End Import section
}
