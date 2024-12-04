//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Revolute Joint (1 degree of freedom: rotation) -------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_JNTREV_INC
#define INOKIN_JNTREV_INC

#include "KinAbstractJoint.h"

namespace InoKin {

//---------------------------------------------------------------------------

// varPos[0] = rotation about z-axis

class JntRev : public AbstractJoint
{
  JntRev(const JntRev& cp) = delete;            // No copying
  JntRev& operator=(const JntRev& cp) = delete; // No assignment

protected:
  virtual AbstractJoint *clone(Grip& newGrip) const;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

public:
  explicit JntRev(Grip& grp, const wchar_t *name);
  explicit JntRev(Grip& grp, const JntRev& cp);

  virtual void initVarsFromPos(bool fixedAlso);
};

} // namespace

// Interface Section

extern "C" __declspec(dllexport) void* JointRevNew(void* cppGrip, const wchar_t* name);

//---------------------------------------------------------------------------
#endif
