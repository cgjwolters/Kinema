//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Cross Joint (2 degrees (rotations) of freedom) -------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_JNTCROSS_INC
#define INOKIN_JNTCROSS_INC

#include "KinAbstractJoint.h"

namespace InoKin {

//---------------------------------------------------------------------------

// varPos[0] = rotation about z-axis
// varPos[1] = rotation about y-axis

class JntCross : public AbstractJoint
{
  JntCross(const JntCross& cp) = delete;            // No copying
  JntCross& operator=(const JntCross& cp) = delete; // No assignment

protected:
  virtual AbstractJoint *clone(Grip& newGrip) const;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

public:
  explicit JntCross(Grip& grp, const wchar_t *name);
  explicit JntCross(Grip& grp, const JntCross& cp);

  virtual void initVarsFromPos(bool fixedAlso);
};

} // namespace

//---------------------------------------------------------------------------
#endif
