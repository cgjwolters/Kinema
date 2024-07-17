//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Ball Joint + 1 slide (4 degrees of freedom) ----------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_JNTBALLSLIDE_INC
#define INOKIN_JNTBALLSLIDE_INC

#include "KinAbstractJoint.h"

namespace InoKin {

//---------------------------------------------------------------------------

// varPos[0] = rotation about z-axis
// varPos[1] = rotation about y-axis
// varPos[2] = rotation about x-axis
// varPos[3] = slide along (transformed) z-axis

class JntBallSlide : public AbstractJoint
{
  JntBallSlide(const JntBallSlide& cp) = delete;            // No copying
  JntBallSlide& operator=(const JntBallSlide& cp) = delete; // No assignment

protected:
  virtual AbstractJoint *clone(Grip& newGrip) const;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

public:
  explicit JntBallSlide(Grip& grp, const wchar_t *name);
  explicit JntBallSlide(Grip& grp, const JntBallSlide& cp);

  virtual void initVarsFromPos(bool fixedAlso);
};

} // namespace

//---------------------------------------------------------------------------
#endif
