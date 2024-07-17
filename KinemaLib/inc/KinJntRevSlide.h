//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Revolute Joint + slide (2 degrees of freedom) --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_JNTREVSLIDE_INC
#define INOKIN_JNTREVSLIDE_INC

#include "KinAbstractJoint.h"

namespace InoKin {

//-------------------------------------------------------------------------------

// varPos[0] = rotation about z-axis
// varPos[1] = slide along z-axis

class JntRevSlide : public AbstractJoint
{
  JntRevSlide(const JntRevSlide& cp) = delete;            // No copying
  JntRevSlide& operator=(const JntRevSlide& cp) = delete; // No assignment

protected:
  virtual AbstractJoint *clone(Grip& newGrip) const;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

public:
  explicit JntRevSlide(Grip& grp, const wchar_t *name);
  explicit JntRevSlide(Grip& grp, const JntRevSlide& cp);

  virtual void initVarsFromPos(bool fixedAlso);
};

} // namespace

//-------------------------------------------------------------------------------
#endif
