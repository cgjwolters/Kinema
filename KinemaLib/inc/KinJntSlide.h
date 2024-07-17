//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------- Kinema: Kinematic Simulation Program ---------------------------
//-------------------------------------------------------------------------------
//------------------------ Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//------------------------------------------------------ C.Wolters --------------
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------- Slide Joint (1 degrees of freedom: translation) ----------------
//-------------------------------------------------------------------------------

#ifndef INOKIN_JNTSLIDE_INC
#define INOKIN_JNTSLIDE_INC

#include "KinAbstractJoint.h"

namespace InoKin {

//-------------------------------------------------------------------------------

class JntSlide : public AbstractJoint
{
  JntSlide(const JntSlide& cp) = delete;            // No copying
  JntSlide& operator=(const JntSlide& cp) = delete; // No assignment

protected:
  virtual AbstractJoint *clone(Grip& newGrip) const;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

public:
  explicit JntSlide(Grip& grp, const wchar_t *name);
  explicit JntSlide(Grip& grp, const JntSlide& cp);

  virtual void initVarsFromPos(bool fixedAlso);
};

} // namespace

//-------------------------------------------------------------------------------
#endif
