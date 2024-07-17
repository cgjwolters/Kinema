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

#include "KinJntSlide.h"

using namespace Ino;

namespace InoKin {

//-------------------------------------------------------------------------------

AbstractJoint *JntSlide::clone(Grip& newGrip) const
{
  return new JntSlide(newGrip,*this);
}

//-------------------------------------------------------------------------------

JntSlide::JntSlide(Grip& grp, const wchar_t *name)
: AbstractJoint(grp,name,1)
{
  isAngular[0] = false;
}

//-------------------------------------------------------------------------------

JntSlide::JntSlide(Grip& grp, const JntSlide& cp)
: AbstractJoint(grp,cp)
{
}

//-------------------------------------------------------------------------------

void JntSlide::getVarTrf(int idx, Trf3& trf) const
{
  trf.init();
  trf.isDerivative = false;

  if (idx != 0) return;

  trf(2,3) = -varPos[0];
}

//-------------------------------------------------------------------------------

void JntSlide::getVarDerTrf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx != 0) return;

  trf(2,3) = -1.0;
}

//-------------------------------------------------------------------------------

void JntSlide::getVarDer2Trf(int /*idx*/, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;
}

//-------------------------------------------------------------------------------

void JntSlide::getVarDer3Trf(int /*idx*/, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;
}

//-------------------------------------------------------------------------------

void JntSlide::initVarsFromPos(bool fixedAlso)
{
  calcPosFromNeighbours();

  const Trf3& trf = getPos();

  if (fixedAlso || !getFixed(0)) {
    varPos[0] = -trf(2,3);
  }

  setModelModified();
}

} //namespace

//-------------------------------------------------------------------------------
