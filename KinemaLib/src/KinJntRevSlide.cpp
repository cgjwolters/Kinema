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

#include "KinJntRevSlide.h"

#include <cmath>

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

AbstractJoint *JntRevSlide::clone(Grip& newGrip) const
{
  return new JntRevSlide(newGrip,*this);
}

//---------------------------------------------------------------------------

JntRevSlide::JntRevSlide(Grip& grp, const wchar_t *name)
: AbstractJoint(grp,name,2)
{
  isAngular[0] = true;
  isAngular[1] = false;
}

//---------------------------------------------------------------------------

JntRevSlide::JntRevSlide(Grip& grp, const JntRevSlide& cp)
: AbstractJoint(grp,cp)
{
}

//---------------------------------------------------------------------------

void JntRevSlide::getVarTrf(int idx, Trf3& trf) const
{
  trf.init();
  trf.isDerivative = false;

  if (idx == 0) {
    double c = cos(varPos[0]);
    double s = sin(varPos[0]);

    trf(0,0) =  c; trf(0,1) = s;
    trf(1,0) = -s; trf(1,1) = c;
  }
  else if (idx == 1) trf(2,3) = -varPos[1];
}

//---------------------------------------------------------------------------

void JntRevSlide::getVarDerTrf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx == 0) {
    double c = cos(varPos[0]);
    double s = sin(varPos[0]);

    trf(0,0) = -s; trf(0,1) =  c;
    trf(1,0) = -c; trf(1,1) = -s;
  }
  else if (idx == 1) trf(2,3) = -1.0;
}

//---------------------------------------------------------------------------

void JntRevSlide::getVarDer2Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx == 0) {
    double c = cos(varPos[0]);
    double s = sin(varPos[0]);

    trf(0,0) = -c; trf(0,1) = -s;
    trf(1,0) =  s; trf(1,1) = -c;
  }
}

//---------------------------------------------------------------------------

void JntRevSlide::getVarDer3Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx == 0) {
    double c = cos(varPos[0]);
    double s = sin(varPos[0]);

    trf(0,0) =  s; trf(0,1) = -c;
    trf(1,0) =  c; trf(1,1) =  s;
  }
}

//---------------------------------------------------------------------------

void JntRevSlide::initVarsFromPos(bool fixedAlso)
{
  calcPosFromNeighbours();

  const Trf3& trf = getPos();

  if (fixedAlso || !getFixed(0)) {
    varPos[0] = atan2(trf(0,1),trf(0,0));
  }

  if (fixedAlso || !getFixed(1)) {
    varPos[1] = -trf(2,3);
  }

  setModelModified();
}

} // namespace

// Interface Section

void* JointRevSlideNew(void* cppGrip, const wchar_t* name) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  InoKin::JntRevSlide* jnt = new InoKin::JntRevSlide(*grip, name);

  return jnt;
}

//---------------------------------------------------------------------------
