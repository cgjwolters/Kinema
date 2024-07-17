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

#include "KinJntRev.h"

#include <cmath>

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

AbstractJoint *JntRev::clone(Grip& newGrip) const
{
  return new JntRev(newGrip,*this);
}

//---------------------------------------------------------------------------

JntRev::JntRev(Grip& grp, const wchar_t *name)
: AbstractJoint(grp,name,1)
{
  isAngular[0] = true;
}

//---------------------------------------------------------------------------

JntRev::JntRev(Grip& grp, const JntRev& cp)
: AbstractJoint(grp,cp)
{
}

//---------------------------------------------------------------------------

void JntRev::getVarTrf(int idx, Trf3& trf) const
{
  trf.init();
  trf.isDerivative = false;

  if (idx != 0) return;

  double c = cos(varPos[0]);
  double s = sin(varPos[0]);

  trf(0,0) =  c; trf(0,1) = s;
  trf(1,0) = -s; trf(1,1) = c;
}

//---------------------------------------------------------------------------

void JntRev::getVarDerTrf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx != 0) return;

  double c = cos(varPos[0]);
  double s = sin(varPos[0]);

  trf(0,0) = -s; trf(0,1) =  c;
  trf(1,0) = -c; trf(1,1) = -s;
}

//---------------------------------------------------------------------------

void JntRev::getVarDer2Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx != 0) return;

  double c = cos(varPos[0]);
  double s = sin(varPos[0]);

  trf(0,0) = -c; trf(0,1) = -s;
  trf(1,0) =  s; trf(1,1) = -c;
}

//---------------------------------------------------------------------------

void JntRev::getVarDer3Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx != 0) return;

  double c = cos(varPos[0]);
  double s = sin(varPos[0]);

  trf(0,0) =  s; trf(0,1) = -c;
  trf(1,0) =  c; trf(1,1) =  s;
}

//---------------------------------------------------------------------------

void JntRev::initVarsFromPos(bool fixedAlso)
{
  calcPosFromNeighbours();

  const Trf3& trf = getPos();

  if (fixedAlso || !getFixed(0)) {
    varPos[0] = atan2(trf(0,1),trf(0,0));
  }

  setModelModified();
}

} // namespace

//---------------------------------------------------------------------------
