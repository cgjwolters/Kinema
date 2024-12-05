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

#include "KinJntBallSlide.h"

#include <cmath>

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

AbstractJoint *JntBallSlide::clone(Grip& newGrip) const
{
  return new JntBallSlide(newGrip,*this);
}

//---------------------------------------------------------------------------

JntBallSlide::JntBallSlide(Grip& grp, const wchar_t *name)
: AbstractJoint(grp,name,4)
{
  isAngular[0] = true;
  isAngular[1] = true;
  isAngular[2] = true;
  isAngular[3] = false;
}

//---------------------------------------------------------------------------

JntBallSlide::JntBallSlide(Grip& grp, const JntBallSlide& cp)
: AbstractJoint(grp,cp)
{
}

//---------------------------------------------------------------------------

void JntBallSlide::getVarTrf(int idx, Trf3& trf) const
{
  trf.init();
  trf.isDerivative = false;

  switch (idx) {
    case 0: {
      double ca = cos(varPos[0]);
      double sa = sin(varPos[0]);

      trf(0,0) =  ca;  trf(0,1) = sa; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = -sa;  trf(1,1) = ca; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =  0; trf(2,2) = 1; trf(2,3) = 0;
    }
    break;

    case 1: {
      double cb = cos(varPos[1]);
      double sb = sin(varPos[1]);

      trf(0,0) =  cb;  trf(0,1) =  0; trf(0,2) = -sb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  1; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) =  sb;  trf(2,1) =  0; trf(2,2) =  cb; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cc = cos(varPos[2]);
      double sc = sin(varPos[2]);

      trf(0,0) = 1;  trf(0,1) =   0; trf(0,2) =  0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) =  cc; trf(1,2) = sc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = -sc; trf(2,2) = cc; trf(2,3) = 0;
    }
    break;

    case 3:
      trf(0,0) = 1;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 1; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 1; trf(2,3) = varPos[3];
    break;
  }
}

//---------------------------------------------------------------------------

void JntBallSlide::getVarDerTrf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = false;

  switch (idx) {
    case 0: {
      double ca = cos(varPos[0]);
      double sa = sin(varPos[0]);

      trf(0,0) = -sa;  trf(0,1) =  ca; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = -ca;  trf(1,1) = -sa; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;

    case 1: {
      double cb = cos(varPos[1]);
      double sb = sin(varPos[1]);

      trf(0,0) = -sb;  trf(0,1) =  0; trf(0,2) = -cb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) =  cb;  trf(2,1) =  0; trf(2,2) = -sb; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cc = cos(varPos[2]);
      double sc = sin(varPos[2]);

      trf(0,0) = 0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = -sc; trf(1,2) =  cc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = -cc; trf(2,2) = -sc; trf(2,3) = 0;
    }
    break;

    case 3:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 1;
    break;
  }
}

//---------------------------------------------------------------------------

void JntBallSlide::getVarDer2Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = false;

  switch (idx) {
    case 0: {
      double ca = cos(varPos[0]);
      double sa = sin(varPos[0]);

      trf(0,0) = -ca;  trf(0,1) = -sa; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) =  sa;  trf(1,1) = -ca; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;

    case 1: {
      double cb = cos(varPos[1]);
      double sb = sin(varPos[1]);

      trf(0,0) = -cb;  trf(0,1) =  0; trf(0,2) =  sb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) = -sb;  trf(2,1) =  0; trf(2,2) = -cb; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cc = cos(varPos[2]);
      double sc = sin(varPos[2]);

      trf(0,0) = 0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = -cc; trf(1,2) = -sc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) =  sc; trf(2,2) = -cc; trf(2,3) = 0;
    }
    break;

    case 3:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    break;
  }
}

//---------------------------------------------------------------------------

void JntBallSlide::getVarDer3Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = false;

  switch (idx) {
    case 0: {
      double ca = cos(varPos[0]);
      double sa = sin(varPos[0]);

      trf(0,0) =  sa;  trf(0,1) = -ca; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) =  ca;  trf(1,1) =  sa; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;

    case 1: {
      double cb = cos(varPos[1]);
      double sb = sin(varPos[1]);

      trf(0,0) =  sb;  trf(0,1) =  0; trf(0,2) =  cb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) = -cb;  trf(2,1) =  0; trf(2,2) =  sb; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cc = cos(varPos[2]);
      double sc = sin(varPos[2]);

      trf(0,0) = 0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) =  sc; trf(1,2) = -cc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) =  cc; trf(2,2) =  sc; trf(2,3) = 0;
    }
    break;

    case 3:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    break;
  }
}

//---------------------------------------------------------------------------

void JntBallSlide::initVarsFromPos(bool fixedAlso)
{
  calcPosFromNeighbours();

  const Trf3& trf = getPos();

  if (fixedAlso || !getFixed(0)) {
    varPos[0] = atan2(trf(0,1),trf(0,0));
  }

  if (fixedAlso || !getFixed(2)) {
    varPos[2] = atan2(trf(1,2),trf(2,2));
  }

  if (fixedAlso || !getFixed(1)) {
    double ca = cos(varPos[0]);
    double sa = sin(varPos[0]);

    if (fabs(ca) > fabs(sa))
         varPos[1] = atan2(-trf(0,2),trf(0,0)/ca);
    else varPos[1] = atan2(-trf(0,2),trf(0,1)/sa);
  }

  if (fixedAlso || !getFixed(3)) {
    varPos[3] = trf(2,3);
  }

  setModelModified();
}

} // namespace

void* JointBallSlideNew(void* cppGrip, const wchar_t* name) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  InoKin::JntBallSlide* jnt = new InoKin::JntBallSlide(*grip, name);

  return jnt;
}

//-------------------------------------------------------------------------------
