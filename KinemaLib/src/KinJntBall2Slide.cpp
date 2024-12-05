//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Ball Joint + 2 slides (5 degrees of freedom) ---------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinJntBall2Slide.h"

#include <cmath>

// ATTN: Joint is unstable if all rotations are near zero!

using namespace Ino;

namespace InoKin {

//-------------------------------------------------------------------------------

AbstractJoint *JntBall2Slide::clone(Grip& newGrip) const
{
  return new JntBall2Slide(newGrip,*this);
}

//-------------------------------------------------------------------------------

JntBall2Slide::JntBall2Slide(Grip& grp, const wchar_t *name)
: AbstractJoint(grp,name,5)
{
  isAngular[0] = false;
  isAngular[1] = true;
  isAngular[2] = true;
  isAngular[3] = true;
  isAngular[4] = false;
}

//-------------------------------------------------------------------------------

JntBall2Slide::JntBall2Slide(Grip& grp, const JntBall2Slide& cp)
: AbstractJoint(grp,cp)
{
}

//-------------------------------------------------------------------------------

void JntBall2Slide::getVarTrf(int idx, Trf3& trf) const
{
  trf.init();
  trf.isDerivative = false;

  switch (idx) {
    case 0:
      trf(0,0) = 1;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 1; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 1; trf(2,3) = varPos[0];
    break;

    case 1: {
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) =  ca;  trf(0,1) = sa; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = -sa;  trf(1,1) = ca; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =  0; trf(2,2) = 1; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) =  cb;  trf(0,1) =  0; trf(0,2) = -sb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  1; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) =  sb;  trf(2,1) =  0; trf(2,2) =  cb; trf(2,3) = 0;
    }
    break;

    case 3: {
      double cc = cos(varPos[3]);
      double sc = sin(varPos[3]);

      trf(0,0) = 1;  trf(0,1) =   0; trf(0,2) =  0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) =  cc; trf(1,2) = sc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = -sc; trf(2,2) = cc; trf(2,3) = 0;
    }
    break;

    case 4:
      trf(0,0) = 1;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 1; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 1; trf(2,3) = varPos[4];
    break;
  }
}

//-------------------------------------------------------------------------------

void JntBall2Slide::getVarDerTrf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  switch (idx) {
    case 0:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 1;
    break;

    case 1: {
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) = -sa;  trf(0,1) =  ca; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = -ca;  trf(1,1) = -sa; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) = -sb;  trf(0,1) =  0; trf(0,2) = -cb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) =  cb;  trf(2,1) =  0; trf(2,2) = -sb; trf(2,3) = 0;
    }
    break;

    case 3: {
      double cc = cos(varPos[3]);
      double sc = sin(varPos[3]);

      trf(0,0) = 0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = -sc; trf(1,2) =  cc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = -cc; trf(2,2) = -sc; trf(2,3) = 0;
    }
    break;

    case 4:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 1;
    break;
  }
}

//-------------------------------------------------------------------------------

void JntBall2Slide::getVarDer2Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  switch (idx) {
    case 0:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    break;

    case 1: {
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) = -ca;  trf(0,1) = -sa; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) =  sa;  trf(1,1) = -ca; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) = -cb;  trf(0,1) =  0; trf(0,2) =  sb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) = -sb;  trf(2,1) =  0; trf(2,2) = -cb; trf(2,3) = 0;
    }
    break;

    case 3: {
      double cc = cos(varPos[3]);
      double sc = sin(varPos[3]);

      trf(0,0) = 0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = -cc; trf(1,2) = -sc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) =  sc; trf(2,2) = -cc; trf(2,3) = 0;
    }
    break;

    case 4:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    break;
  }
}

//-------------------------------------------------------------------------------

void JntBall2Slide::getVarDer3Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  switch (idx) {
    case 0:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    break;

    case 1: {
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) =  sa;  trf(0,1) = -ca; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) =  ca;  trf(1,1) =  sa; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;

    case 2: {
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) =  sb;  trf(0,1) =  0; trf(0,2) =  cb; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) = -cb;  trf(2,1) =  0; trf(2,2) =  sb; trf(2,3) = 0;
    }
    break;

    case 3: {
      double cc = cos(varPos[3]);
      double sc = sin(varPos[3]);

      trf(0,0) = 0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) =  sc; trf(1,2) = -cc; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) =  cc; trf(2,2) =  sc; trf(2,3) = 0;
    }
    break;

    case 4:
      trf(0,0) = 0;  trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0;  trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0;  trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    break;
  }
}

//-------------------------------------------------------------------------------

void JntBall2Slide::initVarsFromPos(bool fixedAlso)
{
  calcPosFromNeighbours();

  const Trf3& trf = getPos();

  if (fixedAlso || !getFixed(0)) {
    if (fabs(trf(0,2)) > fabs(trf(1,2))) {
      if (fabs(trf(0,2)) < 1e-12) varPos[0] = 0;
      else varPos[0] = trf(0,3)/trf(0,2);
    }
    else {
      if (fabs(trf(1,2)) < 1e-12) varPos[0] = 0;
      else varPos[0] = trf(1,3)/trf(1,2);
    }
  }

  if (fixedAlso || !getFixed(3)) {
    varPos[1] = atan2(trf(0,1),trf(0,0));
  }

  if (fixedAlso || !getFixed(3)) {
    varPos[3] = atan2(trf(1,2),trf(2,2));
  }

  if (fixedAlso || !getFixed(2)) {
    double ca = cos(varPos[1]);
    double sa = sin(varPos[1]);

    if (fabs(ca) > fabs(sa))
         varPos[2] = atan2(-trf(0,2),trf(0,0)/ca);
    else varPos[2] = atan2(-trf(0,2),trf(0,1)/sa);
  }

  if (fixedAlso || !getFixed(4)) {
    varPos[4] = trf(2,3)-trf(2,2)*varPos[0];
  }

  setModelModified();
}

} // namespace

void* JointBall2SlideNew(void* cppGrip, const wchar_t* name) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  InoKin::JntBall2Slide* jnt = new InoKin::JntBall2Slide(*grip, name);

  return jnt;
}

//-------------------------------------------------------------------------------
