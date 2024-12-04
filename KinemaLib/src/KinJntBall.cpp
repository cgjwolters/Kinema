//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Ball Joint (3 degrees (rotations) of freedom ---------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinJntBall.h"

#include <cmath>

using namespace Ino;

namespace InoKin {

//-------------------------------------------------------------------------------

AbstractJoint *JntBall::clone(Grip& newGrip) const
{
  return new JntBall(newGrip,*this);
}

//-------------------------------------------------------------------------------

JntBall::JntBall(Grip& grp, const wchar_t *name)
: AbstractJoint(grp,name,3)
{
  isAngular[0] = true;
  isAngular[1] = true;
  isAngular[2] = true;
}

//-------------------------------------------------------------------------------

JntBall::JntBall(Grip& grp, const JntBall& cp)
: AbstractJoint(grp,cp)
{
}

//-------------------------------------------------------------------------------

void JntBall::getVarTrf(int idx, Trf3& trf) const
{
  trf.init();
  trf.isDerivative = false;

  if (idx == 0) {
    double ca = cos(varPos[0]);
    double sa = sin(varPos[0]);

    trf(0,0) =  ca;  trf(0,1) = sa; trf(0,2) = 0; trf(0,3) = 0;
    trf(1,0) = -sa;  trf(1,1) = ca; trf(1,2) = 0; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) =  0; trf(2,2) = 1; trf(2,3) = 0;
  }
  else if (idx == 1) {
    double cb = cos(varPos[1]);
    double sb = sin(varPos[1]);

    trf(0,0) =  cb;  trf(0,1) =  0; trf(0,2) = -sb; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) =  1; trf(1,2) =   0; trf(1,3) = 0;
    trf(2,0) =  sb;  trf(2,1) =  0; trf(2,2) =  cb; trf(2,3) = 0;
  }
  else if (idx == 2) {
    double cc = cos(varPos[2]);
    double sc = sin(varPos[2]);

    trf(0,0) =   1;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) =  cc; trf(1,2) =  sc; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) = -sc; trf(2,2) =  cc; trf(2,3) = 0;
  }
}

//-------------------------------------------------------------------------------

void JntBall::getVarDerTrf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx == 0) {
    double ca = cos(varPos[0]);
    double sa = sin(varPos[0]);

    trf(0,0) = -sa;  trf(0,1) =  ca; trf(0,2) = 0; trf(0,3) = 0;
    trf(1,0) = -ca;  trf(1,1) = -sa; trf(1,2) = 0; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
  }
  else if (idx == 1) {
    double cb = cos(varPos[1]);
    double sb = sin(varPos[1]);

    trf(0,0) = -sb;  trf(0,1) =  0; trf(0,2) = -cb; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
    trf(2,0) =  cb;  trf(2,1) =  0; trf(2,2) = -sb; trf(2,3) = 0;
  }
  else if (idx == 2) {
    double cc = cos(varPos[2]);
    double sc = sin(varPos[2]);

    trf(0,0) =   0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) = -sc; trf(1,2) =  cc; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) = -cc; trf(2,2) = -sc; trf(2,3) = 0;
  }
}

//-------------------------------------------------------------------------------

void JntBall::getVarDer2Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx == 0) {
    double ca = cos(varPos[0]);
    double sa = sin(varPos[0]);

    trf(0,0) = -ca;  trf(0,1) = -sa; trf(0,2) = 0; trf(0,3) = 0;
    trf(1,0) =  sa;  trf(1,1) = -ca; trf(1,2) = 0; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
  }
  else if (idx == 1) {
    double cb = cos(varPos[1]);
    double sb = sin(varPos[1]);

    trf(0,0) = -cb;  trf(0,1) =  0; trf(0,2) =  sb; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
    trf(2,0) = -sb;  trf(2,1) =  0; trf(2,2) = -cb; trf(2,3) = 0;
  }
  else if (idx == 2) {
    double cc = cos(varPos[2]);
    double sc = sin(varPos[2]);

    trf(0,0) =   0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) = -cc; trf(1,2) = -sc; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) =  sc; trf(2,2) = -cc; trf(2,3) = 0;
  }
}

//-------------------------------------------------------------------------------

void JntBall::getVarDer3Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  if (idx == 0) {
    double ca = cos(varPos[0]);
    double sa = sin(varPos[0]);

    trf(0,0) =  sa;  trf(0,1) = -ca; trf(0,2) = 0; trf(0,3) = 0;
    trf(1,0) =  ca;  trf(1,1) =  sa; trf(1,2) = 0; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) =   0; trf(2,2) = 0; trf(2,3) = 0;
  }
  else if (idx == 1) {
    double cb = cos(varPos[1]);
    double sb = sin(varPos[1]);

    trf(0,0) =  sb;  trf(0,1) =  0; trf(0,2) =  cb; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) =  0; trf(1,2) =   0; trf(1,3) = 0;
    trf(2,0) = -cb;  trf(2,1) =  0; trf(2,2) =  sb; trf(2,3) = 0;
  }
  else if (idx == 2) {
    double cc = cos(varPos[2]);
    double sc = sin(varPos[2]);

    trf(0,0) =   0;  trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
    trf(1,0) =   0;  trf(1,1) =  sc; trf(1,2) = -cc; trf(1,3) = 0;
    trf(2,0) =   0;  trf(2,1) =  cc; trf(2,2) =  sc; trf(2,3) = 0;
  }
}

//-------------------------------------------------------------------------------

void JntBall::initVarsFromPos(bool fixedAlso)
{
  calcPosFromNeighbours();

  const Trf3& trf = getPos();

  if (fixedAlso || !getFixed(0)) {
    varPos[0] = atan2(trf(0,1),trf(0,0));
  }

  if (fixedAlso || !getFixed(1)) {
    double ca = cos(varPos[0]);
    double sa = sin(varPos[0]);

    if (fabs(ca) > fabs(sa))
         varPos[1] = atan2(-trf(0,2),trf(0,0)/ca);
    else varPos[1] = atan2(-trf(0,2),trf(0,1)/sa);
  }

  if (fixedAlso || !getFixed(2)) {
    varPos[2] = atan2(trf(1,2),trf(2,2));
  }

  setModelModified();
}

} // namespace

// Interface Section

void* JointBallNew(void* cppGrip, const wchar_t* name) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  InoKin::JntBall* jnt = new InoKin::JntBall(*grip, name);

  return jnt;
}

//-------------------------------------------------------------------------------
