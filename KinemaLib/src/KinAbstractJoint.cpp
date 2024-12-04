//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------- Kinema: Kinematic Simulation Program ---------------------------
//-------------------------------------------------------------------------------
//------------------------ Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//------------------------------------------------------ C.Wolters --------------
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------- Abstract Joint Base Class --------------------------------------
//-------------------------------------------------------------------------------

#include "KinAbstractJoint.h"

#include "KinBody.h"
#include "KinGrip.h"
#include "KinFunction.h"
#include "KinModel.h"

#include "Matrix.h"

//#include "stdio.h"

using namespace Ino;

namespace InoKin {

//-------------------------------------------------------------------------------

void AbstractJoint::getSpeedTrf(int idx, Trf3& spTrf) const
{
  getVarDerTrf(idx,spTrf);
  spTrf *= varSpeed[idx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::getAccelTrf(int idx, Trf3& accTrf) const
{
  getVarDer2Trf(idx,accTrf);
  accTrf *= sqr(varSpeed[idx]);

  Trf3 derTrf;
  getVarDerTrf(idx,derTrf);
  derTrf *= varAccel[idx];

  accTrf += derTrf;
}

//-------------------------------------------------------------------------------

void AbstractJoint::getJerkTrf(int idx, Trf3& jerkTrf) const
{
  Trf3 derTrf;
  getVarDerTrf(idx,derTrf);
  derTrf *= varJerk[idx];

  Trf3 accTrf;
  getVarDer2Trf(idx,accTrf);
  accTrf *= varSpeed[idx] * varAccel[idx];

  getVarDer3Trf(idx,jerkTrf);
  jerkTrf *= sqr(varSpeed[idx]) * varSpeed[idx];

  jerkTrf += derTrf;
  jerkTrf += accTrf;
}

//-------------------------------------------------------------------------------

void AbstractJoint::setPos()
{
  pos.init();

  Trf3 trf;

  for (int i=0; i<varCnt; i++) {
    getVarTrf(i,trf);
    pos.preMultWith(trf);
  }

  pos.invertInto(invPos);
}

//-------------------------------------------------------------------------------

void AbstractJoint::setSpeed()
{
  der1.zero();
  invDer1.zero();

  Trf3 derTrf,trf,curTrf;

  for (int i=0; i<varCnt; i++) {
    getVarTrf(i,trf);
    der1.preMultWith(trf);

    getSpeedTrf(i,derTrf);
    derTrf *= curTrf;

    der1 += derTrf;

    curTrf.preMultWith(trf);
  }

  invDer1 = der1;
  invDer1 *= invPos;
  invDer1.preMultWith(invPos);
  invDer1 *= -1.0;
}

//-------------------------------------------------------------------------------

void AbstractJoint::setAccel()
{
  der2.zero();
  invDer2.zero();

  Trf3 accTrf,trf,curTrf;

  for (int i=0; i<varCnt; i++) {
    getVarTrf(i,trf);
    der2.preMultWith(trf);

    getAccelTrf(i,accTrf);
    accTrf *= curTrf;

    der2 += accTrf;

    curTrf.preMultWith(trf);
  }

  getAccMixed(trf);

  der2 += trf;

  invDer2 = invPos;
  invDer2 *= der1;
  invDer2.preMultWith(der1);
  invDer2 *= 2.0;

  invDer2 -= der2;
  invDer2 *= invPos;
  invDer2.preMultWith(invPos);
}

//-------------------------------------------------------------------------------

void AbstractJoint::setJerk()
{
  der3.zero();
  invDer3.zero();

  Trf3 jerkTrf,trf,curTrf;

  for (int i=0; i<varCnt; i++) {
    getVarTrf(i,trf);
    der3.preMultWith(trf);

    getJerkTrf(i,jerkTrf);
    jerkTrf *= curTrf;

    der3 += jerkTrf;

    curTrf.preMultWith(trf);
  }

  getJerkMixed(trf);
  der3 += trf;

  Trf3 dTrf(der1); dTrf *= invPos;
  Trf3 aTrf(der2); aTrf *= invPos;

  invDer3 = dTrf; invDer3 *= dTrf; invDer3 *= 2.0;
  invDer3 += aTrf; 
  invDer3.preMultWith(dTrf);

  aTrf *= dTrf;
  invDer3 += aTrf;
  invDer3 *= 3.0;

  jerkTrf = der3; jerkTrf *= invPos;
  invDer3 -= jerkTrf;
  invDer3.preMultWith(invPos);
}

//-------------------------------------------------------------------------------

AbstractJoint::AbstractJoint(Grip& grp, const wchar_t *name, int nrVars)
: Object(grp.model,name),
  pos(), invPos(), der1(), invDer1(),
  der2(), invDer2(), der3(), invDer3(),
  fixedPos(new bool[nrVars]), varIdx(new int[nrVars]),
  derLst(new Trf3[nrVars]), derLstValid(false),
  invDerLst(new Trf3[nrVars]), invDerLstValid(new bool[nrVars]),
  varPos(new double[nrVars]), varSpeed(new double[nrVars]),
  varAccel(new double[nrVars]), varJerk(new double[nrVars]),
  isAngular(new bool[nrVars]), varCnt(nrVars), grip(grp)
{
  delete grip.joint;
  grip.joint = this;

  for (int i=0; i<varCnt; i++) {
    invDerLstValid[i] = false;
    varPos[i]         = 0.0;
    varSpeed[i]       = 0.0;
    varAccel[i]       = 0.0;
    fixedPos[i]       = false;
    varIdx[i]         = -1;
    isAngular[i]      = false;
  }

  der1.zero();    der1.isDerivative    = true;
  invDer1.zero(); invDer1.isDerivative = true;
  der2.zero();    der2.isDerivative    = true;
  invDer2.zero(); invDer2.isDerivative = true;
  der3.zero();    der3.isDerivative    = true;
  invDer3.zero(); invDer3.isDerivative = true;

  setModelTopoModified();
}

//-------------------------------------------------------------------------------

AbstractJoint::AbstractJoint(Grip& newGrp, const AbstractJoint& cp)
: Object(newGrp.model,cp),
  der1(cp.der1), invDer1(cp.invDer1),
  der2(cp.der2), invDer2(cp.invDer2),
  der3(cp.der3), invDer3(cp.invDer3),
  fixedPos(new bool[cp.varCnt]),
  varIdx(new int[cp.varCnt]), pos(cp.pos), invPos(cp.invPos),
  derLst(new Trf3[cp.varCnt]), derLstValid(false),
  invDerLst(new Trf3[cp.varCnt]), invDerLstValid(new bool[cp.varCnt]),
  varPos(new double[cp.varCnt]), varSpeed(new double[cp.varCnt]),
  varAccel(new double[cp.varCnt]), varJerk(new double[cp.varCnt]),
  isAngular(new bool[cp.varCnt]), varCnt(cp.varCnt), grip(newGrp)
{
  delete grip.joint;
  grip.joint = this;

  for (int i=0; i<varCnt; i++) {
    invDerLstValid[i] = cp.invDerLstValid[i];
    varPos[i]         = cp.varPos[i];
    varSpeed[i]       = cp.varSpeed[i];
    varAccel[i]       = cp.varAccel[i];
    varJerk[i]        = cp.varJerk[i];
    fixedPos[i]       = cp.fixedPos[i];
    varIdx[i]         = cp.varIdx[i];
    isAngular[i]      = cp.isAngular[i];
  }

  if (&newGrp.model != &cp.model) setModelTopoModified();
}

//-------------------------------------------------------------------------------

AbstractJoint::~AbstractJoint()
{
  delete[] fixedPos;
  delete[] varIdx;
  delete[] derLst;
  delete[] invDerLst;
  delete[] invDerLstValid;
  delete[] varPos;
  delete[] varSpeed;
  delete[] varAccel;
  delete[] varJerk;
  delete[] isAngular;

  Function *func = model.getFunctionList().findByConnection(*this);
  delete func;

  if (grip.joint == this) grip.joint = NULL;

  setModelTopoModified();
}

//-------------------------------------------------------------------------------

int AbstractJoint::getVarCnt(bool fixd) const
{
  int fc = 0;

  for (int i=0; i<varCnt; i++) {
    if (fixedPos[i] == fixd) fc++;
  }

  return fc;
}

//-------------------------------------------------------------------------------

bool AbstractJoint::getFixed(int locIdx) const
{
  if (locIdx < 0 || locIdx > varCnt) return false;

  return fixedPos[locIdx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::setFixed(int locIdx, bool isFixed)
{
  if (locIdx < 0 || locIdx > varCnt) return;

  fixedPos[locIdx] = isFixed;

  setModelTopoModified();
}

//-------------------------------------------------------------------------------

void AbstractJoint::setFixedAll(bool isFixed)
{
  for (int i=0; i<varCnt; i++) fixedPos[i] = isFixed;

  setModelTopoModified();
}

//-------------------------------------------------------------------------------

bool AbstractJoint::getIsAngular(int locIdx) const
{
  if (locIdx < 0 || locIdx >= varCnt) return false;

  return isAngular[locIdx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::clearVarIndices()
{
  for (int i=0; i<varCnt; ++i) varIdx[i] = -1;
}

//-------------------------------------------------------------------------------

int AbstractJoint::getVarIdx(int locIdx) const
{
  if (locIdx < 0 || locIdx >= varCnt)
    return -1;

  return varIdx[locIdx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::setVarIdx(int locIdx, int var_idx)
{
  if (locIdx < 0 || locIdx >= varCnt) return;

  varIdx[locIdx] = var_idx;

  setModelTopoModified();
}

//-------------------------------------------------------------------------------

int AbstractJoint::getLocIdx(int vIdx) const
{
  if (vIdx < 1) return -1;

  for (int i=0; i<varCnt; i++) {
    if (varIdx[i] == vIdx) return i;
  }

  return -1;
}

//-------------------------------------------------------------------------------

double AbstractJoint::getVal(int locIdx) const
{
  if (locIdx < 0 || locIdx >= varCnt) return 0.0;

  return varPos[locIdx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::setVal(int locIdx, double newVal)
{
  if (locIdx < 0 || locIdx >= varCnt) return;

  varPos[locIdx] = newVal;
  setPos();

  setModelModified();
}

//-------------------------------------------------------------------------------

void AbstractJoint::getVars(bool isFixed, Vector& varVec) const
{
  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) varVec[varIdx[i]] = varPos[i];
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::setVars(bool isFixed, const Vector& varVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) {
      varPos[i] = varVec[varIdx[i]];
      set = true;
    }
  }

  if (set) setPos();
}

//-------------------------------------------------------------------------------

void AbstractJoint::setVars(const Vector& varVec, const Vector& fixedVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (varIdx[i] < 0) continue;

    if (fixedPos[i]) 
         varPos[i] = fixedVec[varIdx[i]];
    else varPos[i] = varVec[varIdx[i]];

    set = true;
  }

  if (set) setPos();
}

//-------------------------------------------------------------------------------

double AbstractJoint::getSpeed(int locIdx) const
{
  if (locIdx < 0 || locIdx >= varCnt) return 0.0;

  return varSpeed[locIdx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::setSpeed(int locIdx, double newSpeed)
{
  if (locIdx < 0 || locIdx >= varCnt) return;

  varSpeed[locIdx] = newSpeed;

  setSpeed();

  setModelModified();
}

//-------------------------------------------------------------------------------

void AbstractJoint::getSpeeds(bool isFixed, Vector& speedVec) const
{
  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) {
      speedVec[varIdx[i]] = varSpeed[i];
    }
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::setSpeeds(bool isFixed, const Vector& speedVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) {
      varSpeed[i] = speedVec[varIdx[i]];
      set = true;
    }
  }

  if (set) setSpeed();
}

//-------------------------------------------------------------------------------

void AbstractJoint::setSpeeds(const Vector& varSpeedVec,
                                                  const Vector& fixedSpeedVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (varIdx[i] < 0) continue;

    if (fixedPos[i]) 
         varSpeed[i] = fixedSpeedVec[varIdx[i]];
    else varSpeed[i] = varSpeedVec[varIdx[i]];

    set = true;
  }

  if (set) setSpeed();
}

//-------------------------------------------------------------------------------

void AbstractJoint::clearTrfCaches()
{
  derLstValid = false;
  for (int i=0; i<varCnt; ++i) invDerLstValid[i] = false;
}

//-------------------------------------------------------------------------------

const Trf3& AbstractJoint::getDerivative(int locIdx) const
{
  if (!derLstValid) getDerivativeAll();

  return derLst[locIdx];
}

//-------------------------------------------------------------------------------

const Trf3& AbstractJoint::getInvDerivative(int locIdx) const
{
  if (!derLstValid) getDerivativeAll();

  if (!invDerLstValid[locIdx]) {
    invDerLst[locIdx] = derLst[locIdx];

    Trf3& trf = invDerLst[locIdx];

    trf *= invPos;
    trf.preMultWith(invPos);
    trf *= -1.0;

    invDerLstValid[locIdx] = true;
  }

  return invDerLst[locIdx];
}

//-------------------------------------------------------------------------------
// Minimizing the number of Trf3 multiplies

void AbstractJoint::getDerivativeAll() const
{
  derLst[0].init();

  for (int i=1; i<varCnt; i++) {
    getVarTrf(i-1,derLst[i]);
    if (i>1) derLst[i] *= derLst[i-1];
  }

  Trf3 trf;

  for (int i=0; i<varCnt; i++) {
    getVarDerTrf(i,trf);
    derLst[i].preMultWith(trf);
  }

  Trf3 eTrf;

  if (varCnt >= 2) {
    getVarTrf(varCnt-1,eTrf);
    derLst[varCnt-2].preMultWith(eTrf);
  }

  for (int i=varCnt-3; i>=0; --i) {
    getVarTrf(i+1,trf);
    eTrf *= trf; 

    derLst[i].preMultWith(eTrf);
  }

  derLstValid = true;
}

//-------------------------------------------------------------------------------

void AbstractJoint::getAccMixed(Trf3& mixTrf) const
{
  mixTrf.zero();
  mixTrf.isDerivative = true;

  Trf3 subTrf1,derTrf1,trf1,curTrf1;

  subTrf1.zero();
  subTrf1.isDerivative = true;

  for (int i=0; i<varCnt-1; i++) {
    getVarTrf(i,trf1);
    subTrf1.preMultWith(trf1);

    getSpeedTrf(i,derTrf1);
    derTrf1 *= curTrf1;

    subTrf1 += derTrf1;
    curTrf1.preMultWith(trf1);

    Trf3 subTrf2,derTrf2,trf2,curTrf2;

    subTrf2.zero();
    subTrf2.isDerivative = true;

    for (int j=i+1; j<varCnt; ++j) {
      getVarTrf(j,trf2);
      subTrf2.preMultWith(trf2);

      getSpeedTrf(i,derTrf2);
      derTrf2 *= curTrf2;

      subTrf2 += derTrf2;
      curTrf2.preMultWith(trf2);
    }

    subTrf2 *= subTrf1;
    mixTrf += subTrf2;
  }

  mixTrf *= 2.0;
}

//-------------------------------------------------------------------------------

void AbstractJoint::getInvAccMixed(Trf3& invMixTrf) const
{
  invMixTrf.zero();
  invMixTrf.isDerivative = true;

  Trf3 subTrf1,derTrf1,trf1,curTrf1;

  subTrf1.zero();
  subTrf1.isDerivative = true;

  for (int i=0; i<varCnt-1; i++) {
    getVarTrf(i,trf1); trf1.invert();
    subTrf1 *= trf1;

    getSpeedTrf(i,derTrf1);
    derTrf1 *= trf1; derTrf1.preMultWith(trf1); derTrf1 *= -1.0;

    derTrf1.preMultWith(curTrf1);

    subTrf1 += derTrf1;
    curTrf1 *= trf1;

    Trf3 subTrf2,derTrf2,trf2,curTrf2;

    subTrf2.zero();
    subTrf2.isDerivative = true;

    for (int j=i+1; j<varCnt; ++j) {
      getVarTrf(j,trf2); trf2.invert();
      subTrf2 *= trf2;

      getSpeedTrf(i,derTrf2);

      derTrf2 *= trf2; derTrf2.preMultWith(trf2); derTrf2 *= -1.0;
      derTrf2.preMultWith(curTrf2);

      subTrf2 += derTrf2;
      curTrf2 *= trf2;
    }

    subTrf2.preMultWith(subTrf1);
    invMixTrf += subTrf2;
  }

  invMixTrf *= 2.0;
}

//-------------------------------------------------------------------------------

double AbstractJoint::getAccel(int locIdx) const
{
  if (locIdx < 0 || locIdx >= varCnt) return 0.0;

  return varAccel[locIdx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::setAccel(int locIdx, double newAccel)
{
  if (locIdx < 0 || locIdx >= varCnt) return;

  varAccel[locIdx] = newAccel;
  setAccel();
  setModelModified();
}

//-------------------------------------------------------------------------------

void AbstractJoint::getAccels(bool isFixed, Vector& accelVec) const
{
  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) {
      accelVec[varIdx[i]] = varAccel[i];
    }
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::setAccels(bool isFixed, const Vector& accelVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) {
      varAccel[i] = accelVec[varIdx[i]];
      set = true;
    }
  }

  if (set) setAccel();
}

//-------------------------------------------------------------------------------

void AbstractJoint::setAccels(const Vector& varAccelVec,
                                          const Vector& fixedAccelVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (varIdx[i] < 0) continue;

    if (fixedPos[i])
         varAccel[i] = fixedAccelVec[varIdx[i]];
    else varAccel[i] = varAccelVec[varIdx[i]];

    set = true;
  }

  if (set) setAccel();
}

//-------------------------------------------------------------------------------

void AbstractJoint::getSecDerivative(int locIdx, Trf3& accTrf) const
{
  accTrf.zero();
  accTrf.isDerivative = true;

  if (locIdx < 0 || locIdx > varCnt) return;

  accTrf.init();

  Trf3 trf;

  for (int i=0; i<varCnt; i++) {
    if (i == locIdx) getVarDer2Trf(i,trf);
    else getVarTrf(i,trf);

    accTrf.preMultWith(trf);
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getInvSecDerivative(int locIdx, Trf3& accTrf) const
{
  accTrf.zero();
  accTrf.isDerivative = true;

  if (locIdx < 0 || locIdx >= varCnt) return;

  const Trf3& dTrf = getDerivative(locIdx);

  accTrf = invPos;
  accTrf *= dTrf;
  accTrf.preMultWith(dTrf);
  accTrf *= 2.0;

  Trf3 aTrf;
  getSecDerivative(locIdx,aTrf);

  accTrf -= aTrf;
  accTrf *= invPos;
  accTrf.preMultWith(invPos);
}

//-------------------------------------------------------------------------------

void AbstractJoint::getPosRange(int lwbIdx, int upbIdx, Trf3& posTrf) const
{
  posTrf.init();

  Trf3 trf;

  for (int i=lwbIdx; i<upbIdx; ++i) {
    getVarTrf(i,trf);
    posTrf.preMultWith(trf);
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getInvPosRange(int lwbIdx, int upbIdx, Trf3& posTrf) const
{
  posTrf.init();

  Trf3 trf;

  for (int i=lwbIdx; i<upbIdx; ++i) {
    getVarTrf(i,trf); trf.invert();
    posTrf *= trf;
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getOneDerRange(int lwbIdx, int upbIdx, Trf3& derTrf) const
{
  derTrf.init();

  Trf3 trf,trf2;

  for (int i=lwbIdx; i<upbIdx; ++i) {
    getPosRange(0,i,trf);

    getSpeedTrf(i,trf2);
    trf.preMultWith(trf2);

    getPosRange(i+1,upbIdx,trf2);
    trf.preMultWith(trf2);

    derTrf += trf;
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getInvOneDerRange(int lwbIdx, int upbIdx, Trf3& derTrf) const
{
  derTrf.init();

  Trf3 trf,trf1,trf2;

  for (int i=lwbIdx; i<upbIdx; ++i) {
    getInvPosRange(0,i,trf);

    getVarTrf(i,trf1); trf1.invert();

    getSpeedTrf(i,trf2);
    trf2 *= trf1; trf2.preMultWith(trf1); trf2 *= -1.0;

    trf *= trf2;

    getInvPosRange(i+1,upbIdx,trf2);
    trf *= trf2;

    derTrf += trf;
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getTwoDerRange(int lwbIdx, int upbIdx, Trf3& derTrf) const
{
  derTrf.init();

  Trf3 trf,trf2,spTrf;

  for (int i=lwbIdx; i<upbIdx-1; ++i) {
    getPosRange(0,i,trf);

    getSpeedTrf(i,spTrf);
    trf.preMultWith(spTrf);

    getOneDerRange(i+1,upbIdx,trf2);
    trf.preMultWith(trf2);

    derTrf += trf;
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getInvTwoDerRange(int lwbIdx, int upbIdx, Trf3& derTrf) const
{
  derTrf.init();

  Trf3 trf,trf2,spTrf;

  for (int i=lwbIdx; i<upbIdx-1; ++i) {
    getInvPosRange(0,i,trf);

    getVarTrf(i,trf2); trf2.invert();

    getSpeedTrf(i,spTrf);
    spTrf *= trf2; spTrf.preMultWith(trf2); spTrf *= -1.0;
    trf *= spTrf;

    getInvOneDerRange(i+1,upbIdx,trf2);
    trf *= trf2;

    derTrf += trf;
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getJerkMixed(Trf3& mixTrf)
{
  mixTrf.zero();
  mixTrf.isDerivative = true;

  Trf3 trf, trf2;

  for (int i=1; i<varCnt; i++) {
    getOneDerRange(0,i,trf);

    getAccelTrf(i,trf2);
    trf.preMultWith(trf2);

    getPosRange(i+1,varCnt,trf2);
    trf.preMultWith(trf2);
    trf *= 3.0;

    mixTrf += trf;
  }

  for (int i=0; i<varCnt-1; i++) {
    getPosRange(0,i,trf);

    getAccelTrf(i,trf2);
    trf.preMultWith(trf2);

    getOneDerRange(i+1,varCnt,trf2);
    trf.preMultWith(trf2);
    trf *= 3.0;

    mixTrf += trf;
  }

  for (int i=0; i<varCnt-2; i++) {
    getPosRange(0,i,trf);

    getSpeedTrf(i,trf2);
    trf.preMultWith(trf2);

    getTwoDerRange(i+1,varCnt,trf2);
    trf.preMultWith(trf2);
    trf *= 6.0;

    mixTrf += trf;
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getInvJerkMixed(Trf3& invMixTrf)
{
  invMixTrf.zero();
  invMixTrf.isDerivative = true;

  Trf3 trf, trf1, trf2, invPosTrf, spTrf;

  for (int i=1; i<varCnt; i++) {
    getInvOneDerRange(0,i,trf);

    getVarTrf(i,invPosTrf); invPosTrf.invert();
    getSpeedTrf(i,trf1);

    trf2 = invPosTrf;
    trf2 *= trf1;
    trf2.preMultWith(trf1);
    trf2 *= 2.0;

    getAccelTrf(i,trf1);
    trf2 -= trf1;

    trf2 *= invPosTrf;
    trf2.preMultWith(invPosTrf); // Now inverse accel trf

    trf *= trf2;

    getInvPosRange(i+1,varCnt,trf2);
    trf *= trf2;
    trf *= 3.0;

    invMixTrf += trf;
  }

  for (int i=0; i<varCnt-1; i++) {
    getInvPosRange(0,i,trf);

    getVarTrf(i,invPosTrf); invPosTrf.invert();
    getSpeedTrf(i,trf1);

    trf2 = invPosTrf;
    trf2 *= trf1;
    trf2.preMultWith(trf1);
    trf2 *= 2.0;

    getAccelTrf(i,trf1);
    trf2 -= trf1;

    trf2 *= invPosTrf;
    trf2.preMultWith(invPosTrf); // Now inverse accel trf

    trf *= trf2;

    getInvOneDerRange(i+1,varCnt,trf2);
    trf *= trf2;
    trf *= 3.0;

    invMixTrf += trf;
  }

  for (int i=0; i<varCnt-2; i++) {
    getInvPosRange(0,i,trf);

    getVarTrf(i,invPosTrf); invPosTrf.invert();
    getSpeedTrf(i,trf2);
    trf2 *= invPosTrf;
    trf2.preMultWith(invPosTrf);
    trf2 *= -1.0; // Now invspeed

    trf *= trf2;

    getInvTwoDerRange(i+1,varCnt,trf2);
    trf *= trf2;
    trf *= 6.0;

    invMixTrf += trf;
  }
}

//-------------------------------------------------------------------------------

double AbstractJoint::getJerk(int locIdx) const
{
  if (locIdx < 0 || locIdx >= varCnt) return 0.0;

  return varJerk[locIdx];
}

//-------------------------------------------------------------------------------

void AbstractJoint::setJerk(int locIdx, double newJerk)
{
  if (locIdx < 0 || locIdx >= varCnt) return;

  varJerk[locIdx] = newJerk;
  setJerk();
  setModelModified();
}

//-------------------------------------------------------------------------------

void AbstractJoint::getJerks(bool isFixed, Vector& jerkVec) const
{
  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) {
      jerkVec[varIdx[i]] = varJerk[i];
    }
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::setJerks(bool isFixed, const Vector& jerkVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (isFixed == fixedPos[i] && varIdx[i] >= 0) {
      varJerk[i] = jerkVec[varIdx[i]];
      set = true;
    }
  }

  if (set) setJerk();
}

//-------------------------------------------------------------------------------

void AbstractJoint::setJerks(const Vector& varJerkVec,
                                                const Vector& fixedJerkVec)
{
  bool set = false;

  for (int i=0; i<varCnt; i++) {
    if (varIdx[i] < 0) continue;

    if (fixedPos[i]) 
         varJerk[i] = fixedJerkVec[varIdx[i]];
    else varJerk[i] = varJerkVec[varIdx[i]];
    set = true;
  }

  if (set) setJerk();
}

//-------------------------------------------------------------------------------

void AbstractJoint::getThirdDerivative(int locIdx, Trf3& jerkTrf) const
{
  jerkTrf.zero();
  jerkTrf.isDerivative = true;

  if (locIdx < 0 || locIdx > varCnt) return;

  jerkTrf.init();

  Trf3 trf;

  for (int i=0; i<varCnt; i++) {
    if (i == locIdx) getVarDer3Trf(i,trf);
    else getVarTrf(i,trf);

    jerkTrf.preMultWith(trf);
  }
}

//-------------------------------------------------------------------------------

void AbstractJoint::getInvThirdDerivative(int locIdx, Trf3& jerkTrf) const
{
  jerkTrf.zero();
  jerkTrf.isDerivative = true;

  if (locIdx < 0 || locIdx >= varCnt) return;

  Trf3 dTrf = getDerivative(locIdx);

  const Trf3& dInvTrf = getInvDerivative(locIdx);

  Trf3 aTrf;
  getSecDerivative(locIdx,aTrf);

  Trf3 aInvTrf;
  getInvSecDerivative(locIdx,aInvTrf);

  getThirdDerivative(locIdx,jerkTrf);

  jerkTrf *= invPos;

  aTrf *= dInvTrf;
  dTrf *= aInvTrf;

  aTrf += dTrf;
  aTrf *= 3.0;

  jerkTrf += aTrf;

  jerkTrf.preMultWith(invPos);
  jerkTrf *= -1.0;
}

//-------------------------------------------------------------------------------

void AbstractJoint::calcPosFromNeighbours()
{
  grip.getPos1().invertInto(pos);

  Trf3 trf;
  const Body *body1 = grip.getBody1();
  if (body1) body1->getPos().invertInto(trf);

  pos.preMultWith(trf);

  const Body *body2 = grip.getBody2();
  if (body2) pos.preMultWith(body2->getPos());

  pos.preMultWith(grip.getPos2());
  pos.invertInto(invPos);
}

//-------------------------------------------------------------------------------

void AbstractJoint::copyStateFrom(const AbstractJoint *jnt)
{
  if (!jnt) return;

  pos     = jnt->pos;
  invPos  = jnt->invPos;
  der1    = jnt->der1;
  invDer1 = jnt->invDer1;
  der2    = jnt->der2;
  invDer2 = jnt->invDer2;
  der3    = jnt->der3;
  invDer3 = jnt->invDer3;

  for (int i=0; i<varCnt; i++) {
    varPos[i]    = jnt->varPos[i];
    varSpeed[i]  = jnt->varSpeed[i];
    varAccel[i]  = jnt->varAccel[i];
    varJerk[i]   = jnt->varJerk[i];
    isAngular[i] = jnt->isAngular[i];
  }
}
} // namespace

// Interface Section

void GetPosAbstractJoint(void* cppJoint, Ino::Trf3& pos)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  pos = jnt->getPos();
}

void GetInvPosAbstractJoint(void* cppJoint, Ino::Trf3& invPos)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  invPos = jnt->getInvPos();
}

void GetDerAbstractJoint(void* cppJoint, Ino::Trf3& der)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  der = jnt->getDer();
}

void GetInvDerAbstractJoint(void* cppJoint, Ino::Trf3& invDer)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  invDer = jnt->getInvDer();
}

void GetAccAbstractJoint(void* cppJoint, Ino::Trf3& acc)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  acc = jnt->getAcc();
}

void GetInvAccAbstractJoint(void* cppJoint, Ino::Trf3& invAcc)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  invAcc = jnt->getInvAcc();
}

void GetJerkAbstractJoint(void* cppJoint, Ino::Trf3& jerk)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jerk = jnt->getJerk();
}

void GetInvJerkAbstractJoint(void* cppJoint, Ino::Trf3& invJerk)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  invJerk = jnt->getInvJerk();
}

int GetVarCntAbstractJoint(void* cppJoint)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getVarCnt();
}

int GetVarCntAbstractJoint2(void* cppJoint, bool fixd)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getVarCnt(fixd);
}

bool GetFixedAbstractJoint(void* cppJoint, int locIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getFixed(locIdx);
}

void SetFixedAbstractJoint(void* cppJoint, int locIdx, bool isFixed)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->setFixed(locIdx, isFixed);
}

void SetFixedAllAbstractJoint(void* cppJoint, bool isFixed)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->setFixedAll(isFixed);
}

bool GetIsAngularAbstractJoint(void* cppJoint, int locIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getIsAngular(locIdx);
}

void InitVarsFromPosAbstractJoint(void* cppJoint, bool fixedAlso)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->initVarsFromPos(fixedAlso);
}

void ClearVarIndicesAbstractJoint(void* cppJoint)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->clearVarIndices();
}

int GetVarIdxAbstractJoint(void* cppJoint, int locIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getVarIdx(locIdx);
}

void SetVarIdxAbstractJoint(void* cppJoint, int locIdx, int varIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->setVarIdx(locIdx, varIdx);
}

int GetLocIdxAbstractJoint(void* cppJoint, int vIdx)
{ 
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getLocIdx(vIdx);
}

double GetValAbstractJoint(void* cppJoint, int locIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getVal(locIdx);
}

void SetValAbstractJoint(void* cppJoint, int locIdx, double newVal)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->setVal(locIdx, newVal);
}

double GetSpeedAbstractJoint(void* cppJoint, int locIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getSpeed(locIdx);
}

void SetSpeedAbstractJoint(void* cppJoint, int locIdx, double newSpeed)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->setSpeed(locIdx, newSpeed);
}

void ClearTrfCachesAbstractJoint(void* cppJoint)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->clearTrfCaches();
}

void GetDerivativeAbstractJoint(void* cppJoint, int locIdx, Ino::Trf3& derTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  derTrf = jnt->getDerivative(locIdx);
}

void GetInvDerivativeAbstractJoint(void* cppJoint, int locIdx, Ino::Trf3& invDerTrf) 
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  invDerTrf = jnt->getInvDerivative(locIdx);
}

double GetAccelAbstractJoint(void* cppJoint, int locIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getAccel(locIdx);
}

void SetAccelAbstractJoint(void* cppJoint, int locIdx, double newAccel)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->setAccel(locIdx, newAccel);
}

void GetSecDerivativeAbstractJoint(void* cppJoint, int locIdx, Ino::Trf3 accTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getSecDerivative(locIdx, accTrf);
}

void GetInvSecDerivativeAbstractJoint(void* cppJoint, int locIdx, Ino::Trf3 accTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getInvSecDerivative(locIdx, accTrf);
}

void GetAccMixedAbstractJoint(void* cppJoint, Ino::Trf3& mixTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getAccMixed(mixTrf);
}

void GetInvAccMixedAbstractJoint(void* cppJoint, Ino::Trf3& invMixTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getInvAccMixed(invMixTrf);
}

void GetJerkMixedAbstractJoint(void* cppJoint, Ino::Trf3& mixTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getJerkMixed(mixTrf);
}

void GetInvJerkMixedAbstractJoint(void* cppJoint, Ino::Trf3& invMixTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getInvJerkMixed(invMixTrf);
}

double GetJerkAbstractJoint2(void* cppJoint, int locIdx)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  return jnt->getJerk(locIdx);
}

void SetJerkAbstractJoint(void* cppJoint, int locIdx, double newJerk) {
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->setJerk(locIdx, newJerk);
}

void GetThirdDerivativeAbstractJoint(void* cppJoint, int locIdx, Ino::Trf3& jerkTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getThirdDerivative(locIdx, jerkTrf);
}

void GetInvThirdDerivativeAbstractJoint(void* cppJoint, int locIdx, Ino::Trf3& jerkTrf)
{
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJoint;

  jnt->getInvThirdDerivative(locIdx, jerkTrf);
}

// End Interface Section

//-------------------------------------------------------------------------------
