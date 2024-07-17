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

#ifndef INOKIN_JOINT_INC
#define INOKIN_JOINT_INC

#include "KinObject.h"

#include "Trf.h"

namespace Ino
{
  class Vector;
}

namespace InoKin {

//-------------------------------------------------------------------------------

class Grip;

class AbstractJoint : public Object
{
  Ino::Trf3 pos, invPos;
  Ino::Trf3 der1,invDer1;
  Ino::Trf3 der2,invDer2;
  Ino::Trf3 der3,invDer3;

  bool *const fixedPos;
  int  *const varIdx;

  Ino::Trf3 *const derLst;
  mutable bool derLstValid;

  Ino::Trf3 *const invDerLst;
  bool *const invDerLstValid;

  void getPosRange(int lwbIdx, int upbIdx, Ino::Trf3& posTrf) const;
  void getInvPosRange(int lwbIdx, int upbIdx, Ino::Trf3& posTrf) const;
  void getInvOneDerRange(int lwbIdx, int upbIdx, Ino::Trf3& derTrf) const;
  void getOneDerRange(int lwbIdx, int upbIdx, Ino::Trf3& derTrf) const;
  void getTwoDerRange(int lwbIdx, int upbIdx, Ino::Trf3& derTrf) const;
  void getInvTwoDerRange(int lwbIdx, int upbIdx, Ino::Trf3& derTrf) const;

  void getSpeedTrf(int idx, Ino::Trf3& spTrf) const;
  void getAccelTrf(int idx, Ino::Trf3& accTrf) const;
  void getJerkTrf(int idx, Ino::Trf3& jerkTrf) const;

  void getDerivativeAll() const;

  void setPos();
  void setSpeed();
  void setAccel();
  void setJerk();

  AbstractJoint(const AbstractJoint& cp) = delete;             // No copying
  AbstractJoint& operator=(const AbstractJoint& src) = delete; // No assignment

protected:
  double *const varPos;
  double *const varSpeed;
  double *const varAccel;
  double *const varJerk;
  bool   *const isAngular;

  virtual AbstractJoint *clone(Grip& newGrip) const = 0;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const = 0;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const = 0;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const = 0;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const = 0;

  void calcPosFromNeighbours();

  void copyStateFrom(const AbstractJoint *jnt);

public:
  const int varCnt;
  Grip& grip;

  explicit AbstractJoint(Grip& grp, const wchar_t *name, int nrVars);
  explicit AbstractJoint(Grip& newgrp, const AbstractJoint& cp);

  virtual ~AbstractJoint();

  const Ino::Trf3& getPos() const { return pos; }
  const Ino::Trf3& getInvPos() const { return invPos; }

  const Ino::Trf3& getDer() const { return der1; }
  const Ino::Trf3& getInvDer() const { return invDer1; }

  const Ino::Trf3& getAcc() const { return der2; }
  const Ino::Trf3& getInvAcc() const { return invDer2; }

  const Ino::Trf3& getJerk() const { return der3; }
  const Ino::Trf3& getInvJerk() const { return invDer3; }

  int getVarCnt() const { return varCnt; }
  int getVarCnt(bool fixed) const;

  bool getFixed(int locIdx) const;
  virtual void setFixed(int locIdx, bool isFixed);

  virtual void setFixedAll(bool isFixed);

  bool getIsAngular(int locIdx) const;

  virtual void initVarsFromPos(bool fixedAlso) = 0;

  void clearVarIndices();

  int  getVarIdx(int locIdx) const;
  void setVarIdx(int locIdx, int varIdx);

  int  getLocIdx(int vIdx) const;

  double getVal(int locIdx) const;
  void   setVal(int locIdx, double newVal);

  void getVars(bool isFixed, Ino::Vector& varVec) const;
  void setVars(bool isFixed, const Ino::Vector& varVec);
  void setVars(const Ino::Vector& varVec, const Ino::Vector& fixedVec);

  double getSpeed(int locIdx) const;
  void   setSpeed(int locIdx, double newSpeed);

  void getSpeeds(bool isFixed, Ino::Vector& speedVec) const;
  void setSpeeds(bool isFixed, const Ino::Vector& speedVec);
  void setSpeeds(const Ino::Vector& varSpeedVec,
                 const Ino::Vector& fixedSpeedVec);

  void clearTrfCaches();

  const Ino::Trf3& getDerivative(int locIdx) const;
  const Ino::Trf3& getInvDerivative(int locIdx) const;

  double getAccel(int locIdx) const;
  void   setAccel(int locIdx, double newAccel);

  void getAccels(bool isFixed, Ino::Vector& accelVec) const;
  void setAccels(bool isFixed, const Ino::Vector& accelVec);
  void setAccels(const Ino::Vector& varAccelVec,
                 const Ino::Vector& fixedAccelVec);

  void getSecDerivative(int locIdx, Ino::Trf3& accTrf) const;
  void getInvSecDerivative(int locIdx, Ino::Trf3& accTrf) const;

  void getAccMixed(Ino::Trf3& mixTrf) const;
  void getInvAccMixed(Ino::Trf3& invMixTrf) const;

  void getJerkMixed(Ino::Trf3& mixTrf);
  void getInvJerkMixed(Ino::Trf3& invMixTrf);

  double getJerk(int locIdx) const;
  void   setJerk(int locIdx, double newJerk);

  void getJerks(bool isFixed, Ino::Vector& jerkVec) const;
  void setJerks(bool isFixed, const Ino::Vector& jerkVvec);
  void setJerks(const Ino::Vector& varJerkVec,
                const Ino::Vector& fixedJerkVec);

  void getThirdDerivative(int locIdx, Ino::Trf3& jerkTrf) const;
  void getInvThirdDerivative(int locIdx, Ino::Trf3& jerkTrf) const;

  friend class Grip;
  friend class GripList;
  friend class Topology;
  friend class Model;
};

} // namespace

//-------------------------------------------------------------------------------
#endif
