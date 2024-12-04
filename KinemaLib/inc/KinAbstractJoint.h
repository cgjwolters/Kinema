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

// Interface Section

extern "C" __declspec(dllexport) void GetPosAbstractJoint(void * cppJoint, Ino::Trf3& pos);
extern "C" __declspec(dllexport) void GetInvPosAbstractJoint(void * cppJoint, Ino::Trf3& invPos);
extern "C" __declspec(dllexport) void GetDerAbstractJoint(void * cppJoint, Ino::Trf3& der);
extern "C" __declspec(dllexport) void GetInvDerAbstractJoint(void * cppJoint, Ino::Trf3& invDer);
extern "C" __declspec(dllexport) void GetAccAbstractJoint(void * cppJoint, Ino::Trf3& acc);
extern "C" __declspec(dllexport) void GetInvAccAbstractJoint(void * cppJoint, Ino::Trf3& invAcc);
extern "C" __declspec(dllexport) void GetJerkAbstractJoint(void * cppJoint, Ino::Trf3& jerk);
extern "C" __declspec(dllexport) void GetInvJerkAbstractJoint(void * cppJoint, Ino::Trf3& invJerk);
extern "C" __declspec(dllexport) int GetVarCntAbstractJoint(void * cppJoint);
extern "C" __declspec(dllexport) int GetVarCntAbstractJoint2(void * cppJoint, bool fixd);
extern "C" __declspec(dllexport) bool GetFixedAbstractJoint(void * cppJoint, int locIdx);
extern "C" __declspec(dllexport) void SetFixedAbstractJoint(void * cppJoint, int locIdx, bool isFixed);
extern "C" __declspec(dllexport) void SetFixedAllAbstractJoint(void * cppJoint, bool isFixed);
extern "C" __declspec(dllexport) bool GetIsAngularAbstractJoint(void * cppJoint, int locIdx);
extern "C" __declspec(dllexport) void InitVarsFromPosAbstractJoint(void * cppJoint, bool fixedAlso);
extern "C" __declspec(dllexport) void ClearVarIndicesAbstractJoint(void * cppJoint);
extern "C" __declspec(dllexport) int GetVarIdxAbstractJoint(void * cppJoint, int locIdx);
extern "C" __declspec(dllexport) void SetVarIdxAbstractJoint(void * cppJoint, int locIdx, int varIdx);
extern "C" __declspec(dllexport) int GetLocIdxAbstractJoint(void * cppJoint, int vIdx);
extern "C" __declspec(dllexport) double GetValAbstractJoint(void * cppJoint, int locIdx);
extern "C" __declspec(dllexport) void SetValAbstractJoint(void * cppJoint, int locIdx, double newVal);
extern "C" __declspec(dllexport) double GetSpeedAbstractJoint(void * cppJoint, int locIdx);
extern "C" __declspec(dllexport) void SetSpeedAbstractJoint(void * cppJoint, int locIdx, double newSpeed);
extern "C" __declspec(dllexport) void ClearTrfCachesAbstractJoint(void * cppJoint);
extern "C" __declspec(dllexport) void GetDerivativeAbstractJoint(void * cppJoint, int locIdx, Ino::Trf3& derTrf);
extern "C" __declspec(dllexport) void GetInvDerivativeAbstractJoint(void * cppJoint, int locIdx, Ino::Trf3& invDerTrf);
extern "C" __declspec(dllexport) double GetAccelAbstractJoint(void * cppJoint, int locIdx);
extern "C" __declspec(dllexport) void SetAccelAbstractJoint(void * cppJoint, int locIdx, double newAccel);
extern "C" __declspec(dllexport) void GetSecDerivativeAbstractJoint(void * cppJoint, int locIdx, Ino::Trf3 accTrf);
extern "C" __declspec(dllexport) void GetInvSecDerivativeAbstractJoint(void * cppJoint, int locIdx, Ino::Trf3 accTrf);
extern "C" __declspec(dllexport) void GetAccMixedAbstractJoint(void * cppJoint, Ino::Trf3& mixTrf);
extern "C" __declspec(dllexport) void GetInvAccMixedAbstractJoint(void * cppJoint, Ino::Trf3& invMixTrf);
extern "C" __declspec(dllexport) void GetJerkMixedAbstractJoint(void * cppJoint, Ino::Trf3& mixTrf);
extern "C" __declspec(dllexport) void GetInvJerkMixedAbstractJoint(void * cppJoint, Ino::Trf3& invMixTrf);
extern "C" __declspec(dllexport) double GetJerkAbstractJoint2(void * cppJoint, int locIdx);
extern "C" __declspec(dllexport) void SetJerkAbstractJoint(void * cppJoint, int locIdx, double newJerk);
extern "C" __declspec(dllexport) void GetThirdDerivativeAbstractJoint(void * cppJoint, int locIdx, Ino::Trf3& jerkTrf);
extern "C" __declspec(dllexport) void GetInvThirdDerivativeAbstractJoint(void * cppJoint, int locIdx, Ino::Trf3& jerkTrf);


//-------------------------------------------------------------------------------
#endif
