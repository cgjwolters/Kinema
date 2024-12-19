//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Kinema: Kinematic Simulation Program -------------------------
//---------------------------------------------------------------------------
//---------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 ---------
//---------------------------------------------------- C.Wolters ------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Storage for a model state ------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_STATE_INC
#define INOKIN_STATE_INC

#include "Array.h"

#include "Matrix.h"

//---------------------------------------------------------------------------

namespace Ino
{
  class Vec3;
}

namespace InoKin {

class AbstractJoint;
class Topology;
class Sequence;

class State : public Ino::ArrayElem
{
  Sequence& seq;
  const int seqIdx;
  __int64 seqTm;

  Ino::Vector varPos, fixedPos;
  Ino::Vector varSpeed, fixedSpeed;
  Ino::Vector varAccel, fixedAccel;
  Ino::Vector varJerk, fixedJerk;

  State& operator=(const State& src) = delete; // No Assigment
  State(const State& cp) = delete;             // No copying

  explicit State(Sequence& sequence, const State& cp);

public:
  explicit State(Sequence& sequence, int index, __int64 mdlTm,
                 const Topology& srcTopo);

  Sequence& getSequence() const { return seq; }
  int getIdx() const { return seqIdx; }

  __int64 getTm() const { return seqTm; }

  bool getPos(const AbstractJoint& jnt, int varIdx, double& posVal) const;
  bool getSpeed(const AbstractJoint& jnt, int varIdx, double& speedVal) const;
  bool getAccel(const AbstractJoint& jnt, int varIdx, double& accVal) const;
  bool getJerk(const AbstractJoint& jnt, int varIdx, double& jerkVal) const;

  // For all other info call this method and interrogate the topology:
  bool setTopologyToThis() const;

  friend class Sequence;
};

} // namespace

extern "C" __declspec(dllexport) void* StateNew(InoKin::Sequence* seq, int index, long tm, InoKin::Topology* cppTopo);

extern "C" __declspec(dllexport) void* GetSequenceState(void* cppState);

extern "C" __declspec(dllexport) int GetIdxState(void *cppState);

extern "C" __declspec(dllexport) __int64 GetTmState(void *cppState);

extern bool GetPosState(AbstractJoint jnt, int varIdx, out double posVal);

//bool getPos(const AbstractJoint& jnt, int varIdx, double& posVal) const;
//bool getSpeed(const AbstractJoint& jnt, int varIdx, double& speedVal) const;
//bool getAccel(const AbstractJoint& jnt, int varIdx, double& accVal) const;
//bool getJerk(const AbstractJoint& jnt, int varIdx, double& jerkVal) const;
//
//// For all other info call this method and interrogate the topology:
//bool setTopologyToThis() const;


//---------------------------------------------------------------------------
#endif
