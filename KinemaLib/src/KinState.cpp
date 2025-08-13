//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Kinema: Kinematic Simulation Program -------------------------
//---------------------------------------------------------------------------
//---------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 ---------
//---------------------------------------------------- C.Wolters ------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Storage for sequence of model position/speeds/accels ---------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinState.h"
#include "KinGrip.h"
#include "KinModel.h"
#include "KinTopology.h"
#include "KinAbstractJoint.h"
#include "KinSequence.h"

#include "Vec.h"

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

State::State(Sequence& sequence, int index,
                                 __int64 mdlTm, const Topology& srcTopo)
: ArrayElem(),
  seq(sequence), seqIdx(index), seqTm(mdlTm),
  varPos(0), fixedPos(0), varSpeed(0), fixedSpeed(0),
  varAccel(0), fixedAccel(0), varJerk(0), fixedJerk(0)
{
  srcTopo.getPosVector(varPos,false);
  srcTopo.getPosVector(fixedPos,true);
  srcTopo.getSpeedVector(varSpeed,false);
  srcTopo.getSpeedVector(fixedSpeed,true);
  srcTopo.getAccelVector(varAccel,false);
  srcTopo.getAccelVector(fixedAccel,true);
  srcTopo.getJerkVector(varJerk,false);
  srcTopo.getJerkVector(fixedJerk,true);
}

//---------------------------------------------------------------------------

State::State(Sequence& sequence, const State& cp)
: seq(sequence), seqIdx(cp.seqIdx), seqTm(cp.seqTm),
  varPos(cp.varPos), fixedPos(cp.fixedPos),
  varSpeed(cp.varSpeed), fixedSpeed(cp.fixedSpeed),
  varAccel(cp.varAccel), fixedAccel(cp.fixedAccel),
  varJerk(cp.varJerk), fixedJerk(cp.fixedJerk)
{
}

//---------------------------------------------------------------------------

static bool getValue(const Vector& values,
                     const AbstractJoint& jnt, int varIdx, double& varVal)
{
  if (varIdx < 0 || varIdx >= jnt.getVarCnt()) return false;

  int idx = jnt.getVarIdx(varIdx);
  if (idx < 0 || idx >= values.size()) return false;

  varVal = values[idx];

  return true;
}

//---------------------------------------------------------------------------

bool State::getPos(const AbstractJoint& jnt, int varIdx, double& posVal) const
{
  return getValue(varPos,jnt,varIdx,posVal);
}

//---------------------------------------------------------------------------

bool State::getSpeed(const AbstractJoint& jnt, int varIdx, double& speedVal) const
{
  return getValue(varSpeed,jnt,varIdx,speedVal);
}

//---------------------------------------------------------------------------

bool State::getAccel(const AbstractJoint& jnt, int varIdx, double& accVal) const
{
  return getValue(varAccel,jnt,varIdx,accVal);
}

//---------------------------------------------------------------------------

bool State::getJerk(const AbstractJoint& jnt, int varIdx, double& jerkVal) const
{
  return getValue(varJerk,jnt,varIdx,jerkVal);
}

//---------------------------------------------------------------------------

bool State::setTopologyToThis() const
{
  Topology& topo = seq.getTopology();

  topo.setPosVectors(varPos,fixedPos);
  bool ok = topo.updatePositions();

  topo.setSpeedVectors(varSpeed, fixedSpeed);
  topo.updateSpeeds();

  topo.setAccelVectors(varAccel, fixedAccel);
  bool updateAccels();

  topo.setJerkVectors(varJerk, fixedJerk);
  bool updateJerks();

  Model *mdl = topo.getModel();
  if (mdl) mdl->applyOffset();

  return ok;
}

} // namespace

// Interface Section

void* StateNew(InoKin::Sequence& sequence, int index, long tm, const InoKin::Topology& srcTopo)
{
  void* newState = new InoKin::State(sequence, index, tm, srcTopo);

  return newState;
}

void *GetSequenceState(void* cppState)
{
  InoKin::State* state = (InoKin::State*)cppState;

  return &state->getSequence();
}

int GetIdxState(void* cppState)
{
  InoKin::State* state = (InoKin::State*)cppState;

  return state->getIdx();
}

__int64 GetTmState(void* cppState)
{
  InoKin::State* state = (InoKin::State*)cppState;

  return state->getTm();
}

bool GetPosState(void *cppState, void *cppJnt, int varIdx, double& posVal)
{
  InoKin::State* state = (InoKin::State*)cppState;
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint *)cppJnt;

  return state->getPos(*jnt, varIdx, posVal);
}

bool GetSpeedState(void* cppState, void *cppJnt, int varIdx, double& speedVal)
{
  InoKin::State* state = (InoKin::State*)cppState;
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJnt;

  return state->getSpeed(*jnt, varIdx, speedVal);
}

bool GetAccelState(void* cppState, void *cppJnt, int varIdx, double& accVal)
{
  InoKin::State* state = (InoKin::State*)cppState;
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJnt;

  return state->getAccel(*jnt, varIdx, accVal);
}

bool GetJerkState(void* cppState, void *cppJnt, int varIdx, double& jerkVal)
{
  InoKin::State* state = (InoKin::State*)cppState;
  InoKin::AbstractJoint* jnt = (InoKin::AbstractJoint*)cppJnt;

  return state->getJerk(*jnt, varIdx, jerkVal);
}

bool SetTopologyToThisState(void* cppState)
{
  InoKin::State* state = (InoKin::State*)cppState;

  return state->setTopologyToThis();
}

// End Interface Section
//---------------------------------------------------------------------------
