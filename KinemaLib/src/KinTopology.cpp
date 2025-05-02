//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Topology of bodies and grips -----------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinTopology.h"
#include "KinModel.h"
#include "KinBody.h"
#include "KinGrip.h"
#include "KinAbstractJoint.h"
#include "KinProbe.h"
#include "Matrix.h"
#include "Exceptions.h"

#include <deque>
#include <cmath>

#ifdef _WIN32
#include <malloc.h>
#else
#include <alloca.h>
#endif

using namespace Ino;

namespace InoKin {

//-------------------------------------------------------------------------------

Topology::Topology(Model& mdl)
: Ino::Array<GripList *>(true), model(&mdl),
  topoBodyLst(*new BodyList()),
  topoGripLst(*new GripList()),
  rowSz(0), colSz(0), varSz(0), fixedSz(0),
  angularVar(NULL),
  posValid(false), speedValid(false),
  accelValid(false), jerkValid(false),
  seqLst(true,2),
  prepMat(*new Matrix(0,0)),
  solMat(*new Matrix(0,0)),
  rhs(*new Vector(0)),
  solMat2(0,0),
  speedMat2(0,0),
  solRhs2(0),
  speedRhs2(0)
{
}

//-------------------------------------------------------------------------------

Topology::Topology(Model& mdl, const Topology& cp, bool withSequences)
: LoopList(true,cp.size()), model(&mdl),
  topoBodyLst(*new BodyList()),
  topoGripLst(*new GripList()),
  rowSz(cp.rowSz), colSz(cp.colSz), varSz(cp.varSz), fixedSz(cp.fixedSz),
  angularVar(NULL),
  posValid(cp.posValid), speedValid(cp.speedValid),
  accelValid(cp.accelValid), jerkValid(cp.jerkValid),
  seqLst(true,cp.seqLst.size()),
  prepMat(*new Matrix(0,0)),
  solMat(*new Matrix(0,0)),
  rhs(*new Vector(0)),
  solMat2(0,0),
  speedMat2(0,0),
  solRhs2(0),
  speedRhs2(0)
{
  int sz = cp.topoBodyLst.size();

  for (int i=0; i<sz; ++i) {
    topoBodyLst.add(mdl.getBodyList().byId(cp.topoBodyLst[i]->getId()));
  }

  sz = cp.topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    topoGripLst.add(mdl.getGripList().byId(cp.topoGripLst[i]->getId()));
  }

  sz = cp.size();

  for (int i=0; i<sz; ++i) {
    GripList *gLst = new GripList(*cp[i]);

    int gSz = gLst->size();
    
    for (int j=0; j<gSz; ++j) {
      const Grip *grp = gLst->get(j);
      if (grp) gLst->set(j,mdl.getGripList().byId(grp->getId()));
    }

    add(gLst);
  }

  angularVar = new bool[varSz];
  topoGripLst.setAngularVars(angularVar);

  if (withSequences) {
    for (int i=0; i<cp.seqLst.size(); ++i)
      seqLst.add(new Sequence(*this,*cp.seqLst[i]));
  }
}

//-------------------------------------------------------------------------------

Topology::~Topology()
{
  delete &topoGripLst;
  delete &topoBodyLst;

  delete[] angularVar;

  delete &prepMat;
  delete &solMat;
  delete &rhs;
}

//-------------------------------------------------------------------------------

void Topology::clear()
{
  topoBodyLst.clear();
  topoGripLst.clear();

  LoopList::clear();

  rowSz   = 0;
  colSz   = 0;
  varSz   = 0;
  fixedSz = 0;

  delete[] angularVar; angularVar = NULL;

  posValid   = false;
  speedValid = false;
  accelValid = false;
  jerkValid  = false;

  seqLst.clear();
}

//-------------------------------------------------------------------------------

void Topology::updateJointTransforms()
{
  int sz = topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = topoGripLst[i];
  
    AbstractJoint *jnt = grp->getJoint();
    if (!jnt) continue;

    if (posValid)   jnt->setPos();
    if (speedValid) jnt->setSpeed();
    if (accelValid) jnt->setAccel();
    if (jerkValid)  jnt->setJerk();
  }
}

//-------------------------------------------------------------------------------

void Topology::buildLoop(Grip *grp)
{
  topoGripLst.add(grp);

  typedef std::deque<Grip *>GripLoop;

  GripLoop gripLoop;

  gripLoop.push_back(grp);

  Body *body1 = grp->getBody1();
  if (!body1) throw NullPointerException("Topology::buildLoop (body1)");

  Body *body2 = grp->getBody2();
  if (!body2) throw NullPointerException("Topology::buildLoop (body2)");

  if (body1->getTreeLevel() > body2->getTreeLevel()) {
    grp = body1->getParentGrip();
    gripLoop.push_front(grp);

    body1 = grp->getOtherBody(*body1);
  }
  else if (body1->getTreeLevel() < body2->getTreeLevel()) {
    grp = body2->getParentGrip();
    gripLoop.push_back(grp);

    body2 = grp->getOtherBody(*body2);
  }
  else grp->loopCnt = 1;

  // Now at equal level

  for (;;) {
    Body *parentBody1 = body1->getParent();
    _ASSERT(parentBody1 != NULL);

    Body *parentBody2 = body2->getParent();
    _ASSERT(parentBody1 != NULL);

    if (parentBody1 == parentBody2) {
      gripLoop.push_front(body1->getParentGrip());
      gripLoop.push_back(body2->getParentGrip());

      break;
    }

    grp = body1->gripTo(*parentBody2);

    if (grp) {
      gripLoop.push_front(grp);
      gripLoop.push_back(body2->getParentGrip());
      break;
    }

    grp = body2->gripTo(*parentBody1);
    if (grp) {
      gripLoop.push_front(body1->getParentGrip());
      gripLoop.push_back(grp);
      break;
    }

    grp = parentBody1->gripTo(*parentBody2);
    if (grp) {
      gripLoop.push_front(body1->getParentGrip());
      gripLoop.push_front(grp);
      gripLoop.push_back(body2->getParentGrip());
      break;
    }

    gripLoop.push_front(body1->getParentGrip());
    body1 = parentBody1;

    gripLoop.push_back(body2->getParentGrip());
    body2 = parentBody2;
  }

  GripList *grpLst = new GripList();
  GripLoop::iterator it = gripLoop.begin();

  for (; it != gripLoop.end(); ++it) grpLst->add(*it);

  add(grpLst);
}

//-------------------------------------------------------------------------------

void Topology::bodyScan(int idx)
{
  Body *body = topoBodyLst[idx];

  int lvl = body->getTreeLevel();

  int sz = body->gripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = body->gripLst[i];
    if (grp->getLoopCnt() > 0)
      continue; // Same level bridge already in a loop

    Body *ob = grp->getOtherBody(*body);

    if (ob->getTreeLevel() < 0) { // Not seen before
      ob->setTreeLevel(lvl+1);
      grp->parentRel = true;

      topoGripLst.add(grp);
      topoBodyLst.add(ob);
    }
    else if (ob->getTreeLevel() < lvl) 
      continue; // Already handled
    else {
      buildLoop(grp);
    }
  }
}

//-------------------------------------------------------------------------------

void Topology::loopScan(Array<GripList *>& loopLst, int idx)
{
  GripList *curGrpLst = loopLst[idx];

  int lvl = curGrpLst->loopLvl;

  GripList::PeerSet::iterator it = curGrpLst->peerSet.begin();
  GripList::PeerSet::iterator end = curGrpLst->peerSet.end();

  for (;it!=end; ++it) {
    GripList *grpLst = *it;

    if (grpLst->loopLvl < 0) { // Not seen before
      grpLst->loopLvl = lvl+1;

      loopLst.add(grpLst);
    }
  }
}

//-------------------------------------------------------------------------------
// Reduce the matrix bandwidth
//
// The loops are set up as vertices in a graph.
// A breadth first search as in the Cuthill–McKee algorithm is
// used to order the loops.
// Then variable indices are assigned
// VERY effective!!, seriously reduces the bandwidth and
// makes the algorith very fast.
// (Have seen example improvements in compute time by a factor of 60)

void Topology::analyzeLoops()
{
  for (int i=0; i<size(); ++i) {
    GripList *grpLst = get(i);
    grpLst->peerSet.clear();

    int sz = grpLst->size();

    for (int j=0; j<sz; ++j) grpLst->get(j)->loopCnt = -1;
  }

  for (int i=0; i<size(); ++i) {
    GripList *grpLst = get(i);

    int sz = grpLst->size();

    for (int j=0; j<sz; ++j) {
      Grip *grp = grpLst->get(j);
      
      if (grp->loopCnt < 0) grp->loopCnt = i; // Misusing loopCnt
    }
  }

  for (int i=0; i<size(); ++i) {
    GripList *grpLst = get(i);
    grpLst->loopLvl = -1;

    int sz = grpLst->size();

    for (int j=0; j<sz; ++j) {
      int idx = grpLst->get(j)->loopCnt;

      if (idx != i) {
        grpLst->peerSet.insert(get(idx)); // Auto sorted to increasing degree
        get(idx)->peerSet.insert(grpLst);
      }
    }
  }

  GripList *firstLst = NULL;
  size_t peerSz = 0;

  for (int i=0; i<size(); ++i) {
    GripList *grpLst = get(i);

    size_t pSz = grpLst->peerSet.size();

    if (!firstLst || pSz < peerSz) {
      firstLst = grpLst;
      peerSz = pSz;
    }
  }


  LoopList loopLst(size());

  if (firstLst) {
    loopLst.add(firstLst);
    firstLst->loopLvl = 0;
  }

  for (int i=0; i<loopLst.size(); ++i) loopScan(loopLst, i); // Size increasing!!

  assignVarIndices(loopLst);
}

//-------------------------------------------------------------------------------

void Topology::assignVarIndices(LoopList& loopLst)
{
  int sz = topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();
    if (jnt) jnt->clearVarIndices();
  }

  rowSz = 0;
  colSz = 0; // See below
  varSz = 0;
  fixedSz = 0;

  int loopSz = loopLst.size();

  for (int i=0; i<loopSz; ++i) {
    GripList *grpLst = loopLst[i];
    int gSz = grpLst->size();

    for (int j=0; j<gSz; ++j) {
      AbstractJoint *jnt = grpLst->get(j)->getJoint();
      if (!jnt) continue;
      
      int varCnt = jnt->getVarCnt();

      for (int k=0; k<varCnt; ++k) {
        if (jnt->getVarIdx(k) < 0) {
          if (jnt->getFixed(k))
               jnt->setVarIdx(k,fixedSz++);
          else jnt->setVarIdx(k,varSz++);
        }
      }
    }
  }

  // Determine required matrix bandwidth: (set in colSz)

  enum { MaxAllocaSz = 16000 }; // Max nr of items on stack

  int *lwbIdx, *upbIdx;

  if (varSz > MaxAllocaSz) {
    lwbIdx = new int[varSz];
    upbIdx = new int[varSz];
  }
  else {
    lwbIdx = (int *)_malloca(varSz*sizeof(int));
    upbIdx = (int *)_malloca(varSz*sizeof(int));
  }

  memset(lwbIdx,-1,varSz*sizeof(int));
  memset(upbIdx, 0,varSz*sizeof(int));

  for (int i=0; i<loopSz; ++i) {
    GripList *grpLst = loopLst[i];
    int gSz = grpLst->size();

    for (int j=0; j<gSz; ++j) {
      AbstractJoint *jnt = grpLst->get(j)->getJoint();
      if (!jnt) continue;
      
      int varCnt = jnt->getVarCnt();

      for (int k=0; k<varCnt; ++k) {
        if (jnt->getFixed(k)) continue;

        int idx = jnt->getVarIdx(k);

        if (lwbIdx[idx] < 0) lwbIdx[idx] = rowSz;
        upbIdx[idx] = rowSz+6;
      }
    }

    rowSz += 6;
  }

  // Determine bandwidth

  colSz = 0;

  for (int i=0; i<varSz; ++i) {
    for (int j=0; j<i-colSz; ++j) {
      if (std::max(lwbIdx[i],lwbIdx[j]) < std::min(upbIdx[i],upbIdx[j])) {
        int bw = i-j;
        if (bw > colSz) {
          colSz = bw;
          break;
        }
      }
    }
  }

  if (colSz) colSz++;

  if (varSz > MaxAllocaSz) {
    delete[] lwbIdx;
    delete[] upbIdx;
  }

  delete[] angularVar; angularVar = new bool[varSz];

  topoGripLst.setAngularVars(angularVar);
}

//-------------------------------------------------------------------------------
// Sets in each grip the number of loops it is part of

void Topology::setLoopCounts()
{
  int lSz = size();

  for (int i=0; i<lSz; ++i) {
    GripList *grpLst = get(i);

    int gSz = grpLst->size();
    for (int j=0; j<gSz; ++j) grpLst->get(j)->loopCnt = 0;
  }

  for (int i=0; i<lSz; ++i) {
    GripList *grpLst = get(i);

    int gSz = grpLst->size();
    for (int j=0; j<gSz; ++j) grpLst->get(j)->loopCnt++;
  }
}

//-------------------------------------------------------------------------------

void Topology::prepare(Body *fstBody)
{
  clear();

  topoBodyLst.ensureCapacity(model->bodyLst.size());
  topoGripLst.ensureCapacity(model->gripLst.size());

  fstBody->setTreeLevel(0);
  topoBodyLst.add(fstBody);

  rowSz = 0;
  colSz = 0;
  varSz = 0;
  fixedSz = 0;

  for (int i=0; i<topoBodyLst.size(); ++i) bodyScan(i); // Size increasing!!

  analyzeLoops();
  setLoopCounts();

  int sz = topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = topoGripLst[i];

    AbstractJoint *jnt = grp->getJoint();
    if (!jnt) throw NullPointerException("Topology::prepare: null joint");

    jnt->initVarsFromPos(true);
  }
}

//---------------------------------------------------------------------------

void Topology::sizeMats()
{
  prepMat.resize(6,varSz);
  solMat.resize(varSz,colSz);
  rhs.setSize(varSz);
}

//---------------------------------------------------------------------------

static void updateDists(const Trf3& trf, double& maxRot, double& maxDist)
{
  maxRot = std::max(maxRot,fabs(trf(0,2)));
  maxRot = std::max(maxRot,fabs(trf(1,0)));
  maxRot = std::max(maxRot,fabs(trf(2,1)));

  double offset = sqrt(sqr(trf(0,3)) + sqr(trf(1,3)) + sqr(trf(2,3)));

  maxDist = std::max(maxDist,offset);
}

//-------------------------------------------------------------------------------

bool Topology::composePosMatrixRow(const GripList& grpLst,
                                   double& maxRot, double& maxDist)
{
  Trf3 loopTrf;
  if (!grpLst.setPreTrfs(loopTrf)) return false;

  updateDists(loopTrf,maxRot,maxDist);

  Body *body = grpLst.firstBody();
  if (!body) return false;

  enum { MaxAllocaSz = 32000 }; // Max nr of items on stack

  int idxLstSz = 0;
  int *idxLst;
 
  if (varSz > MaxAllocaSz) idxLst = new int[varSz];
  else idxLst = (int *)_malloca(varSz*sizeof(int));

  for (int i=0; i<varSz; ++i) idxLst[i] = -1;

  Trf3 curTrf;

  int grpSz = grpLst.size();

  for (int i=grpSz-1; i>=0; --i) {
    Grip& grp = *grpLst[i];

    bool atBody1 = body == grp.getBody1();

    if (atBody1) curTrf.preMultWith(grp.getPos1());
    else curTrf.preMultWith(grp.getPos2());

    const Trf3& preTrf = grpLst.getPreTrf(i);
    AbstractJoint& jnt = *grp.getJoint();

    int varCnt = jnt.getVarCnt();

    for (int j=0; j<varCnt; ++j) {
      if (jnt.getFixed(j)) continue;

      int varIdx = jnt.getVarIdx(j);

      Trf3 trf(curTrf);

      if (atBody1) trf.preMultWith(jnt.getDerivative(j));
      else trf.preMultWith(jnt.getInvDerivative(j));

      trf.preMultWith(preTrf);

      // Six matrix rows
      prepMat(0,varIdx) = trf(0,2);
      prepMat(1,varIdx) = trf(1,0);
      prepMat(2,varIdx) = trf(2,1);
      prepMat(3,varIdx) = trf(0,3);
      prepMat(4,varIdx) = trf(1,3);
      prepMat(5,varIdx) = trf(2,3);

      rhs[varIdx] -= trf(0,2) * loopTrf(0,2);
      rhs[varIdx] -= trf(1,0) * loopTrf(1,0);
      rhs[varIdx] -= trf(2,1) * loopTrf(2,1);
      rhs[varIdx] -= trf(0,3) * loopTrf(0,3);
      rhs[varIdx] -= trf(1,3) * loopTrf(1,3);
      rhs[varIdx] -= trf(2,3) * loopTrf(2,3);

      idxLst[idxLstSz++] = varIdx;
    }

    if (atBody1) {
      curTrf.preMultWith(jnt.pos);
      curTrf.preMultWith(grp.getInvPos2());
    }
    else {
      curTrf.preMultWith(jnt.invPos);
      curTrf.preMultWith(grp.getInvPos1());
    }

    body = grp.getOtherBody(*body);
  }

  for (int i=0; i<6; ++i) {
    for (int j=0; j<idxLstSz; ++j) {
      int colIdx = idxLst[j];

      double m = prepMat(i,colIdx);

      for (int k=0; k<idxLstSz; ++k) {
        int rowIdx = idxLst[k];
        if (rowIdx < colIdx) continue;

        // Store element of AT-A, optimized storage, see Matrix::solveLDLT()
        solMat(colIdx,rowIdx-colIdx) += m * prepMat(i,rowIdx);
      }
    }
  }

  if (varSz > MaxAllocaSz) delete[] idxLst;

  return true;
}

//---------------------------------------------------------------------------

bool Topology::composePosEq(double& maxRot, double& maxDist, int& maxIdx)
{
  maxRot  = 0.0;
  maxDist = 0.0;
  maxIdx = -1;

  solMat.clear();
  rhs.clear();

  topoGripLst.clearJointTrfCaches();

  // model.func_lst.updateAllPos();

  bool ok = true;
  int sz = size();

  for (int i=0; i<sz; i++) {
    GripList& grpLst = *get(i);

    double lastMaxDist = maxDist;
    if (!composePosMatrixRow(grpLst,maxRot,maxDist)) ok = false;

    if (maxDist > lastMaxDist) maxIdx = i;
  }

  return ok;
}

//---------------------------------------------------------------------------

void Topology::limitSolution(Vector& sol)
{
  double maxAng = atan(1.0)/4.5;  // 10 degrees

  double maxRot = 0.0;

  for (int i=0; i<varSz; ++i) {
    if (angularVar[i]) {
      double val = fabs(sol[i]);

      if (val > maxRot)
        maxRot = val;
    }
  }

  if (maxRot > maxAng) sol *= (maxAng/maxRot); 
}

//---------------------------------------------------------------------------

Sequence& Topology::newSequence(const wchar_t *name)
{
  Sequence *seq = new Sequence(*this,name);
  seqLst.add(seq);

  return *seq;
}

//---------------------------------------------------------------------------

bool Topology::solvePos(int maxIter, double rotTol, double posTol,
                                                Vector& varPosVec, int& iter)
{
  if (rowSz < 1 || colSz < 1 || varSz < 1) {
    posValid   = false;
    speedValid = false;
    accelValid = false;
    jerkValid  = false;

    return false;
  }

  posValid = true;
  varPosVec.setSize(varSz);

  sizeMats();

  double maxRot=0.0, maxDist=0.0;
  int maxIdx = -1;

  for (iter=0; iter<maxIter; ++iter) {

    if (!composePosEq(maxRot,maxDist,maxIdx))
      return false;

    if (iter > 0) {
      if (maxRot <= rotTol && maxDist <= posTol) {
        solMat2 = solMat;
        solRhs2 = rhs;
        return true;
      }
    }

    solMat.solveLDLT(rhs);

    limitSolution(rhs);

    // double len = rhs.len();

    getPosVector(varPosVec);
    varPosVec += rhs;
    setPosVector(varPosVec);
    updatePositions();
  }

  posValid = false;
  varPosVec.setSize(0);

  return false;
}

//-------------------------------------------------------------------------------

bool Topology::getPosVector(Vector& posVec, bool fixed) const
{
  if (!posValid) {
    posVec.setSize(0);
    return false;
  }

  if (fixed) posVec.setSize(fixedSz);
  else posVec.setSize(varSz);

  posVec.clear();

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->getVars(fixed, posVec);
  }

  return true;
}

//-------------------------------------------------------------------------------

void Topology::setPosVector(const Vector& posVec, bool fixed)
{
  speedValid = false;
  accelValid = false;
  jerkValid  = false;

  if (posVec.size() != (fixed ? fixedSz : varSz)) {
    posValid = false;
    return;
  }

  posValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->setVars(fixed,posVec);
  }
}

//---------------------------------------------------------------------------

void Topology::setPosVectors(const Vector& varPosVec,
                                                  const Vector& fixedPosVec)
{
  speedValid = false;
  accelValid = false;
  jerkValid  = false;

  if (varPosVec.size() != varSz || fixedPosVec.size() != fixedSz) {
    posValid = false;
    return;
  }

  posValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();
    if (!jnt) continue;
    
    jnt->setVars(varPosVec,fixedPosVec);
  }
}

//---------------------------------------------------------------------------

static void updatePos(Grip& grip)
{
  Body *body1 = grip.getBody1();
  Body *body2 = grip.getBody2();

  if (body1->getTreeLevel() < body2->getTreeLevel()) {
    Trf3 trf(body1->getPos());

    trf.preMultWith(grip.getPos1());
    trf.preMultWith(grip.getJoint()->getPos());
    trf.preMultWith(grip.getInvPos2());

    grip.getBody2()->setPos(trf);
  }
  else {
    Trf3 trf(body2->getPos());

    trf.preMultWith(grip.getPos2());
    trf.preMultWith(grip.getJoint()->getInvPos());
    trf.preMultWith(grip.getInvPos1());

    grip.getBody1()->setPos(trf);
  }
}

//---------------------------------------------------------------------------

bool Topology::updatePositions() const
{
  if (!posValid) return false;

  int sz = topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = topoGripLst[i];
    if (!grp->parentRel) continue;

    updatePos(*grp);
  }

  return true;
}

//-------------------------------------------------------------------------------

static void addSol(const Trf3& trf, double *v)
{
  v[0] += trf(0,2);
  v[1] += trf(1,0);
  v[2] += trf(2,1);
  v[3] += trf(0,3);
  v[4] += trf(1,3);
  v[5] += trf(2,3);
}

//-------------------------------------------------------------------------------

bool Topology::composeSpeedMatrixRow(const GripList& grpLst)
{
  Trf3 curTrf;
  if (!grpLst.setPreTrfs(curTrf)) return false;

  Body *body = grpLst.firstBody();
  if (!body) return false;

  enum { MaxAllocaSz = 32000 }; // Max nr of items on stack

  int idxLstSz = 0;
  int *idxLst;
 
  if (varSz > MaxAllocaSz) idxLst = new int[varSz];
  else idxLst = (int *)_malloca(varSz*sizeof(int));

  for (int i=0; i<varSz; ++i) idxLst[i] = -1;

  curTrf.init();

  int grpSz = grpLst.size();

  for (int i=grpSz-1; i>=0; --i) {
    Grip& grp = *grpLst[i];

    bool atBody1 = body == grp.getBody1();

    if (atBody1) curTrf.preMultWith(grp.getPos1());
    else curTrf.preMultWith(grp.getPos2());

    const Trf3& preTrf = grpLst.getPreTrf(i);
    AbstractJoint& jnt = *grp.getJoint();

    double spDiff[6] = { 0,0,0,0,0,0 };
    
    int varCnt = jnt.getVarCnt();

    for (int j=0; j<varCnt; j++) {
      if (!jnt.getFixed(j)) continue; // Only fixed vars

      Trf3 trf(curTrf);

      Trf3 jntTrf;

      if (atBody1) jntTrf = jnt.getDerivative(j);
      else jntTrf = jnt.getInvDerivative(j);

      jntTrf *= jnt.getSpeed(j);

      trf.preMultWith(jntTrf);
      trf.preMultWith(preTrf);

      addSol(trf, spDiff);
    }

    for (int j=0; j<varCnt; j++) {
      if (jnt.getFixed(j)) continue; // Only free vars

      Trf3 trf(curTrf);

      if (atBody1) trf.preMultWith(jnt.getDerivative(j));
      else trf.preMultWith(jnt.getInvDerivative(j));

      trf.preMultWith(preTrf);

      int varIdx = jnt.getVarIdx(j);

      // Six matrix rows
      prepMat(0,varIdx) = trf(0,2);
      prepMat(1,varIdx) = trf(1,0);
      prepMat(2,varIdx) = trf(2,1);
      prepMat(3,varIdx) = trf(0,3);
      prepMat(4,varIdx) = trf(1,3);
      prepMat(5,varIdx) = trf(2,3);

      rhs[varIdx] -= spDiff[0];
      rhs[varIdx] -= spDiff[1];
      rhs[varIdx] -= spDiff[2];
      rhs[varIdx] -= spDiff[3];
      rhs[varIdx] -= spDiff[4];
      rhs[varIdx] -= spDiff[5];

      //rhs[varIdx] -= trf(0,2) * spDiff[0];
      //rhs[varIdx] -= trf(1,0) * spDiff[1];
      //rhs[varIdx] -= trf(2,1) * spDiff[2];
      //rhs[varIdx] -= trf(0,3) * spDiff[3];
      //rhs[varIdx] -= trf(1,3) * spDiff[4];
      //rhs[varIdx] -= trf(2,3) * spDiff[5];

      idxLst[idxLstSz++] = varIdx;
    }

    if (atBody1) {
      curTrf.preMultWith(jnt.pos);
      curTrf.preMultWith(grp.getInvPos2());
    }
    else {
      curTrf.preMultWith(jnt.invPos);
      curTrf.preMultWith(grp.getInvPos1());
    }

    body = grp.getOtherBody(*body);
  }

  for (int i=0; i<6; ++i) {
    for (int j=0; j<idxLstSz; ++j) {
      int colIdx = idxLst[j];

      double m = prepMat(i,colIdx);

      for (int k=0; k<idxLstSz; ++k) {
        int rowIdx = idxLst[k];
        if (rowIdx < colIdx) continue;

        // Store element of AT-A, optimized storage, see Matrix::solveLDLT()
        solMat(colIdx,rowIdx-colIdx) += m * prepMat(i,rowIdx);
      }
    }
  }

  if (varSz > MaxAllocaSz) delete[] idxLst;

  return true;
}

//-------------------------------------------------------------------------------

bool Topology::composeSpeedEq()
{
  solMat.clear();
  rhs.clear();

  topoGripLst.clearJointTrfCaches();

  // model.func_lst.updateAllDer();

  bool ok = true;
  int sz = size();

  for (int i=0; i<sz; i++) {
    GripList& grpLst = *get(i);
    if (!composeSpeedMatrixRow(grpLst)) ok = false;
  }

  return ok;
}

//-------------------------------------------------------------------------------

bool Topology::solveSpeed(Vector& speedVec, bool write)
{
  speedValid = false;
  accelValid = false;
  jerkValid  = false;

  if (!posValid) return false;

  speedVec.setSize(varSz);

  if (!composeSpeedEq()) return false;

  speedMat2 = solMat;
  speedRhs2 = rhs;

  //FILE *fd = fopen("\\Temp\\speedrhs.csv","wt");

  //for (int i=0; i<rhs.size(); ++i) fprintf(fd,"%.f\n",rhs[i]);

  //fclose(fd);

  solMat.solveLDLT(rhs);

  speedVec = rhs;

  //fd = fopen("\\Temp\\speedsol.csv","wt");

  //for (int i=0; i<rhs.size(); ++i) fprintf(fd,"%.f\n",rhs[i]);

  //fclose(fd);

  //fd = fopen("\\Temp\\speeddiag.csv","wt");

  //for (int i=0; i<rhs.size(); ++i) fprintf(fd,"%.f\n",solMat(i,i));

  //fclose(fd);

  updateSpeeds();

  speedValid = true;

  return true;
}

//---------------------------------------------------------------------------

bool Topology::getSpeedVector(Vector& speedVec, bool fixed) const
{
  if (!speedValid) {
    speedVec.setSize(0);
    return false;
  }

  if (fixed) speedVec.setSize(fixedSz);
  else speedVec.setSize(varSz);

  speedVec.clear();

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->getSpeeds(fixed, speedVec);
  }

  return true;
}

//---------------------------------------------------------------------------

void Topology::setSpeedVector(const Vector& speedVec, bool fixed)
{
  accelValid = false;
  jerkValid  = false;

  if (speedVec.size() != (fixed ? fixedSz : varSz)) {
    speedValid = false;
    return;
  }

  speedValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->setSpeeds(fixed,speedVec);
  }
}

//---------------------------------------------------------------------------

void Topology::setSpeedVectors(const Vector& varSpeedVec,
                                              const Vector& fixedSpeedVec)
{
  accelValid = false;
  jerkValid  = false;

  if (varSpeedVec.size() != varSz || fixedSpeedVec.size() != fixedSz) {
    speedValid = false;
    return;
  }

  speedValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->setSpeeds(varSpeedVec,fixedSpeedVec);
  }
}

//---------------------------------------------------------------------------

static void updateSpeed(Grip& grip)
{
  Body *body1 = grip.getBody1();
  Body *body2 = grip.getBody2();

  if (body1->getTreeLevel() < body2->getTreeLevel()) {
    Trf3 pos(body1->getPos());
    Trf3 speed(body1->getSpeed());

    pos.preMultWith(grip.getPos1());
    speed.preMultWith(grip.getPos1());

    speed.preMultWith(grip.getJoint()->getPos());
    pos.preMultWith(grip.getJoint()->getDer());
    speed += pos;

    speed.preMultWith(grip.getInvPos2());

    body2->setSpeed(speed);
  }
  else {
    Trf3 pos(body2->getPos());
    Trf3 speed(body2->getSpeed());

    pos.preMultWith(grip.getPos2());
    speed.preMultWith(grip.getPos2());

    speed.preMultWith(grip.getJoint()->getInvPos());
    pos.preMultWith(grip.getJoint()->getInvDer());
    speed += pos;

    speed.preMultWith(grip.getInvPos1());

    body1->setSpeed(speed);
  }
}

//---------------------------------------------------------------------------

bool Topology::updateSpeeds()
{
  if (!speedValid) return false;

  int sz = topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = topoGripLst[i];
    if (!grp->parentRel) continue;

    updateSpeed(*grp);
  }

  return true;
}

//---------------------------------------------------------------------------

bool Topology::composeAccelMatrixRow(const GripList& grpLst)
{
  Trf3 curTrf;
  if (!grpLst.setPreTrfs(curTrf)) return false;

  int grpSz = grpLst.size();
  if (grpSz < 1) return false;

  Body *body = grpLst.firstBody();
  if (!body) return false;

  enum { MaxAllocaSz = 32000 }; // Max nr of items on stack

  int idxLstSz = 0;
  int *idxLst;
 
  if (varSz > MaxAllocaSz) idxLst = new int[varSz];
  else idxLst = (int *)_malloca(varSz*sizeof(int));

  for (int i=0; i<varSz; ++i) idxLst[i] = -1;

  curTrf.init();
 
  Trf3 secDer, der;

  for (int i=grpSz-1; i>=0; i--) {
    Grip& grp = *grpLst[i];
    AbstractJoint& jnt = *grp.getJoint();

    Trf3 accSum;
    accSum.zero();

    bool atBody1 = body == grp.getBody1();

    if (atBody1) {
      curTrf.preMultWith(grp.getPos1());
      jnt.getAccMixed(accSum);
    }
    else {
      curTrf.preMultWith(grp.getPos2());
      jnt.getInvAccMixed(accSum);
    }

    int varCnt = jnt.getVarCnt();

    for (int j=0; j<varCnt; ++j) {
      if (atBody1) jnt.getSecDerivative(j,secDer);
      else jnt.getInvSecDerivative(j,secDer);

      secDer *= sqr(jnt.getSpeed(j));
      accSum += secDer;

      if (!jnt.getFixed(j)) continue;

      // Only fixed vars:

      if (atBody1) der = jnt.getDerivative(j);
      else der = jnt.getInvDerivative(j);

      der *= jnt.getAccel(j);
      accSum += der;
    }

    accSum *= curTrf;

    const Trf3& preTrf = grpLst.getPreTrf(i);
    accSum.preMultWith(preTrf);

    for (int j=0; j<varCnt; j++) {
      if (jnt.getFixed(j)) continue;

      if (atBody1) jnt.getSecDerivative(j,secDer);
      else jnt.getInvSecDerivative(j,secDer);

      Trf3 trf(curTrf);
      trf.preMultWith(secDer);
      trf.preMultWith(preTrf);

      int varIdx = jnt.getVarIdx(j);

      prepMat(0,varIdx) = trf(0,2);
      prepMat(1,varIdx) = trf(1,0);
      prepMat(2,varIdx) = trf(2,1);
      prepMat(3,varIdx) = trf(0,3);
      prepMat(4,varIdx) = trf(1,3);
      prepMat(5,varIdx) = trf(2,3);

      rhs[varIdx] -= trf(0,2) * accSum(0,2);
      rhs[varIdx] -= trf(1,0) * accSum(1,0);
      rhs[varIdx] -= trf(2,1) * accSum(2,1);
      rhs[varIdx] -= trf(0,3) * accSum(0,3);
      rhs[varIdx] -= trf(1,3) * accSum(1,3);
      rhs[varIdx] -= trf(2,3) * accSum(2,3);
    
      idxLst[idxLstSz++] = varIdx;
    }

    if (atBody1) {
      curTrf.preMultWith(jnt.pos);
      curTrf.preMultWith(grp.getInvPos2());
    }
    else {
      curTrf.preMultWith(jnt.invPos);
      curTrf.preMultWith(grp.getInvPos1());
    }

    body = grp.getOtherBody(*body);
  }

  for (int i=0; i<6; ++i) {
    for (int j=0; j<idxLstSz; ++j) {
      int colIdx = idxLst[j];

      double m = prepMat(i,colIdx);

      for (int k=0; k<idxLstSz; ++k) {
        int rowIdx = idxLst[k];
        if (rowIdx < colIdx) continue;

        // Store element of AT-A, optimized storage, see Matrix::solveLDLT()
        solMat(colIdx,rowIdx-colIdx) += m * prepMat(i,rowIdx);
      }
    }
  }

  if (varSz > MaxAllocaSz) delete[] idxLst;

  return true;
}

//---------------------------------------------------------------------------

bool Topology::composeAccelEq()
{
  solMat.clear();
  rhs.clear();

  topoGripLst.clearJointTrfCaches();

  // model.func_lst.updateAllAcc();

  bool ok = true;
  int sz = size();

  for (int i=0; i<sz; i++) {
    GripList& grpLst = *get(i);

    if (!composeAccelMatrixRow(grpLst)) ok = false;
  }

  return ok;
}

//---------------------------------------------------------------------------

bool Topology::solveAccel(Vector& accelVec)
{
  accelValid = false;
  jerkValid  = false;

  accelVec.setSize(varSz);

  if (!composeAccelEq()) return false;

  solMat.solveLDLT(rhs);

  accelVec = rhs;

  updateAccels();

  accelValid = true;

  return true;
}

//---------------------------------------------------------------------------

bool Topology::getAccelVector(Ino::Vector& accelVec, bool fixed) const
{
  if (!accelValid) {
    accelVec.setSize(0);
    return false;
  }

  if (fixed) accelVec.setSize(fixedSz);
  else accelVec.setSize(varSz);

  accelVec.clear();

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->getAccels(fixed, accelVec);
  }

  return true;
}

//---------------------------------------------------------------------------

void Topology::setAccelVector(const Ino::Vector& accelVec, bool fixed)
{
  jerkValid  = false;

  if (accelVec.size() != (fixed ? fixedSz : varSz)) {
    accelValid = false;
    return;
  }

  accelValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->setAccels(fixed,accelVec);
  }
}

//---------------------------------------------------------------------------

void Topology::setAccelVectors(const Vector& varAccelVec,
                                               const Vector& fixedAccelVec)
{
  jerkValid  = false;

  if (varAccelVec.size() != varSz || fixedAccelVec.size() != fixedSz) {
    accelValid = false;
    return;
  }

  accelValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->setAccels(varAccelVec, fixedAccelVec);
  }
}

//---------------------------------------------------------------------------

static void updateAccel(Grip& grip)
{
  Body *body1 = grip.getBody1();
  Body *body2 = grip.getBody2();

  if (body1->getTreeLevel() < body2->getTreeLevel()) {
    Trf3 pos(body1->getPos());
    Trf3 speed(body1->getSpeed());
    Trf3 acc(body1->getAccel());

    pos.preMultWith(grip.getPos1());
    speed.preMultWith(grip.getPos1());
    acc.preMultWith(grip.getPos1());

    acc.preMultWith(grip.getJoint()->getPos());

    speed.preMultWith(grip.getJoint()->getDer());
    speed *= 2.0;
    acc += speed;

    pos.preMultWith(grip.getJoint()->getAcc());
    acc += pos;

    acc.preMultWith(grip.getInvPos2());

    body2->setAccel(acc);
  }
  else {
    Trf3 pos(body2->getPos());
    Trf3 speed(body2->getSpeed());
    Trf3 acc(body2->getAccel());

    pos.preMultWith(grip.getPos2());
    speed.preMultWith(grip.getPos2());
    acc.preMultWith(grip.getPos2());

    acc.preMultWith(grip.getJoint()->getInvPos());

    speed.preMultWith(grip.getJoint()->getInvDer());
    speed *= 2.0;
    acc += speed;

    pos.preMultWith(grip.getJoint()->getInvAcc());
    acc += pos;

    acc.preMultWith(grip.getInvPos1());

    body1->setAccel(acc);
  }
}

//---------------------------------------------------------------------------

bool Topology::updateAccels()
{
  if (!accelValid) return false;

  int sz = topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = topoGripLst[i];
    if (!grp->parentRel) continue;

    updateAccel(*grp);
  }

  return true;
}

//---------------------------------------------------------------------------

bool Topology::composeJerkMatrixRow(const GripList& grpLst)
{
  Trf3 curTrf;
  if (!grpLst.setPreTrfs(curTrf)) return false;

  int grpSz = grpLst.size();
  if (grpSz < 1) return false;

  Body *body = grpLst.firstBody();
  if (!body) return false;

  enum { MaxAllocaSz = 32000 }; // Max nr of items on stack

  int idxLstSz = 0;
  int *idxLst;
 
  if (varSz > MaxAllocaSz) idxLst = new int[varSz];
  else idxLst = (int *)_malloca(varSz*sizeof(int));

  for (int i=0; i<varSz; ++i) idxLst[i] = -1;

  curTrf.init();
 
  Trf3 secDer, thrdDer, der;

  for (int i=grpSz-1; i>=0; i--) {
    Grip& grp = *grpLst[i];
    AbstractJoint& jnt = *grp.getJoint();

    Trf3 jerkSum;
    jerkSum.zero();

    bool atBody1 = body == grp.getBody1();

    if (atBody1) {
      curTrf.preMultWith(grp.getPos1());
      jnt.getJerkMixed(jerkSum);
    }
    else {
      curTrf.preMultWith(grp.getPos2());
      jnt.getInvJerkMixed(jerkSum);
    }

    int varCnt = jnt.getVarCnt();

    for (int j=0; j<varCnt; ++j) {
      if (atBody1) {
        jnt.getSecDerivative(j,secDer);
        jnt.getThirdDerivative(j,thrdDer);
      }
      else {
        jnt.getInvSecDerivative(j,secDer);
        jnt.getInvThirdDerivative(j,thrdDer);
      }

      thrdDer *= sqr(jnt.getSpeed(j)) * jnt.getSpeed(j);
      secDer *= 3.0 * jnt.getSpeed(j) * jnt.getAccel(j);

      jerkSum += thrdDer;
      jerkSum += secDer;

      if (!jnt.getFixed(j)) continue;

      // Only fixed vars:

      if (atBody1) der = jnt.getDerivative(j);
      else der = jnt.getInvDerivative(j);

      der *= jnt.getJerk(j);
      jerkSum += der;
    }

    jerkSum *= curTrf;

    const Trf3& preTrf = grpLst.getPreTrf(i); 
    jerkSum.preMultWith(preTrf);

    for (int j=0; j<varCnt; j++) {
      if (jnt.getFixed(j)) continue;

      if (atBody1) jnt.getThirdDerivative(j,thrdDer);
      else jnt.getInvThirdDerivative(j,thrdDer);

      Trf3 trf(curTrf);
      trf.preMultWith(thrdDer);
      trf.preMultWith(preTrf);

      int varIdx = jnt.getVarIdx(j);

      prepMat(0,varIdx) = trf(0,2);
      prepMat(1,varIdx) = trf(1,0);
      prepMat(2,varIdx) = trf(2,1);
      prepMat(3,varIdx) = trf(0,3);
      prepMat(4,varIdx) = trf(1,3);
      prepMat(5,varIdx) = trf(2,3);

      rhs[varIdx] -= trf(0,2) * jerkSum(0,2);
      rhs[varIdx] -= trf(1,0) * jerkSum(1,0);
      rhs[varIdx] -= trf(2,1) * jerkSum(2,1);
      rhs[varIdx] -= trf(0,3) * jerkSum(0,3);
      rhs[varIdx] -= trf(1,3) * jerkSum(1,3);
      rhs[varIdx] -= trf(2,3) * jerkSum(2,3);
    
      idxLst[idxLstSz++] = varIdx;
    }

    if (atBody1) {
      curTrf.preMultWith(jnt.pos);
      curTrf.preMultWith(grp.getInvPos2());
    }
    else {
      curTrf.preMultWith(jnt.invPos);
      curTrf.preMultWith(grp.getInvPos1());
    }

    body = grp.getOtherBody(*body);
  }

  for (int i=0; i<6; ++i) {
    for (int j=0; j<idxLstSz; ++j) {
      int colIdx = idxLst[j];

      double m = prepMat(i,colIdx);

      for (int k=0; k<idxLstSz; ++k) {
        int rowIdx = idxLst[k];
        if (rowIdx < colIdx) continue;

        // Store element of AT-A, optimized storage, see Matrix::solveLDLT()
        solMat(colIdx,rowIdx-colIdx) += m * prepMat(i,rowIdx);
      }
    }
  }

  if (varSz > MaxAllocaSz) delete[] idxLst;

  return true;
}

//---------------------------------------------------------------------------

bool Topology::composeJerkEq()
{
  solMat.clear();
  rhs.clear();

  topoGripLst.clearJointTrfCaches();

  // model.func_lst.updateAllJerk();

  bool ok = true;
  int sz = size();

  for (int i=0; i<sz; i++) {
    GripList& grpLst = *get(i);

    if (!composeJerkMatrixRow(grpLst)) ok = false;
  }

  return ok;
}

//---------------------------------------------------------------------------

bool Topology::solveJerk(Vector& jerkVec)
{
  jerkValid  = false;

  jerkVec.setSize(varSz);

  if (!composeJerkEq()) return false;

  solMat.solveLDLT(rhs);

  jerkVec = rhs;

  updateJerks();

  jerkValid = true;

  return true;
}

//---------------------------------------------------------------------------

bool Topology::getJerkVector(Ino::Vector& jerkVec, bool fixed) const
{
  if (!jerkValid) {
    jerkVec.setSize(0);
    return false;
  }

  if (fixed) jerkVec.setSize(fixedSz);
  else jerkVec.setSize(varSz);

  jerkVec.clear();

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->getJerks(fixed, jerkVec);
  }

  return true;
}

//---------------------------------------------------------------------------

void Topology::setJerkVector(const Ino::Vector& jerkVec, bool fixed)
{
  if (jerkVec.size() != (fixed ? fixedSz : varSz)) {
    jerkValid = false;
    return;
  }

  jerkValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->setJerks(fixed,jerkVec);
  }
}

//---------------------------------------------------------------------------

void Topology::setJerkVectors(const Vector& varJerkVec,
                                              const Vector& fixedJerkVec)
{
  if (varJerkVec.size() != varSz || fixedJerkVec.size() != fixedSz) {
    jerkValid = false;
    return;
  }

  jerkValid = true;

  int grpSz = topoGripLst.size();

  for (int i=0; i<grpSz; i++) {
    AbstractJoint *jnt = topoGripLst[i]->getJoint();

    if (jnt) jnt->setJerks(varJerkVec,fixedJerkVec);
  }
}

//---------------------------------------------------------------------------

static void updateJerk(Grip& grip)
{
  Body *body1 = grip.getBody1();
  Body *body2 = grip.getBody2();

  if (body1->getTreeLevel() < body2->getTreeLevel()) {
    Trf3 pos(body1->getPos());
    Trf3 speed(body1->getSpeed());
    Trf3 acc(body1->getAccel());
    Trf3 jerk(body1->getJerk());

    pos.preMultWith(grip.getPos1());
    speed.preMultWith(grip.getPos1());
    acc.preMultWith(grip.getPos1());
    jerk.preMultWith(grip.getPos1());

    jerk.preMultWith(grip.getJoint()->getPos());

    acc.preMultWith(grip.getJoint()->getDer());
    acc *= 3.0;
    jerk += acc;

    speed.preMultWith(grip.getJoint()->getAcc());
    speed *= 3.0;
    jerk += speed;

    pos.preMultWith(grip.getJoint()->getJerk());
    jerk += pos;

    jerk.preMultWith(grip.getInvPos2());

    body2->setJerk(jerk);
  }
  else {
    Trf3 pos(body2->getPos());
    Trf3 speed(body2->getSpeed());
    Trf3 acc(body2->getAccel());
    Trf3 jerk(body2->getJerk());

    pos.preMultWith(grip.getPos2());
    speed.preMultWith(grip.getPos2());
    acc.preMultWith(grip.getPos2());
    jerk.preMultWith(grip.getPos2());

    jerk.preMultWith(grip.getJoint()->getInvPos());

    acc.preMultWith(grip.getJoint()->getInvDer());
    acc *= 3.0;
    jerk += acc;

    speed.preMultWith(grip.getJoint()->getInvAcc());
    speed *= 3.0;
    jerk += speed;

    pos.preMultWith(grip.getJoint()->getInvJerk());
    jerk += pos;

    jerk.preMultWith(grip.getInvPos1());

    body1->setJerk(jerk);
  }
}

//---------------------------------------------------------------------------

bool Topology::updateJerks()
{
  if (!jerkValid) return false;

  int sz = topoGripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = topoGripLst[i];
    if (!grp->parentRel) continue;

    updateJerk(*grp);
  }

  return true;
}

//---------------------------------------------------------------------------

void Topology::transform(const Trf3& trf) const
{
  int bSz = topoBodyLst.size();

  for (int i=1; i<bSz; i++) {
    Body *body = topoBodyLst[i];
    body->transform(trf);
  }
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

static Body *findUnseenBody(const BodyList& bodyLst)
{
  int sz = bodyLst.size();

  for (int i=0; i<sz; ++i) {
    Body *body = bodyLst[i];
    if (body->getTreeLevel() < 0) return body;
  }

  return NULL;
}

//-------------------------------------------------------------------------------

bool TopologyList::prepareAll()
{
  clear();

  if (!model) return false;

  model->setTopoModified();

  BodyList& bodyLst = model->bodyLst;

  int bSz = bodyLst.size();
  if (bSz < 1) return false;

  GripList& gripLst = model->gripLst;

  int gSz = gripLst.size();
  if (gSz < 1) return false;

  for (int i=0; i<bSz; ++i) bodyLst[i]->setTreeLevel(-1);
  for (int i=0; i<gSz; ++i) {
    gripLst[i]->loopCnt = 0;
    gripLst[i]->parentRel = false;
  }

  for (;;) {
    Body *fstBody = findUnseenBody(bodyLst);
    if (!fstBody) break;

    Topology *topo = new Topology(*model);

    topo->prepare(fstBody);

    if (topo->size() < 1) delete topo;
    else add(topo);
  }

  return size() > 0;
}

//-------------------------------------------------------------------------------

void TopologyList::transform(const Ino::Trf3& trf) const
{
  int sz = size();

  for (int i=0; i<sz; ++i) get(i)->transform(trf);
}

} // namespace

// Interface Section

static InoKin::Model* GetModelTopology(void* topo)
{
  InoKin::Topology* topology= (InoKin::Topology*)topo;

  return topology->getModel();
}

// End Interface Section

//-------------------------------------------------------------------------------
