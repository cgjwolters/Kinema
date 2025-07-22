//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- An oriented joint attachment point on a pair of bodies ---------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinGrip.h"

#include "KinBody.h"
#include "KinAbstractJoint.h"
#include "KinFunction.h"
#include "KinModel.h"

#include "Exceptions.h"

using namespace Ino;

namespace InoKin {

//-------------------------------------------------------------------------------

Grip::Grip(Model& model, const wchar_t *name,
           Body& body_1, const Trf3& pos_1,
           Body& body_2, const Trf3& pos_2)
: Object(model,name),
  pos1(*new Trf3(pos_1)), invPos1(*new Trf3()),
  pos2(*new Trf3(pos_2)), invPos2(*new Trf3()),
  body1(&body_1), body2(&body_2), joint(NULL),
  parentRel(false), loopCnt(0)
{
  pos1.invertInto(invPos1);
  pos2.invertInto(invPos2);

  model.adopt(this);
  body1->add(this);
  body2->add(this);
}

//-------------------------------------------------------------------------------

Grip::Grip(Model& model, const Grip& cp)
: Object(model,cp),
  pos1(*new Trf3(cp.pos1)), invPos1(*new Trf3(cp.invPos1)),
  pos2(*new Trf3(cp.pos2)), invPos2(*new Trf3(cp.invPos2)),
  body1(NULL), body2(NULL), joint(NULL),
  parentRel(cp.parentRel), loopCnt(cp.loopCnt)
{
  cp.joint->clone(*this);
}

//-------------------------------------------------------------------------------

Grip::~Grip()
{
  delete &pos1;
  delete &invPos1;
  delete &pos2;
  delete &invPos2;

  if (body1 != NULL) body1->remove(this);
  if (body2 != NULL) body2->remove(this);

  delete joint;

  Function *func = model.funcLst.findByConnection(*this);
  delete func;

  model.remove(this);
}

//-------------------------------------------------------------------------------

void Grip::setBody1(Body& body)
{
  if (body1 != NULL) body1->remove(this);
  body1 = &body;
  body1->add(this);
}

//-------------------------------------------------------------------------------

void Grip::setBody2(Body& body)
{
  if (body2 != NULL) body2->remove(this);
  body2 = &body;
  body2->add(this);
}

//-------------------------------------------------------------------------------

void Grip::setPos1(const Trf3& pos)
{
  pos1 = pos;
  pos1.invertInto(invPos1); 
}

//-------------------------------------------------------------------------------

void Grip::setPos2(const Trf3& pos)
{
  pos2 = pos;
  pos2.invertInto(invPos2); 
}

//-------------------------------------------------------------------------------

Body *Grip::getOtherBody(const Body& body) const
{
  if (&body == body1) return body2;
  if (&body == body2) return body1;

  return NULL;
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

GripList::GripList()
: ObjList<Grip>(false),peerSet(),preTrfLst(),loopLvl(-1)
{
}

//-------------------------------------------------------------------------------

GripList::GripList(const GripList& cp)
: ObjList<Grip>(cp),peerSet(),preTrfLst(),loopLvl(-1)
{
}


//-------------------------------------------------------------------------------

GripList& GripList::operator=(const GripList& src)
{
  ObjList<Grip>::operator=(src);

  peerSet.clear();
  preTrfLst.clear();
  loopLvl = -1;

  return *this;
}

//-------------------------------------------------------------------------------

void GripList::setAngularVars(bool *angularVar) const
{
  int sz = size();

  for (int i=0; i<sz; ++i) {
    AbstractJoint *jnt = get(i)->getJoint();
    if (!jnt) continue;

    int varCnt = jnt->getVarCnt();

    for (int j=0; j<varCnt; ++j) {
      int idx = jnt->getVarIdx(j);
      if (idx < 0) continue;

      angularVar[idx] = jnt->getIsAngular(j);
    }
  }
}

//-------------------------------------------------------------------------------

Body *GripList::firstBody() const
{
  int sz = size();

  if (sz < 1) return NULL;

  Body *body = get(0)->getBody1();

  if (body != get(sz-1)->getBody1() &&
      body != get(sz-1)->getBody2()) body = get(0)->getBody2();

  if (body != get(sz-1)->getBody1() &&
      body != get(sz-1)->getBody2()) return NULL;

  return body;
}

//-------------------------------------------------------------------------------

void GripList::clearJointTrfCaches()
{
  int sz = size();

  for (int i=0; i<sz; ++i) {
    Grip& curGrp = *get(i);
    AbstractJoint *jnt = get(i)->getJoint();

    if (jnt) jnt->clearTrfCaches();
  }
}

//-------------------------------------------------------------------------------

bool GripList::setPreTrfs(Trf3& loopTrf) const
{
  preTrfLst.clear();

  int sz = size();
  if (sz < 1) return false;

  preTrfLst.reserve(sz);

  Body *body = firstBody();
  if (!body) return false;

  loopTrf.init();

  for (int i=0; i<sz; i++) {
    Grip& curGrp = *get(i);
    AbstractJoint& jnt = *curGrp.getJoint();

    if (body == curGrp.getBody1()) {
      loopTrf *= curGrp.getInvPos1();

      preTrfLst.push_back(loopTrf);

      loopTrf *= jnt.invPos;
      loopTrf *= curGrp.getPos2();

      body = curGrp.getBody2();
    }
    else if (body == curGrp.getBody2()) {
      loopTrf *= curGrp.getInvPos2();

      preTrfLst.push_back(loopTrf);

      loopTrf *= jnt.pos;
      loopTrf *= curGrp.getPos1();

      body = curGrp.getBody1();
    }
    else throw IllegalStateException("GripList::setPreTrfs");
  }

  return true;
}

//-------------------------------------------------------------------------------

const Trf3& GripList::getPreTrf(int idx) const
{
  int sz = size();

  if (idx < 0 || idx >= sz || idx >= (int)preTrfLst.size())
    throw IndexOutOfBoundsException("GripList::getPreTrf");

  return preTrfLst[idx];
}

//-------------------------------------------------------------------------------

void GripList::setJointsFixedAll(bool fixed) const
{
  int sz = size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = get(i);

    AbstractJoint *jnt = grp->getJoint();
    if (!jnt) continue;
    
    int varSz = jnt->getVarCnt();

    for (int j=0; j<varSz; ++j) jnt->setFixed(j,fixed);
  }
}

//-------------------------------------------------------------------------------

void GripList::setJointsZeroAll() const
{
  int sz = size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = get(i);

    AbstractJoint *jnt = grp->getJoint();
    if (!jnt) continue;
    
    int varSz = jnt->getVarCnt();

    for (int j=0; j<varSz; ++j) {
      jnt->setSpeed(i,0);
      jnt->setAccel(i,0);
      jnt->setJerk(i,0);
    }
  }
}

//-------------------------------------------------------------------------------

AbstractJoint *GripList::getJoint(const wchar_t *jntName) const
{
  if (!jntName || !jntName[0]) return NULL;

  int sz = size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = get(i);

    AbstractJoint *jnt = grp->getJoint();
    if (!jnt) continue;

    if (!compareStr(jnt->getName(),jntName)) return jnt;
  }

  return NULL;
}

//-------------------------------------------------------------------------------

bool GripListCompare::operator()(const GripList *lst1, const GripList *lst2) const
{
  if (lst1->size() < lst2->size()) return true;
  if (lst1->size() > lst2->size()) return false;

  return lst1 < lst2;
}

} // namespace

// Interface Section

void* GripNew(void* cppModel, const wchar_t* name,
  void *cppBody1, void *cppPos1,
  void *cppBody2, void* cppPos2)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;
  InoKin::Body* body1 = (InoKin::Body*)cppBody1;
  InoKin::Body* body2 = (InoKin::Body*)cppBody2;
  Ino::Trf3* pos1 = (Ino::Trf3*)cppPos1;
  Ino::Trf3* pos2 = (Ino::Trf3*)cppPos2;

  InoKin::Grip* grip = new InoKin::Grip(*mdl, name, *body1, *pos1, *body2, *pos2);

  return grip;
}

void SetPos1Grip(void* cppGrip, Trf3& pos) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  grip->setPos1(pos);
}

void SetPos2Grip(void* cppGrip, Trf3& pos) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  grip->setPos2(pos);
}

InoKin::Body* getBody1Grip(void* cppGrip) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  return grip->getBody1();
}

InoKin::Body* getBody2Grip(void* cppGrip) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  return grip->getBody2();
}

InoKin::Body* getOtherBodyGrip(void* cppGrip, const InoKin::Body& body) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  return grip->getOtherBody(body);
}

InoKin::AbstractJoint* getJointGrip(void* cppGrip) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  return grip->getJoint();
}

bool isParentRelGrip(void* cppGrip) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  return grip->isParentRel();
}

int getLoopCntGrip(void* cppGrip) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  return grip->getLoopCnt();
}

void getPos1Grip(void* cppGrip, Trf3& trf) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  trf = grip->getPos1();
}

void getInvPos1Grip(void* cppGrip, Trf3& trf) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  trf = grip->getInvPos1();
}

void getPos2Grip(void* cppGrip, Trf3& trf) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  trf = grip->getPos2();
}

void getInvPos2Grip(void* cppGrip, Trf3& trf) {
  InoKin::Grip* grip = (InoKin::Grip*)cppGrip;

  trf = grip->getInvPos2();
}

// End Interface Section

//-------------------------------------------------------------------------------
