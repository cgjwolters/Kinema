//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//------ Kinema: Kinematic Simulation Program ---------------------------
//-----------------------------------------------------------------------
//---------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//---------------------------------------------- C.Wolters --------------
//-----------------------------------------------------------------------
//------ A Kinematic Body -----------------------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

#include "KinBody.h"

#include "KinProbe.h"
#include "KinGrip.h"
#include "KinFunction.h"
#include "KinModel.h"

#include "Exceptions.h"

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

Body::Body(Model& model, const wchar_t *name)
: Object(model,name),
  position(), speed(), accel(), jerk(),
  gripLst(*new GripList()), probeLst(*new ProbeList(true)),
  treeLvl(-1)
{
  speed.zero();
  speed.isDerivative = true;

  accel.isDerivative = true;
  accel.zero();

  jerk.isDerivative = true;
  jerk.zero();

  model.adopt(this);
}

//---------------------------------------------------------------------------

Body::Body(Model& model, const wchar_t *name, const Trf3& initPos)
: Object(model,name),
  position(initPos), speed(), accel(), jerk(),
  gripLst(*new GripList()), probeLst(*new ProbeList(true)), treeLvl(-1)
{
  speed.zero();
  speed.isDerivative = true;

  accel.zero();
  accel.isDerivative = true;

  jerk.zero();
  jerk.isDerivative = true;

  model.adopt(this);
}

//---------------------------------------------------------------------------

Body::Body(Model& model, const Body& cp)
: Object(model,cp),
  position(cp.position), speed(cp.speed), accel(cp.accel), jerk(cp.jerk),
  gripLst(*new GripList()),
  probeLst(*new ProbeList(true)),
  treeLvl(cp.treeLvl)
{
  int prbCnt = cp.probeLst.size();
  for (int i=0; i<prbCnt; i++) new Probe(*this,*cp.probeLst[i]);
}

//---------------------------------------------------------------------------

Body::~Body()
{
  model.remove(this);

  delete &gripLst;
  delete &probeLst;
}

//---------------------------------------------------------------------------

Body *Body::getParent() const
{
  Grip *grip = getParentGrip();

  if (!grip) return NULL;

  return grip->getOtherBody(*this);
}

//---------------------------------------------------------------------------

Grip *Body::getParentGrip() const
{
  int sz = gripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = gripLst[i];
    if (!grp->isParentRel()) continue;

    Body *body = grp->getOtherBody(*this);

    if (body && body->treeLvl < treeLvl) return grp;
  }

  return NULL;
}

//---------------------------------------------------------------------------

Grip *Body::gripTo(const Body& otherBody) const
{
  int sz = gripLst.size();

  for (int i=0; i<sz; ++i) {
    Grip *grp = gripLst[i];

    if (grp->getOtherBody(otherBody) == this) return grp;
  }

  return NULL;
}

//---------------------------------------------------------------------------

void Body::add(Grip *grip)
{
  gripLst.add(grip);
  setModelTopoModified();
}

//---------------------------------------------------------------------------

void Body::remove(const Grip *grip)
{
  gripLst.remove(grip);
  setModelTopoModified();
}

//---------------------------------------------------------------------------

void Body::setPos(const Ino::Trf3& newPos)
{
  position = newPos;
  setModelModified();
}

//---------------------------------------------------------------------------

void Body::getInvPos(Trf3& invPos) const
{
  invPos = position;
  invPos.invert();
}

//---------------------------------------------------------------------------

void Body::setSpeed(const Ino::Trf3& newSpeed)
{
  speed = newSpeed;
  setModelModified();
}

//---------------------------------------------------------------------------

void Body::getInvSpeed(Trf3& invSpeed) const
{
  Trf3 invPos(position); invPos.invert();

  invSpeed = speed;

  invSpeed *= invPos;
  invSpeed.preMultWith(invPos);
  invSpeed *= -1.0;
}

//---------------------------------------------------------------------------

void Body::setAccel(const Ino::Trf3& newAccel)
{
  accel = newAccel;
  setModelModified();
}

//---------------------------------------------------------------------------

void Body::getInvAccel(Trf3& invAccel) const
{
  Trf3 invPos(position); invPos.invert();

  invAccel = invPos;
  invAccel *= speed;
  invAccel.preMultWith(speed);
  invAccel *= 2.0;

  invAccel -= accel;

  invAccel *= invPos;
  invAccel.preMultWith(invPos);
}

//---------------------------------------------------------------------------

void Body::setJerk(const Trf3& newJerk)
{
  jerk = newJerk;
  setModelModified();
}

//---------------------------------------------------------------------------

void Body::getInvJerk(Trf3& invJerk) const
{
  Trf3 invPos(position); invPos.invert();

  Trf3 dTrf(speed); dTrf *= invPos;
  Trf3 aTrf(accel); aTrf *= invPos;

  invJerk = dTrf; invJerk *= dTrf; invJerk *= 2.0;
  invJerk += aTrf; 
  invJerk.preMultWith(dTrf);

  aTrf *= dTrf;
  invJerk += aTrf;
  invJerk *= 3.0;

  dTrf = jerk; dTrf *= invPos;
  invJerk -= dTrf;
  invJerk.preMultWith(invPos);
}

//---------------------------------------------------------------------------

void Body::getAbsPos(Trf3& trf) const
{
  position.invertInto(trf);
}

//---------------------------------------------------------------------------

void Body::getAbsPos(Vec3& p) const
{
  Trf3 trf;
  position.invertInto(trf);

  p = trf * Vec3(0,0,0);
}

//---------------------------------------------------------------------------

void Body::getAbsSpeed(Vec3& p) const
{
  Trf3 isp;
  getInvSpeed(isp);

  p = isp * Vec3(0,0,0);
}

//---------------------------------------------------------------------------

void Body::translate(const Vec3& offset)
{
  position(0,3) += offset.x;
  position(1,3) += offset.y;
  position(2,3) += offset.z;

  setModelModified();
}

//---------------------------------------------------------------------------

void Body::transform(const Trf3& trf)
{
  position *= trf;
  speed    *= trf;
  accel    *= trf;
  jerk     *= trf;

  setModelModified();
}

//---------------------------------------------------------------------------

void Body::getAbsGripPos(int idx,Trf3& trf) const
{
  if (idx < 0 || idx >= gripLst.size()) {
    trf = Trf3();
    return;
  }

  position.invertInto(trf);

  if (gripLst[idx]->getBody1() == this) trf *= gripLst[idx]->getInvPos1();
  else                                  trf *= gripLst[idx]->getInvPos2();
}

//---------------------------------------------------------------------------

void Body::getAbsGripPos(const wchar_t *name, Trf3& trf) const
{
  const Grip *grp = gripLst[name];

  if (!grp) {
    trf = Trf3();
    return;
  }

  position.invertInto(trf);

  if (grp->getBody1() == this) trf *= grp->getInvPos1();
  else                         trf *= grp->getInvPos2();
}

} // namespace

// Interface Section

void* BodyNew(InoKin::Model& model, const wchar_t* name) {
  InoKin::Body* body = new InoKin::Body(model, name);

  return body;
}

InoKin::Body* GetParentBody(void* cppBody) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  return body->getParent();
}

InoKin::Grip* GetParentGripBody(void* cppBody) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  return body->getParentGrip();
}

int GetTreeLevelBody(void *cppBody) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  return body->getTreeLevel();
}

InoKin::Grip* GripToBody(void* cppBody, InoKin::Body* otherBody) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  return body->gripTo(*otherBody);
}

const Ino::Trf3& GetPosBody(void* cppBody) {
  const InoKin::Body* body = (InoKin::Body*)cppBody;

  return body->getPos();
}

void SetPosBody(void* cppBody, Trf3& newPos)   {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->setPos(newPos);
}

void GetInvPosBody(void* cppBody, Trf3& invPos) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->getInvPos(invPos);
}

void GetSpeedBody(void* cppBody, Trf3& trf) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  trf = body->getSpeed();
}

void SetSpeedBody(void* cppBody, Trf3 newSpeed) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->setSpeed(newSpeed);
}

void GetInvSpeedBody(void* cppBody, Trf3& invSpeed) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->getInvSpeed(invSpeed);
}

void GetAccelBody(void* cppBody, Trf3& accel) {
  InoKin::Body* body = (InoKin::Body*)cppBody;
  accel = body->getAccel();
}

void SetAccelBody(void* cppBody, Trf3 newAccel) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->setAccel(newAccel);
}

void GetInvAccelBody(void* cppBody, Trf3& invAccel) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->getInvAccel(invAccel);
}

void GetJerkBody(void* cppBody, Trf3& trf) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  trf = body->getJerk();
}

void SetJerkBody(void* cppBody, Trf3 newJerk) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->setJerk(newJerk);
}

void GetInvJerkBody(void* cppBody, Trf3& invJerk) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  invJerk = body->getJerk();
}

void GetAbsPosBody(void* cppBody, Vec3& pos) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->getAbsPos(pos);
}

void GetAbsPosBody2(void* cppBody, Trf3& trf) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->getAbsPos(trf);
}

void GetAbsSpeedBody(void* cppBody, Vec3& speed) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->getAbsSpeed(speed);
}

void TranslateBody(void* cppBody, Vec3 offset) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->translate(offset);
}

void TransformBody(void* cppBody, Trf3 trf) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  body->transform(trf);
}

int GetGripCountBody(void* cppBody) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  return body->getGripList().size();
}

InoKin::Grip* GetGripBody(void* cppBody, int idx) {
  InoKin::Body* body = (InoKin::Body*)cppBody;

  if (idx < 0 || idx >= body->getGripList().size()) return nullptr;

  return body->getGripList()[idx];
}

// End Interface Section

//---------------------------------------------------------------------------
