//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- An oriented probe point on a body ------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinProbe.h"

#include "KinBody.h"
#include "KinFunction.h"
#include "KinModel.h"

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

Probe::Probe(Body& prbBody, const wchar_t *name, const Trf3& initPos)
: Object(prbBody.model,name),
  pos(*new Trf3(initPos)),
  body(prbBody)
{
  body.probeLst.add(this);
  
  setModelModified();
}

//---------------------------------------------------------------------------

Probe::Probe(Body& prbBody, const Probe& cp)
: Object(prbBody.model,cp),
  pos(* new Trf3(cp.pos)),
  body(prbBody)
{
  body.probeLst.add(this);

  setModelModified();
}

//---------------------------------------------------------------------------

Probe::~Probe()
{
  Function *func = model.getFunctionList().findByConnection(*this);
  if (func) delete func;

  // TODO solve mem leak
  // body.probeLst.remove(this);

  delete &pos;

  setModelModified();
}

//---------------------------------------------------------------------------

void Probe::getAbsPos(Trf3& absPos) const
{
  body.position.invertInto(absPos);

  Trf3 invPos;
  pos.invertInto(invPos);

  absPos *= invPos;
}

//---------------------------------------------------------------------------

void Probe::getAbsPos(Vec3& absPos) const
{
  Trf3 pos;
  getAbsPos(pos);

  absPos = Vec3();
  absPos.transform3(pos);
}

//---------------------------------------------------------------------------

void Probe::getAbsSpeed(Trf3& absSpeed) const
{
  body.getInvSpeed(absSpeed);

  Trf3 invPos;
  pos.invertInto(invPos);

  absSpeed *= invPos;
}

//---------------------------------------------------------------------------

void Probe::getAbsSpeed(Vec3& absSpeed) const
{
  Trf3 sp;
  getAbsSpeed(sp);

  absSpeed = Vec3();
  absSpeed.transform3(sp);
}

//---------------------------------------------------------------------------

void Probe::getAbsAccel(Trf3& absAccel) const
{
  body.getInvAccel(absAccel);

  Trf3 invPos;
  pos.invertInto(invPos);

  absAccel *= invPos;
}

//---------------------------------------------------------------------------

void Probe::getAbsAccel(Vec3& absAccel) const
{
  Trf3 acc;
  getAbsAccel(acc);

  absAccel = Vec3();
  absAccel.transform3(acc);
}

} // namespace

// Interface section

void* ProbeNew(InoKin::Body& body, const wchar_t* name, Ino::Trf3& initPos)
{
  InoKin::Probe* probe = new InoKin::Probe(body, name, initPos);

  return probe;
}

void GetAbsPosProbe(void* cppPrb, Ino::Vec3& apos)
{
  InoKin::Probe* probe = (InoKin::Probe*)cppPrb;

  probe->getAbsPos(apos);
}

void GetAbsPosProbe2(void* cppPrb, Ino::Trf3& apos)
{
  InoKin::Probe* probe = (InoKin::Probe*)cppPrb;

  probe->getAbsPos(apos);
}

void GetAbsSpeedProbe(void* cppPrb, Ino::Vec3& asp)
{
  InoKin::Probe* probe = (InoKin::Probe*)cppPrb;

  probe->getAbsSpeed(asp);
}

void GetAbsSpeedProbe2(void* cppPrb, Ino::Trf3& asp)
{
  InoKin::Probe* probe = (InoKin::Probe*)cppPrb;

  probe->getAbsSpeed(asp);
}

void GetAbsAccelProbe(void* cppPrb, Ino::Vec3& acc)
{
  InoKin::Probe* probe = (InoKin::Probe*)cppPrb;

  probe->getAbsAccel(acc);
}

void GetAbsAccelProbe2(void* cppPrb, Ino::Trf3& acc)
{
  InoKin::Probe* probe = (InoKin::Probe *)cppPrb;

  probe->getAbsAccel(acc);
}

void GetPosProbe(void* cppPrb, Ino::Trf3& pos)
{
  InoKin::Probe* probe = (InoKin::Probe*)cppPrb;

  pos = probe->getPos();
}

void SetPosProbe(void* cppPrb, Ino::Trf3& newPos)
{
  InoKin::Probe* probe = (InoKin::Probe*)cppPrb;

  probe->setPos(newPos);
}

// End Interface section

//---------------------------------------------------------------------------
