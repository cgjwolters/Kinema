//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------- A Kinematic Body -----------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_BODY_INC
#define INOKIN_BODY_INC

#include "KinObject.h"
#include "KinObjList.h"

#include "KinProbe.h"

#include "Vec.h"
#include "Trf.h"

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

class Grip;
class GripList;
class Model;

class Body : public Object
{
  Trf3 position;
  Ino::Trf3 speed;
  Ino::Trf3 accel;
  Ino::Trf3 jerk;

  int treeLvl;

  GripList& gripLst;
  ProbeList& probeLst;

  Body(const Body& cp) = delete;             // No copying
  Body& operator=(const Body& src) = delete; // No Assignment

  explicit Body(Model& model, const Body& cp);

  void add(Grip *grip);
  void remove(const Grip *grip);

  void setTreeLevel(int newLevel) { treeLvl = newLevel; }

public:
  explicit Body(Model& model, const wchar_t *name);
  explicit Body(Model& model, const wchar_t *name, const Ino::Trf3& init_pos);
  ~Body();

  Body *getParent() const;
  Grip *getParentGrip() const;
  int getTreeLevel() const { return treeLvl; }

  Grip *gripTo(const Body& otherBody) const;

  const Ino::Trf3& getPos() const { return position; }
  void setPos(const Ino::Trf3& newPos);
  void getInvPos(Ino::Trf3& invPos) const;

  const Ino::Trf3& getSpeed() const { return speed; }
  void setSpeed(const Ino::Trf3& newSpeed);
  void getInvSpeed(Ino::Trf3& invSpeed) const;

  const Ino::Trf3& getAccel() const { return accel; }
  void setAccel(const Ino::Trf3& newAccel);
  void getInvAccel(Ino::Trf3& invAccel) const;

  const Ino::Trf3& getJerk() const { return jerk; }
  void setJerk(const Ino::Trf3& newJerk);
  void getInvJerk(Ino::Trf3& invJerk) const;

  void getAbsPos(Ino::Trf3& trf) const;
  void getAbsPos(Ino::Vec3& p) const;

  void getAbsSpeed(Ino::Vec3& p) const;

  void translate(const Ino::Vec3& offset);
  void transform(const Ino::Trf3& trf);

  const GripList& getGripList() const { return gripLst; }

  void getAbsGripPos(int idx, Ino::Trf3& trf) const;
  void getAbsGripPos(const wchar_t *name, Ino::Trf3& trf) const;

 // const ProbeList& getProbeList() const { return probeLst; }

  friend class Grip;
  friend class Probe;
  friend class Model;
  friend class Topology;
  friend class TopologyList;
};

//---------------------------------------------------------------------------

class BodyList: public ObjList<Body>
{
  BodyList& operator=(const BodyList& src); // No assignment

  public:
    BodyList() : ObjList<Body>(false) {}
    BodyList(const BodyList& cp) : ObjList<Body>(cp) {}
};

} // namespace

// Interface Section

extern "C" __declspec(dllexport) void* BodyNew(InoKin::Model& model, const wchar_t* name);

// End Interface Section

//---------------------------------------------------------------------------
#endif
