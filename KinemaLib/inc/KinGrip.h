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

#ifndef INOKIN_GRIP_INC
#define INOKIN_GRIP_INC

#include "KinObject.h"
#include "KinObjList.h"

#include "Trf.h"
#include "Array.h"

#include <vector>
#include <set>

namespace InoKin {

//---------------------------------------------------------------------------

class Body;
class AbstractJoint;
class Model;
class GripList;

class Grip : public Object
{
  Ino::Trf3& pos1, &invPos1, &pos2, &invPos2;
  Body *body1, *body2;
  AbstractJoint *joint;

  bool parentRel;
  int loopCnt;

  Grip(const Grip& cp) = delete;             // No copying
  Grip& operator=(const Grip& src) = delete; // No assignment

  explicit Grip(Model& model, const Grip& cp);

  void setBody1(Body& body);
  void setBody2(Body& body);

public:
  explicit Grip(Model& model, const wchar_t *name,
                Body& body_1, const Ino::Trf3& pos_1,
                Body& body_2, const Ino::Trf3& pos_2);
  ~Grip();

  void setPos1(const Ino::Trf3& pos);
  void setPos2(const Ino::Trf3& pos);

  Body *getBody1() const { return body1; }
  Body *getBody2() const { return body2; }
  
  Body *getOtherBody(const Body& body) const;
  AbstractJoint *getJoint() const { return joint; }

  bool isParentRel() const { return parentRel; }
  int getLoopCnt() const { return loopCnt; }

  const Ino::Trf3& getPos1() const { return pos1; }
  const Ino::Trf3& getInvPos1() const { return invPos1; }

  const Ino::Trf3& getPos2() const { return pos2; }
  const Ino::Trf3& getInvPos2() const { return invPos2; }

  friend class AbstractJoint;
  friend class Model;
  friend class Topology;
  friend class TopologyList;
};

//---------------------------------------------------------------------------

class GripList;

class GripListCompare
{
public:
  bool operator()(const GripList *lst1, const GripList *lst2) const;
};

//---------------------------------------------------------------------------

class GripList : public ObjList<Grip>
{
  typedef std::set<GripList *,GripListCompare> PeerSet;
  PeerSet peerSet;

  mutable std::vector<Ino::Trf3> preTrfLst;

  int loopLvl;

public:
  GripList();
  GripList(const GripList& cp);

  GripList& operator=(const GripList& src);

  void setAngularVars(bool *angularVar) const;

  void clearJointTrfCaches();

  Body *firstBody() const;

  bool setPreTrfs(Ino::Trf3& loopTrf) const;
  const Ino::Trf3& getPreTrf(int idx) const;

  void setJointsFixedAll(bool fixed) const;
  void setJointsZeroAll() const;

  AbstractJoint *getJoint(const wchar_t *jntName) const;

  friend class Topology;
};

typedef Ino::Array<GripList *> LoopList;

} // namespace

//---------------------------------------------------------------------------
#endif
