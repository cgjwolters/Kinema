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

#ifndef INOKIN_PROBE_INC
#define INOKIN_PROBE_INC

#include "KinObject.h"
#include "KinObjList.h"

#include "Vec.h"
#include "Trf.h"

namespace InoKin {

//---------------------------------------------------------------------------

class Body;

class Probe : public Object
{
  Ino::Trf3& pos;

  Probe(const Probe& cp) = delete;             // No copying
  Probe& operator=(const Probe& src) = delete; // No Assignment

public:
  Body& body;

  explicit Probe(Body& prbBody, const wchar_t *name, const Ino::Trf3& initPos);
  explicit Probe(Body& prbBody, const Probe& cp);
  ~Probe();

  void getAbsPos(Ino::Trf3& apos) const;
  void getAbsPos(Ino::Vec3& apos) const;

  void getAbsSpeed(Ino::Trf3& asp) const;
  void getAbsSpeed(Ino::Vec3& asp) const;

  void getAbsAccel(Ino::Trf3& aacc) const;
  void getAbsAccel(Ino::Vec3& aacc) const;

  const Ino::Trf3& getPos() const { return pos; }
  void setPos(const Ino::Trf3& newPos) { pos = newPos; }
};

//---------------------------------------------------------------------------

typedef ObjList<Probe> ProbeList;

} // namespace

//---------------------------------------------------------------------------
#endif
