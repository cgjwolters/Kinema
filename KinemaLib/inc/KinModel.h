//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Main Kinematic Model -------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_MODEL_INC
#define INOKIN_MODEL_INC

#include "Vec.h"

namespace Ino {
  class Trf3;
}

namespace InoKin {

//---------------------------------------------------------------------------

class Body;
class BodyList;
class Grip;
class GripList;
class Function;
class FunctionList;
class TopologyList;

class Model
{
  wchar_t *mdlName;
  bool modified;

  Ino::Vec3 offset;

  BodyList& bodyLst;
  GripList& gripLst;
  FunctionList& funcLst;

  TopologyList& topoLst;

  int find(const Body *body) const;

  void adopt(Body *body);
  void remove(Body *body);

  void adopt(Grip *grip);
  void remove(Grip *grip);

  void adopt(Function *fnc);
  void remove(Function *fnc);

  void cloneFrom(const Model& mdl, bool withSequences);

public:
  static void version(unsigned char& major, unsigned char& minor,
                                               unsigned char& release);

  explicit Model(const wchar_t *name=NULL);
  explicit Model(const Model& cp, bool withSequences=false);
  virtual ~Model();

  Model& operator=(const Model& src);

  void clear();

  const wchar_t *getName() const { return mdlName; }
  void setName(const wchar_t *newName);

  void setTopoModified();
  void setModified(bool mod = true);

  bool isModified() const { return modified; }

  const BodyList& getBodyList() const { return bodyLst; }
  const GripList& getGripList() const { return gripLst; }
  const FunctionList& getFunctionList() const { return funcLst; }
  const TopologyList& getTopologyList() const { return topoLst; }

  void setOffset(const Ino::Vec3& modelOffset); // Remember offset to be applied
  void applyOffset();
  void transform(const Ino::Trf3& trf);

  bool buildTopology();

  friend class Body;
  friend class Grip;
  friend class Function;
  friend class Topology;
  friend class TopologyList;
};

} // namespace

//---------------------------------------------------------------------------
#endif
