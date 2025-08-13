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
  static void version(char& major, char& minor, char& release);

  explicit Model(const wchar_t *name=nullptr);
  explicit Model(const Model& cp, bool withSequences=false);
  virtual ~Model();

  Model& operator=(const Model& src);

  void clear();

  const wchar_t *getName() const { return mdlName; }
  void setName(const wchar_t *newName);

  void setFixedAll(bool fixd);

  void setTopoModified();
  void setModified(bool mod = true);

  bool isModified() const { return modified; }

  const BodyList& getBodyList() const { return bodyLst; }
  const GripList& getGripList() const { return gripLst; }
  const FunctionList& getFunctionList() const { return funcLst; }
  const TopologyList& getTopologyList() const { return topoLst; }

  const Ino::Vec3& getOffset() const { return offset; }
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

// Interface Section

extern "C" __declspec(dllexport) void* ModelNew(const wchar_t* name);

extern "C" __declspec(dllexport) void GetVersionModel(void *cppModel, char *major, char *minor, char *release);

extern "C" __declspec(dllexport) void SetFixedAllModel(void *cppModel, bool fixd);

extern "C" __declspec(dllexport) void ClearModel(void* cppModel);

extern "C" __declspec(dllexport) const wchar_t* GetNameModel(void* cppModel);
extern "C" __declspec(dllexport) void SetNameModel(void* cppModel, const wchar_t* newName);

extern "C" __declspec(dllexport) void SetTopoModifiedModel(void* cppModel);
extern "C" __declspec(dllexport) void SetModifiedModel(void* cppModel, bool mod = true);

extern "C" __declspec(dllexport) bool IsModifiedModel(void* cppModel);

extern "C" __declspec(dllexport) void GetOffsetModel(void* cppModel, Ino::Vec3& modelOffset);
extern "C" __declspec(dllexport) void SetOffsetModel(void* cppModel, const Ino::Vec3& modelOffset);
extern "C" __declspec(dllexport) void ApplyOffsetModel(void* cppModel);
extern "C" __declspec(dllexport) void TransformModel(void* cppModel, const Ino::Trf3& trf);
extern "C" __declspec(dllexport) bool BuildTopologyModel(void* cppModel);
extern "C" __declspec(dllexport) int  GetTopologySizeModel(void* cppModel);
extern "C" __declspec(dllexport) void *GetTopologyModel(void* cppModel, int index);

// End Interface Section

//---------------------------------------------------------------------------
#endif
