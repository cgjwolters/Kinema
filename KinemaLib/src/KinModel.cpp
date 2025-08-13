//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Main Kinematic Model -------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinModel.h"

#include "KinBody.h"
#include "KinGrip.h"
#include "KinAbstractJoint.h"
#include "KinFunction.h"
#include "KinTopology.h"

#include "Trf.h"
#include "Exceptions.h"

//---------------------------------------------------------------------------

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

void Model::version(char& major, char& minor, char& release)
{
  major   = 0;
  minor   = 0;
  release = 1;
}

//---------------------------------------------------------------------------

Model::Model(const wchar_t *name)
: mdlName(dupStr(name)),
  modified(true),
  offset(),
  bodyLst(*new BodyList()),
  gripLst(*new GripList()),
  funcLst(* new FunctionList(true)),
  topoLst(*new TopologyList())
{
  bodyLst.setObjectOwner(true);
  gripLst.setObjectOwner(true);
  topoLst.setModel(*this);
}

//---------------------------------------------------------------------------

Model::Model(const Model& cp, bool withSequences)
: mdlName(NULL),
  modified(true),
  offset(cp.offset),
  bodyLst(*new BodyList()),
  gripLst(*new GripList()),
  funcLst(* new FunctionList(true)),
  topoLst(*new TopologyList())
{
  bodyLst.setObjectOwner(true);
  gripLst.setObjectOwner(true);
  topoLst.setModel(*this);

  cloneFrom(cp,withSequences);
}

//---------------------------------------------------------------------------

Model::~Model()
{
  delete &funcLst;

  gripLst.setObjectOwner(false);
  
  int sz = gripLst.size();
  for (int i=sz-1; i>=0; --i) delete gripLst[i];

  bodyLst.setObjectOwner(false);
  
  sz = bodyLst.size();
  for (int i=sz-1; i>=0; --i) delete bodyLst[i];
  
  delete &gripLst;
  delete &bodyLst;

  delete[] mdlName;

  delete &topoLst;
}

//---------------------------------------------------------------------------

Model& Model::operator=(const Model& src)
{
  delete[] mdlName;
  mdlName = NULL;

  cloneFrom(src,false);

  return *this;
}

//---------------------------------------------------------------------------

void Model::clear()
{
  delete[] mdlName;
  mdlName = NULL;
  
  modified = true;

  topoLst.clear();
  funcLst.clear();
  gripLst.clear();
  bodyLst.clear();
}

//---------------------------------------------------------------------------

void Model::cloneFrom(const Model& src, bool withSequences)
{
  clear();

  mdlName  = dupStr(src.mdlName);
  modified = true;

  src.bodyLst.setIds();
  src.gripLst.setIds();

  int bSz = src.bodyLst.size();
  bodyLst.ensureCapacity(bSz);
  for (int i=0; i<bSz; ++i) bodyLst.add(new Body(*this,*src.bodyLst[i]));

  int gSz = src.gripLst.size();
  gripLst.ensureCapacity(gSz);
  for (int i=0; i<gSz; ++i) gripLst.add(new Grip(*this,*src.gripLst[i]));

  for (int i=0; i<bSz; ++i) {
    Body *dstBody = bodyLst[i];

    const Body *srcBody = src.bodyLst[i];
    int sz = srcBody->gripLst.size();

    for (int j=0; j<sz ;++j) {
      dstBody->gripLst.add(gripLst.byId(srcBody->gripLst[j]->getId()));
    }
  }

  for (int i=0; i<gSz; ++i) {
    Grip *dstGrp = gripLst[i];
    const Grip *srcGrp = src.gripLst[i];
   
    const Body *srcBody = srcGrp->body1;
    if (srcBody) dstGrp->body1 = bodyLst.byId(srcBody->getId());

    srcBody = srcGrp->body2;
    if (srcBody) dstGrp->body2 = bodyLst.byId(srcBody->getId());
  }

  int tSz = src.topoLst.size();

  for (int i=0; i<tSz; ++i) {
    if (src.topoLst[i])
      topoLst.add(new Topology(*this,*src.topoLst[i],withSequences));
  }

  // Todo: Function List

}

//---------------------------------------------------------------------------

void Model::setName(const wchar_t *newName)
{
  if (!compareStr(mdlName,newName)) return;

  delete[] mdlName;
  mdlName = dupStr(newName);

  modified = true;
}

//---------------------------------------------------------------------------

void Model::setFixedAll(bool fixd)
{
  int sz = gripLst.size();

  for (int i = 0; i < sz; ++i) {
    AbstractJoint* jnt = gripLst[i]->getJoint();

    int varSz = jnt->getVarCnt();

    for (int j = 0; j < varSz; ++j) jnt->setFixed(j, fixd);
  }
}

//---------------------------------------------------------------------------

void Model::setTopoModified()
{
  modified = true;
  topoLst.clear();
}

//---------------------------------------------------------------------------

void Model::setModified(bool mod)
{
  modified = mod;
}

//---------------------------------------------------------------------------

void Model::adopt(Body *body)
{
  if (!body) throw NullPointerException("Model::adopt(Body)");

  setTopoModified();

  bodyLst.add(body);
}

//---------------------------------------------------------------------------

void Model::remove(Body *body)
{
  if (!body) throw NullPointerException("Model::remove(Body)");

  setTopoModified();

  bodyLst.remove(body);
}

//---------------------------------------------------------------------------

void Model::adopt(Grip *grip)
{
  if (!grip) throw NullPointerException("Model::adopt(Grip)");

  setTopoModified();

  gripLst.add(grip);
}

//---------------------------------------------------------------------------

void Model::remove(Grip *grip)
{
  if (!grip) throw NullPointerException("Model::remove(Grip)");

  setTopoModified();

  gripLst.remove(grip);
}

//---------------------------------------------------------------------------

void Model::adopt(Function *fnc)
{
  if (!fnc) throw NullPointerException("Model::adopt(Function)");

  setTopoModified();

  funcLst.add(fnc);
}

//---------------------------------------------------------------------------

void Model::remove(Function *fnc)
{
  if (!fnc) throw NullPointerException("Model::remove(Function)");

  setTopoModified();

  funcLst.remove(fnc);
}

//---------------------------------------------------------------------------

int Model::find(const Body *body) const
{
  return bodyLst.find(body);
}

//---------------------------------------------------------------------------

void Model::setOffset(const Vec3& modelOffset) // Remember offset to be applied
{
  offset = modelOffset;
}

//---------------------------------------------------------------------------

void Model::applyOffset()
{
  Trf3 trf(offset,Vec3(0,0,1),Vec3(1,0,0));
  trf.invert();

  transform(trf);
}

void Model::transform(const Ino::Trf3& trf)
{
  int sz = bodyLst.size();

  for (int i=1; i<sz; ++i) bodyLst[i]->transform(trf);
}

//---------------------------------------------------------------------------

bool Model::buildTopology()
{
  topoLst.clear();

  return topoLst.prepareAll();
}

} // namespace

//---------------------------------------------------------------------------
// Interface Section

void* ModelNew(const wchar_t* name)
{
  //fprintf(stdout, "Cpp name: %ls\n", name);
  //fflush(stdout);

  InoKin::Model* mdl = new InoKin::Model(name);

  return mdl;
}

void GetVersionModel(void *cppModel, char *major, char *minor, char *release)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->version(*major, *minor, *release);
}

void SetFixedAllModel(void* cppModel, bool fixd)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->setFixedAll(fixd);
}


void ClearModel(void* cppModel)
{
  InoKin::Model* mdl = (InoKin::Model *)cppModel;
  mdl->clear();
}

const wchar_t* GetNameModel(void* cppModel)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  return mdl->getName();
}

void SetNameModel(void* cppModel, const wchar_t* newName)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->setName(newName);
}

void SetTopoModifiedModel(void* cppModel)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->setTopoModified();
}

void SetModifiedModel(void* cppModel, bool mod)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->setModified(mod);

}

bool IsModifiedModel(void* cppModel)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  return mdl->isModified();
}

void GetOffsetModel(void* cppModel, Ino::Vec3& modelOffset)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  modelOffset = mdl->getOffset();
}

void SetOffsetModel(void* cppModel, const Ino::Vec3& modelOffset)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->setOffset(modelOffset);
}

void ApplyOffsetModel(void* cppModel)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->applyOffset();
}

void TransformModel(void* cppModel, const Ino::Trf3& trf)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  mdl->transform(trf);
}

bool BuildTopologyModel(void* cppModel)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  return mdl->buildTopology();
}

int  GetTopologySizeModel(void* cppModel)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  return mdl->getTopologyList().size();
}

void* GetTopologyModel(void* cppModel, int index)
{
  InoKin::Model* mdl = (InoKin::Model*)cppModel;

  if (index < 0 || index >= mdl->getTopologyList().size()) return nullptr;

  return mdl->getTopologyList()[index];
}


// End Interface Section
//---------------------------------------------------------------------------
