//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Kinema: Kinematic Simulation Program -------------------------
//---------------------------------------------------------------------------
//---------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 ---------
//---------------------------------------------------- C.Wolters ------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//--- Storage for sequence of model states (position/speeds/accels) ---------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinSequence.h"

#include "KinModel.h"
#include "KinGrip.h"
#include "KinAbstractJoint.h"
#include "KinTopology.h"
#include "KinState.h"

#include "Vec.h"

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

Sequence::Sequence(Topology& topo, const wchar_t *seqName)
: StateList(true,1024),
  topology(topo),
  name(dupStr(seqName))
{
}

//---------------------------------------------------------------------------

Sequence::Sequence(Topology& topo, const Sequence& cp)
: StateList(true,cp.size()),
  topology(topo),
  name(dupStr(cp.name))
{
  int sz = cp.size();
  
  for (int i=0; i<sz; ++i) add(new State(*this,*cp[i]));
}

//---------------------------------------------------------------------------

Sequence::~Sequence()
{
  delete[] name;

  // remove self from topology!!
}

//---------------------------------------------------------------------------

Model *Sequence::getModel() const
{
  return topology.getModel();
}

//---------------------------------------------------------------------------

void Sequence::addCurrentTopoState()
{
  add(new State(*this, size(), 0L, topology));
}

bool writeState(FILE* fd, const InoKin::State* st)
{
  int sz = st->getVarPosSize();

  for (int i = 0; i < sz; ++i) {
    double v = st->getVarPos(i);
    fprintf(fd, "%.7f;", v);
  }

  fprintf(fd, "\n");

  return true;
}

void Sequence::writeSequence()
{
  FILE* fd = fopen("C:\\Users\\Clemens\\Documents\\temp\\StateList.csv","w");

  const GripList& lst = getModel()->getGripList();

  int jntSz = lst.size();

  for (int i = 0; i < jntSz; ++i) {
    auto jnt = lst[i]->getJoint();
    int varCnt = jnt->getVarCnt();

    for (int j = 0; j < varCnt; ++j) {
      if (jnt->getFixed(j)) continue;
      fwprintf(fd, L"%ls-%d;", jnt->getName(), j);
    }
  }

  fwprintf(fd, L"\n");

  for (int ii = 0; ii < size(); ++ii) {
    State *st = get(ii);

    for (int i = 0; i < jntSz; ++i) {
      auto jnt = lst[i]->getJoint();
      int varCnt = jnt->getVarCnt();

      for (int j = 0; j < varCnt; ++j) {
        if (jnt->getFixed(j)) continue;

        double posVal;
        st->getPos(*jnt, j, posVal);

        if (jnt->getIsAngular(j)) posVal *= 180.0/Vec2::Pi;

        fwprintf(fd, L"%.7f;", posVal);
      }
    }

    fwprintf(fd, L"\n");
  }


  //for (int i = 0; i < sz; ++i) {
  //  writeState(fd, get(i));
  //}

  fclose(fd);
}

} // namespace

// Interface Section

void* SequenceNew(void *cppTopo, const wchar_t *name)
{
  InoKin::Topology& topo = *(InoKin::Topology*) cppTopo;

  return new InoKin::Sequence(topo, name);
}

void AddCurrentTopoStateSequence(void *cppSequence)
{
  InoKin::Sequence& seq = *(InoKin::Sequence*)cppSequence;

  seq.addCurrentTopoState();
}

void GetStateCountSequence(void* cppSequence, int& count)
{
  InoKin::Sequence& seq = *(InoKin::Sequence*)cppSequence;

  count = seq.size();
}

InoKin::State* GetStateSequence(void* cppSeq, int index)
{
  InoKin::Sequence* seq = (InoKin::Sequence*)cppSeq;

  return seq->get(index);
}

//void WriteStateSequence(void* cppSequence)
//{
//  InoKin::Sequence* seq = (InoKin::Sequence*)cppSequence;
//
//  seq->writeSequence();
//}

void WriteSeqSequence(void* cppSeq) {
  InoKin::Sequence* seq = (InoKin::Sequence*)cppSeq;

  seq->writeSequence();
}

// End Interface Section

//---------------------------------------------------------------------------
