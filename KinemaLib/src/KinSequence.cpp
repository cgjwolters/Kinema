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

} // namespace

// Interface Section

void* SequenceNew(void *cppTopo, const wchar_t *name)
{
  InoKin::Topology& topo = *(InoKin::Topology*) cppTopo;

  return new InoKin::Sequence(topo, name);
}

void AddCurrentTopoStateSequence(InoKin::Sequence *cppSeq)
{
  InoKin::Sequence& seq = *(InoKin::Sequence*)cppSeq;

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

// End Interface Section

//---------------------------------------------------------------------------
