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

#ifndef INOKIN_SEQUENCE_INC
#define INOKIN_SEQUENCE_INC

#include "KinObject.h"

#include "KinState.h"

#include "Array.h"

namespace InoKin {

//---------------------------------------------------------------------------

typedef Ino::Array<State *> StateList;

class Topology;
class Model;

class Sequence : public StateList
{
  Topology& topology;
  wchar_t *name;

  Sequence& operator=(const Sequence& src) = delete; // No Assignment

  explicit Sequence(Topology& topo, const wchar_t *seqName);
  explicit Sequence(Topology& topo, const Sequence& cp);

public:
  virtual ~Sequence();

  Topology& getTopology() const { return topology; }
  Model *getModel() const;
  const wchar_t *getName() const { return name; }

  void addCurrentTopoState();

  friend class Topology;
};

typedef Ino::Array<Sequence *> SequenceList;

} // namespace

// Interface Section

extern "C" __declspec(dllexport) InoKin::Sequence *SequenceNew(InoKin::Topology& cppTopo);

// End Interface Section

//---------------------------------------------------------------------------
#endif
