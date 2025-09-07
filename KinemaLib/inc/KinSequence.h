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

  explicit Sequence(Topology& topo, const Sequence& cp);

public:
  explicit Sequence(Topology& topo, const wchar_t* seqName);
  virtual ~Sequence();

  Topology& getTopology() const { return topology; }
  Model *getModel() const;
  const wchar_t *getName() const { return name; }

  void addCurrentTopoState();

  void writeSequence();

  friend class Topology;
};

typedef Ino::Array<Sequence *> SequenceList;

} // namespace

// Interface Section

extern "C" __declspec(dllexport) void* SequenceNew(void* cppTopo, const wchar_t* name);

extern "C" __declspec(dllexport) void AddCurrentTopoStateSequence(void *cppSequence);

extern "C" __declspec(dllexport) void GetStateCountSequence(void *cppSequence, int& count);

extern "C" __declspec(dllexport) InoKin::State* GetStateSequence(void* cppSequence, int index);

//extern "C" __declspec(dllexport) void WriteStateSequence(void* cppSequence);

extern "C" __declspec(dllexport) void WriteSeqSequence(void* cppSeq);

// End Interface Section

//---------------------------------------------------------------------------
#endif
