//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Kinema: Kinematic Simulation Program -------------------------
//---------------------------------------------------------------------------
//---------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 ---------
//---------------------------------------------------- C.Wolters ------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Persistence Data Definitions ---------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinDataDef.h"

#include "KinObject.h"
#include "KinAbstractJoint.h"
#include "KinAbstractTrack.h"
#include "KinArcLinTrack.h"
#include "KinBody.h"
#include "KinFunction.h"
#include "KinGrip.h"
#include "KinJntAll.h"
#include "KinModel.h"
#include "KinObjList.h"
#include "KinProbe.h"
#include "KinState.h"
#include "KinSequence.h"
#include "KinTopology.h"

#include "PArray.h"
#include "PTrf.h"
#include "PMatrix.h"

namespace InoKin {

using namespace Ino;

//---------------------------------------------------------------------------

static const long ifamMagic = 0x8f3ea1b0;
static const long imagic    = 0x4f3cd7a3;

//---------------------------------------------------------------------------

void KinDataDef::addTypes()
{
  add(new PersistentType<Object>("nl.inofor.kinemalib.Object"));
  add(new PersistentAbstractType<AbstractJoint>("nl.inofor.kinemalib.AbstractJoint"));
  add(new PersistentAbstractType<AbstractTrack>("nl.inofor.kinemalib.AbstractTrack"));
  add(new PersistentType<ArcLinTrackPt>("nl.inofor.kinemalib.ArcLinTrackPt"));
  add(new PersistentType<ArcLinTrack>("nl.inofor.kinemalib.ArcLinTrack"));
  add(new PersistentType<Body>("nl.inofor.kinemalib.Body"));
  add(new PersistentType<PArray<Body *> >("nl.inofor.kinemalib.BodyArray"));
  add(new PersistentType<BodyList>("nl.inofor.kinemalib.BodyList"));
  add(new PersistentAbstractType<Function>("nl.inofor.kinemalib.Function"));
  add(new PersistentType<Ino::PArray<double> >("nl.inofor.kinemalib.DoubleList"));
  add(new PersistentType<FunctionList>("nl.inofor.kinemalib.FunctionList"));
  add(new PersistentType<Grip>("nl.inofor.kinemalib.Grip"));
  add(new PersistentType<PArray<Grip *> >("nl.inofor.kinemalib.GripArray"));
  add(new PersistentType<GripList>("nl.inofor.kinemalib.GripList"));
  add(new PersistentType<LoopList>("nl.inofor.kinemalib.LoopList"));
  add(new PersistentType<JntBall>("nl.inofor.kinemalib.JntBall"));
  add(new PersistentType<JntBallSlide>("nl.inofor.kinemalib.JntBallSlide"));
  add(new PersistentType<JntBall2Slide>("nl.inofor.kinemalib.JntBall2Slide"));
  add(new PersistentType<JntCross>("nl.inofor.kinemalib.JntCross"));
  add(new PersistentType<JntRev>("nl.inofor.kinemalib.JntRev"));
  add(new PersistentType<JntRevSlide>("nl.inofor.kinemalib.JntRevSlide"));
  add(new PersistentType<JntSlide>("nl.inofor.kinemalib.JntSlide"));
  add(new PersistentType<JntTrack>("nl.inofor.kinemalib.JntTrack"));
  add(new PersistentType<Model>("nl.inofor.kinemalib.Model"));
  add(new PersistentType<Probe>("nl.inofor.kinemalib.Probe"));
  add(new PersistentType<ProbeList>("nl.inofor.kinemalib.ProbeList"));
  add(new PersistentType<State>("nl.inofor.kinemalib.State"));
  add(new PersistentType<StateList>("nl.inofor.kinemalib.StateList"));
  add(new PersistentType<Sequence>("nl.inofor.kinemalib.Sequence"));
  add(new PersistentType<SequenceList>("nl.inofor.kinemalib.SequenceList"));
  add(new PersistentType<Topology>("nl.inofor.kinemalib.Topology"));
  add(new PersistentType<PArray<Topology *> >("nl.inofor.kinemalib.TopologyArray"));
  add(new PersistentType<TopologyList>("nl.inofor.kinemalib.TopologyList"));
  add(new PersistentType<PVec3>("nl.inofor.inolib.PVec3"));
  add(new PersistentType<PTrf3>("nl.inofor.inolib.PTrf3"));
  add(new PersistentType<PVector>("nl.inofor.inolib.PVector"));
}

//---------------------------------------------------------------------------

KinDataDef::KinDataDef()
: PersistentTypeDef(ifamMagic, imagic,1,0)
{
  addTypes();
}

} // namespace

//---------------------------------------------------------------------------
