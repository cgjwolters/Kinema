//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Ball Joint (3 degrees (rotations) of freedom ---------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_JNTBALL_INC
#define INOKIN_JNTBALL_INC

#include "KinAbstractJoint.h"

namespace InoKin {

//---------------------------------------------------------------------------

// varPos[0] = rotation about z-axis
// varPos[1] = rotation about y-axis
// varPos[2] = rotation about x-axis

class Grip;

class JntBall : public AbstractJoint
{
  JntBall(const JntBall& cp) = delete;            // No copying
  JntBall& operator=(const JntBall& cp) = delete; // No assignment

protected:
  virtual AbstractJoint *clone(Grip& newGrip) const;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

public:
  explicit JntBall(Grip& grp, const wchar_t *name);
  explicit JntBall(Grip& grp, const JntBall& cp);

  virtual void initVarsFromPos(bool fixedAlso);
};

} // namespace

// Interface Section

extern "C" __declspec(dllexport) void* JointBallNew(void* cppGrip, const wchar_t* name);

//---------------------------------------------------------------------------
#endif
