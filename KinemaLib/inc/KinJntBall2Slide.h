//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Ball Joint + 2 slides (5 degrees of freedom) ---------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_JNTBALL2SLIDE_INC
#define INOKIN_JNTBALL2SLIDE_INC

#include "KinAbstractJoint.h"

namespace InoKin {

//---------------------------------------------------------------------------

// varPos[0] = slide along first z-axis
// varPos[1] = rotation about z-axis
// varPos[2] = rotation about y-axis
// varPos[3] = rotation about x-axis
// varPos[4] = slide along (transformed) z-axis

// ATTN: Joint is unstable if all rotations are near zero!

class JntBall2Slide : public AbstractJoint
{
  JntBall2Slide(const JntBall2Slide& cp) = delete;            // No copying
  JntBall2Slide& operator=(const JntBall2Slide& cp) = delete; // No assignment

protected:
  virtual AbstractJoint *clone(Grip& newGrip) const;

  virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
  virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

public:
  explicit JntBall2Slide(Grip& grp, const wchar_t *name);
  explicit JntBall2Slide(Grip& grp, const JntBall2Slide& cp);

  virtual void initVarsFromPos(bool fixedAlso);
};

} // namespace

// Interface Section

extern "C" __declspec(dllexport) void* JointBall2SlideNew(void* cppGrip, const wchar_t* name);

//---------------------------------------------------------------------------
#endif
