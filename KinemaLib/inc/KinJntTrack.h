//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Kinema: Kinematic Simulation Program -------------------------
//---------------------------------------------------------------------------
//---------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 ---------
//---------------------------------------------------- C.Wolters ------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------ Track Joint (4 degrees of freedom) ---------------------------
//------------ (Sends a body along a track) ---------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_JNTTRACK_INC
#define INOKIN_JNTTRACK_INC

#include "KinAbstractJoint.h"
#include "KinAbstractTrack.h"

namespace InoKin {

//---------------------------------------------------------------------------

// varPos[0] -> position at track
// varPos[1] -> camber of wheel (deviation from vertical)
// varPos[2] -> misalignment (angle with track direction)
// varPos[3] -> slideways slide

// Z-Axis = wheel rotation axis
// Y-Axis = travel direction

  class JntTrack : public AbstractJoint
  {
    double rad;

    const AbstractTrack* trk;

    JntTrack(const JntTrack& cp) = delete;           // No copying
    JntTrack& operator=(const JntTrack& src) = delete; // No assignment

  protected:
    virtual AbstractJoint* clone(Grip& newGrip) const;

    virtual void getVarTrf(int idx, Ino::Trf3& trf) const;
    virtual void getVarDerTrf(int idx, Ino::Trf3& trf) const;
    virtual void getVarDer2Trf(int idx, Ino::Trf3& trf) const;
    virtual void getVarDer3Trf(int idx, Ino::Trf3& trf) const;

  public:

    explicit JntTrack(Grip& grp, const wchar_t* name, const AbstractTrack& track, double wheelRad);
    explicit JntTrack(Grip& grp, const JntTrack& cp);
    virtual ~JntTrack() {}

    virtual void initVarsFromPos(bool fixedAlso);

    double getWheelRad() const { return rad; }
    void setWheelRad(double newRad) { rad = newRad; }
  }

  const AbstractTrack& getTrack() const { return *trk; }

  void replaceTrack(const AbstractTrack& newTrk);
};

} // namespace

// Interface Section

extern "C" __declspec(dllexport) void* JointTrackNew(void* cppGrip, const wchar_t* name,
                                                     void* cppTrk, double wheelRad);

//---------------------------------------------------------------------------
#endif
