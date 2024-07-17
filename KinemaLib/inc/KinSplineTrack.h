//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- 3D Spline-based Track Interpolator -----------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- THIS IS UNTESTED CODE! ----------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_SPLINETRACK_INC
#define INOKIN_SPLINETRACK_INC

// ATTN: Untested code!

#include "KinAbstractTrack.h"
#include "Spline3D.h"
#include "Trf.h"

namespace InoKin {

//---------------------------------------------------------------------------

class SplineTrack : public AbstractTrack
{
  Spline3D& spline;

  double trackPipeRadius;

  SplineTrack& operator=(const SplineTrack& src) = delete; // No assignment

public:
  explicit SplineTrack(double trackPipeDiameter = 0.0);
  explicit SplineTrack(const SplineTrack& cp);
  virtual ~SplineTrack();

  virtual void clear();

  bool build(bool closed, int controlSz,
             const Ino::Vec3 *ptLst, int ptLstSz,
             double& rmsDist, double& maxDist);

  // virtual void reverse() = 0;

  const Spline3D& getSpline() const { return spline; }

  virtual bool isClosed() const;

  virtual double getLength() const;
  virtual double getMaxS() const;

  double getPipeRadius() const { return trackPipeRadius; }   // Track pipe radius
  double getPipeDiameter() const { return trackPipeRadius * 2.0; }

  void setPipeRadius(double radius) { trackPipeRadius = radius; }
  void setPipeDiameter(double diameter) { trackPipeRadius = diameter/2.0; }

  virtual Ino::Vec3 calcCentroid() const;
  virtual void translate(const Ino::Vec3& offset);

  virtual void getPoint(double at_s, Ino::Vec3& p) const;
  virtual void getPointAndDir(double at_s, Ino::Vec3& pnt, Ino::Vec3& dir) const;
  virtual void getDir(double at_s, Ino::Vec3& dir) const;
  virtual void getAcc(double at_s, Ino::Vec3& acc) const;
  virtual void getJerk(double at_s, Ino::Vec3& jerk) const;

  virtual void getXDir(double at_s, Ino::Vec3& x, Ino::Vec3& xDer) const;

  virtual double findPoint(const Ino::Vec3& p) const;
};

} // namespace

//---------------------------------------------------------------------------
#endif
