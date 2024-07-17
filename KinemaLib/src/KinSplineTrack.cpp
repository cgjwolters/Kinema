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

// ATTN: Untested code!

#include "KinSplineTrack.h"

namespace InoKin {

using namespace Ino;

//---------------------------------------------------------------------------

SplineTrack::SplineTrack(double trackPipeDiameter)
: AbstractTrack(), spline(*new Spline3D()),
  trackPipeRadius(trackPipeDiameter/2.0)
{
}

//---------------------------------------------------------------------------

SplineTrack::SplineTrack(const SplineTrack& cp)
: AbstractTrack(cp), spline(*new Spline3D(cp.spline)),
  trackPipeRadius(cp.trackPipeRadius)
{
}

//---------------------------------------------------------------------------

SplineTrack::~SplineTrack()
{
  delete &spline;
}

//---------------------------------------------------------------------------

void SplineTrack::clear()
{
  spline.clear();
}

//---------------------------------------------------------------------------

bool SplineTrack::build(bool closed, int controlSz,
                        const Vec3 *ptLst, int ptLstSz,
                        double& rmsDist, double& maxDist)
{
  return spline.build(4,closed,controlSz,0.0,Spline3D::BuildSimple,
                                             ptLst,ptLstSz,rmsDist,maxDist);
}

//---------------------------------------------------------------------------

bool SplineTrack::isClosed() const
{
  return spline.isClosed();
}

//---------------------------------------------------------------------------

double SplineTrack::getLength() const
{
  return spline.maxU() - spline.minU();
}

//---------------------------------------------------------------------------

double SplineTrack::getMaxS() const
{
  return spline.maxU();
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// TODO: Implement SplineTrack::calcCentroid()

Ino::Vec3 SplineTrack::calcCentroid() const
{
  throw OperationNotSupportedException("To be implemented");
}

//---------------------------------------------------------------------------

void SplineTrack::translate(const Vec3& offset)
{
  Trf3 trf(offset,Vec3(0,0,1),Vec3(1,0,0));
  trf.invert();

  spline.transform(trf);
}

//---------------------------------------------------------------------------

void SplineTrack::getPoint(double at_s, Vec3& p) const
{
  if (!spline.point(at_s, p)) p = Vec3();
}

//---------------------------------------------------------------------------

void SplineTrack::getPointAndDir(double at_s, Vec3& pnt, Vec3& dir) const
{
  if (!spline.point(at_s, pnt)) pnt = Vec3();

  if (!spline.derivativeAt(1,at_s,dir)) {
    dir = Vec3();
    dir.isDerivative = true;
  }
}

//---------------------------------------------------------------------------

void SplineTrack::getDir(double at_s, Vec3& dir) const
{
  if (!spline.derivativeAt(1,at_s,dir)) {
    dir = Vec3();
    dir.isDerivative = true;
  }
}

//---------------------------------------------------------------------------

void SplineTrack::getAcc(double at_s, Vec3& acc) const
{
  if (!spline.derivativeAt(2,at_s,acc)) {
    acc = Vec3();
    acc.isDerivative = true;
  }
}

//---------------------------------------------------------------------------

void SplineTrack::getJerk(double at_s, Vec3& jerk) const
{
  if (!spline.derivativeAt(3,at_s,jerk)) {
    jerk = Vec3();
    jerk.isDerivative = true;
  }
}

//---------------------------------------------------------------------------

void SplineTrack::getXDir(double at_s, Vec3& x, Vec3& xDer) const
{
}

//---------------------------------------------------------------------------

double SplineTrack::findPoint(const Vec3& p) const
{
  return 0.0;
}

} // namespace

//---------------------------------------------------------------------------
