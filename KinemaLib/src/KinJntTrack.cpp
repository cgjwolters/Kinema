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

#include "KinJntTrack.h"

#include "Exceptions.h"

#include <cmath>

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

JntTrack::JntTrack(Grip& grp, const wchar_t *name,
                         const AbstractTrack& track, double wheelRad)
: AbstractJoint(grp,name,4), rad(fabs(wheelRad)), trk(&track)
{
  isAngular[0] = false;
  isAngular[1] = true;
  isAngular[2] = true;
  isAngular[3] = false;
}

//---------------------------------------------------------------------------

JntTrack::JntTrack(Grip& grp, const JntTrack& cp)
: AbstractJoint(grp,cp), rad(cp.rad), trk(cp.trk)
{
}

//---------------------------------------------------------------------------

AbstractJoint *JntTrack::clone(Grip& newGrip) const
{
  return new JntTrack(newGrip,*this);
}

//---------------------------------------------------------------------------

static void leftVarTrf(const Vec3& v, Trf3& trf)
{
  trf.init();
  trf.isDerivative = false;

  trf(0,0) = -v.x*v.z; trf(0,1) = v.x; trf(0,2) = -v.y;
  trf(1,0) = -v.y*v.z; trf(1,1) = v.y; trf(1,2) =  v.x;
  trf(2,0) = 1.0 - sqr(v.z);
                       trf(2,1) = v.z; trf(2,2) =  0.0;
}

//---------------------------------------------------------------------------

static void leftDerTrf(const Vec3& v, const Vec3& a, Trf3& trf)
{
  trf.zero();
  trf.isDerivative = true;

  trf(0,0) = -a.x*v.z-v.x*a.z; trf(0,1) = a.x; trf(0,2) = -a.y;
  trf(1,0) = -a.y*v.z-v.y*a.z; trf(1,1) = a.y; trf(1,2) =  a.x;
  trf(2,0) = -2.0*v.z*a.z;     trf(2,1) = a.z; trf(2,2) =  0.0;
}

//---------------------------------------------------------------------------

static void leftAccTrf(const Vec3& v, const Vec3& a, const Vec3 j, Trf3& trf)
{
  trf.zero();
  trf.isDerivative = true;

  trf(0,0) = -j.x*v.z-2.0*a.x*a.z-v.x*j.z; trf(0,1) = j.x; trf(0,2) = -j.y;
  trf(1,0) = -j.y*v.z-2.0*a.y*a.z-v.y*j.z; trf(1,1) = j.y; trf(1,2) =  j.x;
  trf(2,0) = -2.0*(sqr(a.z)+v.z*j.z);      trf(2,1) = j.z; trf(2,2) =  0.0;
}

//---------------------------------------------------------------------------

static void rightVarTrf(const Vec3& v, Trf3& trf)
{
  double len = sqrt(1.0-sqr(v.z));

  trf.init();
  trf.isDerivative = false;

  trf(0,0) = trf(2,2) = 1.0/len;
}

//---------------------------------------------------------------------------

static void rightDerTrf(const Vec3& v, const Vec3& a, Trf3& trf)
{
  double lensq = 1.0-sqr(v.z);
  double len   = sqrt(lensq);

  trf.zero();
  trf.isDerivative = true;

  trf(0,0) = trf(2,2) = v.z*a.z/lensq/len;
}

//---------------------------------------------------------------------------

static void rightAccTrf(const Vec3& v, const Vec3& a, const Vec3& j,
                                                                 Trf3& trf)
{
  double lensq = 1.0-sqr(v.z);
  double len   = sqrt(lensq);

  trf.zero();
  trf.isDerivative = true;

  trf(0,0) = trf(2,2) = (sqr(a.z)+v.z*j.z +
                                   3.0*sqr(v.z)*sqr(a.z)/lensq)/lensq/len;
}

//---------------------------------------------------------------------------

void JntTrack::getVarTrf(int idx, Trf3& trf) const
{
  trf.init();
  trf.isDerivative = false;

  switch (idx) {
  case 0: { // position matrix
      Vec3 p, v;
      trk->getPointAndDir(varPos[0],p,v);

      Vec3 n,nd;
      trk->getXDir(varPos[0],n,nd);

      trf(0,0) = n.x; trf(0,1) = v.x; trf(0,2) = n.y*v.z - n.z*v.y;
      trf(1,0) = n.y; trf(1,1) = v.y; trf(1,2) = n.z*v.x - n.x*v.z;
      trf(2,0) = n.z; trf(2,1) = v.z; trf(2,2) = n.x*v.y - n.y*v.x;

      trf(0,3) = p.x; trf(1,3) = p.y; trf(2,3) = p.z;

      trf.invert();
    }
    break;

  case 1: { // camber angle
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) = ca;  trf(0,1) = 0; trf(0,2) = -sa; trf(0,3) = 0;
      trf(1,0) =  0;  trf(1,1) = 1; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) = sa;  trf(2,1) = 0; trf(2,2) =  ca; trf(2,3) = 0;
    }
    break;

  case 2: { // misalignment
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) = 1; trf(0,1) =   0; trf(0,2) =  0; trf(0,3) = 0;
      trf(1,0) = 0; trf(1,1) =  cb; trf(1,2) = sb; trf(1,3) = 0;
      trf(2,0) = 0; trf(2,1) = -sb; trf(2,2) = cb; trf(2,3) = 0;
    }
    break;

  case 3: { // Wheel radius and sideways slide
      trf(0,0) = 1; trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = -rad-trk->getPipeRadius();
      trf(1,0) = 0; trf(1,1) = 1; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0; trf(2,1) = 0; trf(2,2) = 1; trf(2,3) = -varPos[3];
    }
    break;
  }
}

//---------------------------------------------------------------------------

void JntTrack::getVarDerTrf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  switch (idx) {
  case 0: { // position matrix
      Vec3 v, a;
      trk->getDir(varPos[0],v);
      trk->getAcc(varPos[0],a);

      Vec3 n,nd;
      trk->getXDir(varPos[0],n,nd);

      trf(0,0) = nd.x; trf(0,1) = a.x; trf(0,2) = nd.y*v.z - nd.z*v.y + n.y*a.z - n.z*a.y;
      trf(1,0) = nd.y; trf(1,1) = a.y; trf(1,2) = nd.z*v.x - nd.x*v.z + n.z*a.x - n.x*a.z;
      trf(2,0) = nd.z; trf(2,1) = a.z; trf(2,2) = nd.x*v.y - nd.y*v.x + n.x*a.y - n.y*a.x;

      trf(0,3) = v.x; trf(1,3) = v.y; trf(2,3) = v.z;

      Trf3 lTrf; getVarTrf(idx,lTrf);

      trf *= lTrf;
      trf.preMultWith(lTrf);
      trf *= -1.0;
    }
    break;

  case 1: { // camber angle
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) = -sa;  trf(0,1) = 0; trf(0,2) = -ca; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) = 0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) =  ca;  trf(2,1) = 0; trf(2,2) = -sa; trf(2,3) = 0;
    }
    break;

  case 2: { // misalignment
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) = 0; trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0; trf(1,1) = -sb; trf(1,2) =  cb; trf(1,3) = 0;
      trf(2,0) = 0; trf(2,1) = -cb; trf(2,2) = -sb; trf(2,3) = 0;
    }
    break;

  case 3: { // Wheel radius and sideways slide
      trf(0,0) = 0; trf(0,1) = 0; trf(0,2) = 0; trf(0,3) =  0;
      trf(1,0) = 0; trf(1,1) = 0; trf(1,2) = 0; trf(1,3) =  0;
      trf(2,0) = 0; trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = -1;
    }
    break;
  }
}

//---------------------------------------------------------------------------
// ATTN: this one is not functional!!

void JntTrack::getVarDer2Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  switch (idx) {
  case 0: { // position matrix
//      throw OperationNotSupportedException("JntTrack::getVarDer2Trf");

      Vec3 v, a, j;
      trk->getDir(varPos[0],v);
      trk->getAcc(varPos[0],a);
      trk->getJerk(varPos[0],j);

      Trf3 lTrf,  rTrf;  leftVarTrf(v,lTrf);      rightVarTrf(v,rTrf);
      Trf3 ldTrf, rdTrf; leftDerTrf(v,a,ldTrf);   rightDerTrf(v,a,rdTrf);
      Trf3 laTrf, raTrf; leftAccTrf(v,a,j,laTrf); rightAccTrf(v,a,j,raTrf);

      trf = rTrf; trf.preMultWith(laTrf);
      rdTrf.preMultWith(ldTrf);
      rdTrf *= 2.0;

      trf += rdTrf;

      raTrf.preMultWith(lTrf);

      trf += raTrf;

      trf(0,3) = a.x; trf(1,3) = a.y; trf(2,3) = a.z;

      // Invert:
      getVarTrf(idx,lTrf);
      getVarDerTrf(idx,ldTrf);

      trf *= lTrf;
      trf.preMultWith(lTrf);
      trf *= -1.0;

      lTrf.invert();
      lTrf *= ldTrf;
      lTrf.preMultWith(ldTrf);
      lTrf *= 2.0;

      trf += lTrf;
    }
    break;

  case 1: { // camber angle
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) = -ca;  trf(0,1) = 0; trf(0,2) =  sa; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) = 0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) = -sa;  trf(2,1) = 0; trf(2,2) = -ca; trf(2,3) = 0;
    }
    break;

  case 2: { // misalignment
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) = 0; trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0; trf(1,1) = -cb; trf(1,2) = -sb; trf(1,3) = 0;
      trf(2,0) = 0; trf(2,1) =  sb; trf(2,2) = -cb; trf(2,3) = 0;
    }
    break;

  case 3: { // Wheel radius and sideways slide
      trf(0,0) = 0; trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0; trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0; trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;
  }
}

//---------------------------------------------------------------------------
// ATTN: this one is not functional!!

void JntTrack::getVarDer3Trf(int idx, Trf3& trf) const
{
  trf.zero();
  trf.isDerivative = true;

  switch (idx) {
  case 0: { // position matrix
//      throw OperationNotSupportedException("JntTrack::getVarDer3Trf");

      Vec3 v, a, j;
      trk->getDir(varPos[0],v);
      trk->getAcc(varPos[0],a);
      trk->getJerk(varPos[0],j);

      Trf3 ltrf,  rtrf;  leftVarTrf(v,ltrf);      rightVarTrf(v,rtrf);
      Trf3 ldtrf, rdtrf; leftDerTrf(v,a,ldtrf);   rightDerTrf(v,a,rdtrf);
      Trf3 latrf, ratrf; leftAccTrf(v,a,j,latrf); rightAccTrf(v,a,j,ratrf);

      trf = rtrf; trf.preMultWith(latrf);
      rdtrf.preMultWith(ldtrf);
      rdtrf *= 2.0;

      trf += rdtrf;

      ratrf.preMultWith(ltrf);

      trf += ratrf;

      trf(0,3) = a.x; trf(1,3) = a.y; trf(2,3) = a.z;

      // Invert:
      getVarTrf(idx,ltrf);
      getVarDerTrf(idx,ldtrf);

      trf *= ltrf;
      trf.preMultWith(ltrf);
      trf *= -1.0;

      ltrf.invert();
      ltrf *= ldtrf;
      ltrf.preMultWith(ldtrf);
      ltrf *= 2.0;

      trf += ltrf;
    }
    break;

  case 1: { // camber angle
      double ca = cos(varPos[1]);
      double sa = sin(varPos[1]);

      trf(0,0) =  sa;  trf(0,1) = 0; trf(0,2) =  ca; trf(0,3) = 0;
      trf(1,0) =   0;  trf(1,1) = 0; trf(1,2) =   0; trf(1,3) = 0;
      trf(2,0) = -ca;  trf(2,1) = 0; trf(2,2) =  sa; trf(2,3) = 0;
    }
    break;

  case 2: { // misalignment
      double cb = cos(varPos[2]);
      double sb = sin(varPos[2]);

      trf(0,0) = 0; trf(0,1) =   0; trf(0,2) =   0; trf(0,3) = 0;
      trf(1,0) = 0; trf(1,1) =  sb; trf(1,2) = -cb; trf(1,3) = 0;
      trf(2,0) = 0; trf(2,1) =  cb; trf(2,2) =  sb; trf(2,3) = 0;
    }
    break;

  case 3: { // Wheel radius and sideways slide
      trf(0,0) = 0; trf(0,1) = 0; trf(0,2) = 0; trf(0,3) = 0;
      trf(1,0) = 0; trf(1,1) = 0; trf(1,2) = 0; trf(1,3) = 0;
      trf(2,0) = 0; trf(2,1) = 0; trf(2,2) = 0; trf(2,3) = 0;
    }
    break;
  }
}

//---------------------------------------------------------------------------

void JntTrack::initVarsFromPos(bool fixedAlso)
{
  calcPosFromNeighbours();

  Trf3 trf(getInvPos());

  Vec3 p(0,0,0);
  p.transform3(trf);

  if (fixedAlso || !getFixed(0)) varPos[0] = trk->findPoint(p);

  Vec3 v;
  trk->getPoint(varPos[0],p);
  trk->getDir(varPos[0],v);

  Vec3 n,nd;
  trk->getXDir(varPos[0],n,nd);

  Trf3 rTrf;

  rTrf(0,0) = n.x; rTrf(0,1) = v.x; rTrf(0,2) = n.y*v.z - n.z*v.y;
  rTrf(1,0) = n.y; rTrf(1,1) = v.y; rTrf(1,2) = n.z*v.x - n.x*v.z;
  rTrf(2,0) = n.z; rTrf(2,1) = v.z; rTrf(2,2) = n.x*v.y - n.y*v.x;

  rTrf(0,3) = p.x; rTrf(1,3) = p.y; rTrf(2,3) = p.z;

  trf = getPos();
  trf *= rTrf;

  //-----------------------------

  if (fixedAlso || !getFixed(1)) varPos[1] = atan2(-trf(0,2),trf(0,0));

  if (fixedAlso || !getFixed(2)) varPos[2] = atan2(-trf(2,1),trf(1,1));

  if (fixedAlso || !getFixed(3)) varPos[3] = -trf(2,3);

  setModelModified();
}

//---------------------------------------------------------------------------

void JntTrack::replaceTrack(const AbstractTrack &newTrk)
{
  trk = &newTrk;
}

} // namespace

//---------------------------------------------------------------------------
