//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Kinema: Kinematic Simulation Program -----------------------
//---------------------------------------------------------------------------
//------------------------ Copyright Inofor Hoek Aut BV Dec 1999 ------------
//------------------------------------------------------ C.Wolters ----------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- 3D arc-linear ArcLinTrack Interpolator -------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinArcLinTrack.h"

#include "Exceptions.h"

// #include <cmath>

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

ArcLinTrackPt::ArcLinTrackPt(const Vec3& v)
: Vec3(v), prv(NULL), nxt(NULL), isArc(false),
  s(0.0), minS(0.0), maxS(0.0), rad(0.0),
  start(0.0), end(0.0), norm(), center(),
  zDir(*new Vec3()), ltrf()
{
}

//---------------------------------------------------------------------------

ArcLinTrackPt::~ArcLinTrackPt()
{
  delete &zDir;
}

//---------------------------------------------------------------------------

void ArcLinTrackPt::setPoint(const Vec3& v)
{
  Vec3::operator=(v);
}

//---------------------------------------------------------------------------

static void calcStartEnd(const Vec3& norm, const Vec3& center,
                         const Vec3& spt, const Vec3& ept,
                         bool ccw,
                         double& start, double& end)
{
  double div = 1.0/64.0;

  Vec3 Wy(0,1,0);
  Vec3 Wz(0,0,1);

  Vec3 Xa;

  if (fabs(norm.x) < div && fabs(norm.y) < div) Xa = Wy.outer(norm);
  else  Xa = Wz.outer(norm);

  Xa.unitLen3();

  Trf3 trf(center,norm,Xa);

  Vec3 lspt(spt), lept(ept);

  lspt.transform3(trf); start = atan2(lspt.y,lspt.x);
  lept.transform3(trf); end   = atan2(lept.y,lept.x);

  if (start < 0.0) start += Vec2::Pi2;
  if (end   < 0.0) end   += Vec2::Pi2;

  if (ccw) {
    if (end < start) end += Vec2::Pi2;
  }
  else if (end > start) end -= Vec2::Pi2;
}

//---------------------------------------------------------------------------

void ArcLinTrackPt::calcArc()
{
  s = 0;

  Vec3 v1 = *this; v1 -= *prv;
  Vec3 v2 = *nxt; v2 -= *this;

  norm = v1.outer(v2);

  if (norm.len3() < 1e-10) { // Straight segment
    isArc = false;

    maxS =  v2.len3();
    minS = -v1.len3();

    double div = 1.0/64.0;

    Vec3 Wy(0,1,0), Wz(0,0,1), Ya;
    Vec3 Xa(v2); Xa -= v1; Xa.unitLen3();

    if (fabs(Xa.x) < div && fabs(Xa.y) < div) norm = Xa.outer(Wy);
    else {
      Ya = Wz.outer(Xa); Ya.unitLen3();
      norm = Xa.outer(Ya);
    }

    norm.unitLen3();

    Trf3 trf(*this,norm,Xa);
    trf.invertInto(ltrf);
  }
  else {
    norm.unitLen3();

    isArc = true;

    Vec3 xdir = v1; xdir.unitLen3();

    Trf3 trf(*this,norm,xdir);

    Vec3 p1(*nxt); p1.transform3(trf); p1 /= 2.0;

    double a = (v1.len3()/2.0+p1.x)/p1.y;

    center = p1; center.x -= a*p1.y; center.y += a*p1.x;
    rad = center.len2();

    Vec3 sv(*prv); sv -= center; 

    trf.invert();
    center.transform3(trf);

    xdir = *this; xdir -= center; xdir.unitLen3();

    trf = Trf3(*this,norm,xdir);

    v1 = *prv; v1.transform3(trf); v1.x += rad; minS = atan2(v1.y,v1.x);
    v1 = *nxt; v1.transform3(trf); v1.x += rad; maxS = atan2(v1.y,v1.x);

    minS *= rad; maxS *= rad;

    trf.invertInto(ltrf);

    calcStartEnd(norm, center, *prv, *nxt, minS < 0.0, start, end);
  }
}

//---------------------------------------------------------------------------

void ArcLinTrackPt::getPoint(double atRel, Vec3& p) const
{
  p.isDerivative = false;

  if (!isArc) {
    if (atRel < 0.0) {
      p = *this * (1.0+atRel) - *prv * atRel;
    }
    else {
      p = *this * (1.0-atRel) + *nxt * atRel;
    }
  }
  else {
    if (atRel < 0.0) atRel *= -minS;
    else              atRel *=  maxS;

    atRel /= rad;

    p.x = rad * (cos(atRel) - 1.0); p.y = rad * sin(atRel);

    p.transform3(ltrf);
  }
}

//---------------------------------------------------------------------------

void ArcLinTrackPt::getDir(double atRel, Vec3& v) const
{
  v.isDerivative = true;

  if (!isArc) {
    if (atRel < 0.0) {
      v = *this - *prv;
    }
    else {
      v = *nxt - *this;
    }
  }
  else {
    if (atRel < 0.0) {
      atRel *= -minS; atRel /= rad;

      v.x =  minS * sin(atRel);
      v.y = -minS * cos(atRel);
    }
    else {
      atRel *= maxS; atRel /= rad;

      v.x = -maxS * sin(atRel);
      v.y =  maxS * cos(atRel);
    }

    v.transform3(ltrf);
  }
}

//---------------------------------------------------------------------------

void ArcLinTrackPt::getAcc(double atRel, Vec3& a) const
{
  a.isDerivative = true;

  if (!isArc) {
    a.x = 0.0;
    a.y = 0.0;
  }
  else {
    if (atRel < 0.0) {
      atRel *= -minS; atRel /= rad;

      a.x = -minS * minS * cos(atRel)/rad;
      a.y = -minS * minS * sin(atRel)/rad;
    }
    else {
      atRel *= maxS; atRel /= rad;

      a.x = -maxS * maxS * cos(atRel)/rad;
      a.y = -maxS * maxS * sin(atRel)/rad;
    }

    a.transform3(ltrf);
  }
}

//---------------------------------------------------------------------------

void ArcLinTrackPt::getJerk(double atRel, Vec3& j) const
{
  if (!isArc) {
    j.x = 0.0;
    j.y = 0.0;
  }
  else {
    if (atRel < 0.0) {
      atRel *= -minS; atRel /= rad;

      j.x = -minS * minS * minS * sin(atRel)/rad/rad;
      j.y =  minS * minS * minS * cos(atRel)/rad/rad;
    }
    else {
      atRel *= maxS; atRel /= rad;

      j.x =  maxS * maxS * maxS * sin(atRel)/rad/rad;
      j.y = -maxS * maxS * maxS * cos(atRel)/rad/rad;
    }

    j.isDerivative = true;
    j.transform3(ltrf);
  }
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

ArcLinTrack::ArcLinTrack(bool trkClosed, double trackPipeDiameter)
: AbstractTrack(), trk(true), closed(trkClosed),
  trackPipeRadius(trackPipeDiameter/2.0), length(0.0)
{
}

//---------------------------------------------------------------------------

ArcLinTrack::ArcLinTrack(const ArcLinTrack& cp)
: AbstractTrack(cp), trk(cp.trk.isObjectOwner()), closed(cp.closed),
  trackPipeRadius(cp.trackPipeRadius), length(cp.length)
{
  int sz = cp.trk.size();

  for (int i=0; i<sz; ++i) trk.add(new ArcLinTrackPt(cp.trk[i]->getPoint()));
  trk.setObjectOwner(cp.trk.isObjectOwner());

  setRelations();
  validate(); // Also sets length
}

//---------------------------------------------------------------------------


ArcLinTrack::~ArcLinTrack()
{
  clear();
}

//---------------------------------------------------------------------------

void ArcLinTrack::clear()
{
  trk.clear();
  closed = false;
}

//---------------------------------------------------------------------------

const Vec3& ArcLinTrack::getPoint(int idx) const
{
  if (idx < 0 || idx >= trk.size())
    throw IndexOutOfBoundsException("ArcLinTrack::getPoint");

  return trk[idx]->getPoint();
}

int ArcLinTrack::addPoint(const Vec3& pt)
{
  return trk.add(new ArcLinTrackPt(pt));
}

//---------------------------------------------------------------------------

const ArcLinTrackPt& ArcLinTrack::getTrackPt(int idx) const
{
  if (idx < 0 || idx >= trk.size())
    throw IndexOutOfBoundsException("ArcLinTrack::getTrackPt");

  if (!trk[idx]) throw NullPointerException("ArcLinTrack::getTrackPt");

  return *trk[idx];
}

//---------------------------------------------------------------------------

Vec3 ArcLinTrack::calcCentroid() const
{
  Vec3 centroid;

  for (int i=0; i<trk.size(); ++i) {
    centroid += *trk.get(i);
  }

  centroid /= trk.size();

  return centroid;
}

//---------------------------------------------------------------------------

void ArcLinTrack::translate(const Vec3& offset)
{
  for (int i=0; i<trk.size(); ++i) {
    *trk.get(i) += offset;
  }

  validate();
}

//---------------------------------------------------------------------------

void ArcLinTrack::setClosed(bool clsd)
{
  if (clsd == closed) return;

  closed = clsd;

  validate();
}

//---------------------------------------------------------------------------

bool ArcLinTrack::setTrack(const Vec3 *ptLst, int ptSz, bool trackClosed,
                           double trackPipeDiameter)
{
  clear();

  closed = trackClosed;
  trackPipeRadius = trackPipeDiameter/2.0;

  if (!ptLst) throw NullPointerException("ArcLinTrack::setTrack");
  if (ptSz < 0) throw IllegalArgumentException("ArcLinTrack::setTrack");

  trk.clear();
  trk.ensureCapacity(ptSz);

  for (int i = 0; i < ptSz; ++i) trk.add(new ArcLinTrackPt(ptLst[i]));

  setRelations();
  validate();  // Also sets length

  return true;
}

//---------------------------------------------------------------------------

bool ArcLinTrack::setTrack(double *xPtLst, double *yPtList, double *zPtList, int ptSz,
                           bool trackClosed, double trackPipeDiameter)
{
  clear();

  closed = trackClosed;
  trackPipeRadius = trackPipeDiameter / 2.0;

  if (!xPtLst || !yPtList || !zPtList) throw NullPointerException("ArcLinTrack::setTrack2");
  if (ptSz < 0) throw IllegalArgumentException("ArcLinTrack::setTrack");

  trk.clear();
  trk.ensureCapacity(ptSz);

  for (int i = 0; i < ptSz; ++i) trk.add(new ArcLinTrackPt(Vec3(xPtLst[i], yPtList[i], zPtList[i])));

  setRelations();
  validate();  // Also sets length

  return true;
}

//---------------------------------------------------------------------------

void ArcLinTrack::setRelations()
{
  int sz = trk.size();

  trk[0]->prv = trk[sz-1];
  for (int i=1; i<sz; ++i) trk[i]->prv = trk[i-1];

  trk[sz-1]->nxt = trk[0];
  for (int i=0; i<sz-1; ++i) trk[i]->nxt = trk[i+1];
}

//---------------------------------------------------------------------------

void ArcLinTrack::validate()
{
  int sz = trk.size();

  for (int i=0; i<sz; i++) trk[i]->calcArc();

  for (int i=1; i<sz; i++) {
    ArcLinTrackPt& pt = *trk[i];

    double ds = (fabs(pt.prv->maxS) + fabs(pt.minS))/2.0;

    pt.s = pt.prv->s + ds;
  }

  length = trk[sz-1]->s;
  
  if (closed) length += (fabs(trk[sz-1]->maxS +
                                               fabs(trk[0]->minS))/2.0);
}

//---------------------------------------------------------------------------

int ArcLinTrack::findUpper(double at_s) const
{
   int lwb = 0, upb = trk.size()-1;

   while (lwb <= upb) {
     int cur = (lwb + upb)/2;

     if (trk[cur]->s <= at_s) lwb = cur+1;
     else                        upb = cur-1;
   }

   return lwb;
}

//---------------------------------------------------------------------------

void ArcLinTrack::findPointPair(double at_s, int& lowidx,
                                int& hghidx, double& relParm) const
{
  int sz = trk.size();

  at_s = fmod(at_s,length);
  if (at_s < 0.0) at_s += length;

  hghidx = findUpper(at_s);
  lowidx = hghidx-1; if (lowidx < 0) lowidx = sz-1;
  if (hghidx >= sz) hghidx = 0;

  double ds   = at_s - trk[lowidx]->s;
  double span = trk[hghidx]->s - trk[lowidx]->s;

  if (span < 0.0) span += length;
  if (ds   < 0.0) ds   += length;

  relParm = ds/span;
}

/* ---------------------------------------------------------------------- */

void ArcLinTrack::setCoTrack(const ArcLinTrack& coTrk,
                                      bool reverseDir,double maxSDiff)
{
  maxSDiff /= 2.0;

  double lastS = 0;

  int sz = trk.size();

  for (int i=0; i<sz; i++) {
    if (lastS < 0) lastS += coTrk.length;
    else if (lastS > coTrk.length) lastS -= coTrk.length;

    // Yes this is not entirely correct at start and finish, but:

    double minS = lastS - maxSDiff;
    if (minS < 0) minS += coTrk.length;

    double maxS = lastS + maxSDiff;
    if (maxS > coTrk.length) maxS -= coTrk.length;

    lastS = coTrk.findPoint(*trk[i],minS,maxS,trk[i]->zDir);
    
    trk[i]->zDir -= *trk[i]; trk[i]->zDir.unitLen3();

    if (reverseDir) trk[i]->zDir *= -1.0;
  }
}

//---------------------------------------------------------------------------

double ArcLinTrack::getMaxS() const
{
  if (trk.size() < 1) return 0.0;

  return trk[trk.size()-1]->s;
}

//---------------------------------------------------------------------------

void ArcLinTrack::getPoint(double at_s, Vec3& p) const
{
  int sz = trk.size();

  if (!closed) {
    if (at_s < 0.0) {
      trk[0]->getDir(0.0,p);
      
      p.unitLen3(); p *= at_s;
      p += trk[0]->getPoint();

      return;
    }
    else if (at_s > length) {
      int idx = sz - 1;
      if (idx < 0) idx = 0;

      trk[idx]->getDir(0.0,p);
      
      p.unitLen3(); p *= (at_s-length);
      p += trk[idx]->getPoint();

      return;
    }
  }

  int lowidx, hghidx;
  double rel_parm;

  findPointPair(at_s, lowidx, hghidx, rel_parm);

  Vec3 p2;
  trk[lowidx]->getPoint(rel_parm,p);      p  *= (1.0 - rel_parm);
  trk[hghidx]->getPoint(rel_parm-1.0,p2); p2 *= rel_parm;

  p += p2;
}

//---------------------------------------------------------------------------

void ArcLinTrack::getPointAndDir(double at_s, Vec3& pnt, Vec3& dir) const
{
  int sz = trk.size();

  if (!closed) {
    if (at_s < 0.0) {
      trk[0]->getDir(0.0,dir);
      
      dir.unitLen3();
      
      pnt = dir; pnt *= at_s;
      pnt += trk[0]->getPoint();

      return;
    }
    else if (at_s > length) {
      int idx = sz - 1;
      if (idx < 0) idx = 0;

      trk[idx]->getDir(0.0,dir);
      
      dir.unitLen3();
      
      pnt = dir; pnt *= (at_s-length);
      pnt += trk[idx]->getPoint();

      return;
    }
  }

  int lowIdx, hghIdx;
  double relParm;

  findPointPair(at_s, lowIdx, hghIdx, relParm);

  Vec3 v1,v2,p1,p2;
  trk[lowIdx]->getDir(relParm,v1);       
  trk[lowIdx]->getPoint(relParm,p1);

  trk[hghIdx]->getDir(relParm-1.0,v2);    
  trk[hghIdx]->getPoint(relParm-1.0,p2);

  v1 *= (1.0-relParm); v2 *= relParm;

  dir = v1; dir += v2; dir += (p2 - p1);

  if (dir.len3() > 0.0) dir.unitLen3();

  p1 *= (1.0 - relParm);
  p2 *= relParm;

  pnt = p1; pnt += p2;
}

//---------------------------------------------------------------------------

void ArcLinTrack::getDir(double at_s, Vec3& dir) const
{
  int sz = trk.size();

  if (!closed) {
    if (at_s < 0.0) {
      trk[0]->getDir(0.0,dir);
      dir.unitLen3();

      return;
    }
    else if (at_s > length) {
      int idx = sz - 1;
      if (idx < 0) idx = 0;

      trk[idx]->getDir(0.0,dir);
      dir.unitLen3();

      return;
    }
  }

  int lowidx, hghidx;
  double rel_parm;

  findPointPair(at_s, lowidx, hghidx, rel_parm);

  Vec3 v1,v2,p1,p2;
  trk[lowidx]->getDir(rel_parm,v1);     v1 *= (1.0-rel_parm);
  trk[lowidx]->getPoint(rel_parm,p1);

  trk[hghidx]->getDir(rel_parm-1.0,v2); v2 *= rel_parm;
  trk[hghidx]->getPoint(rel_parm-1.0,p2);

  dir = v1; dir += v2; dir += (p2 - p1);

  if (dir.len3() > 0.0) dir.unitLen3();
}

//---------------------------------------------------------------------------

void ArcLinTrack::getAcc(double at_s, Vec3& acc) const
{
    if (!closed) {
      if (at_s < 0.0 || at_s > length) {
        acc = Vec3(0,0,0);
        acc.isDerivative = true;

        return;
      }
    }

    int lowidx, hghidx;
    double rel_parm;

    findPointPair(at_s, lowidx, hghidx, rel_parm);

    Vec3 a1,a2,v1,v2,p1,p2;
    trk[lowidx]->getAcc(rel_parm,a1);
    trk[lowidx]->getDir(rel_parm,v1);
    trk[lowidx]->getPoint(rel_parm,p1);

    trk[hghidx]->getAcc(rel_parm-1.0,a2);
    trk[hghidx]->getDir(rel_parm-1.0,v2);
    trk[hghidx]->getPoint(rel_parm-1.0,p2);

    a1 *= (1.0-rel_parm); a2 *= rel_parm;
    acc = a1; acc += a2; acc += (v2 - v1)*2.0;

    v1 *= (1.0-rel_parm); v2 *= rel_parm;

    Vec3 dir = v1; dir += v2; dir += (p2 - p1);

    double len = dir.len3();
    dir /= len;

    double proj = acc * dir;

    acc -= (dir*proj);
    acc /= (len*len);
}

//---------------------------------------------------------------------------

void ArcLinTrack::getJerk(double at_s, Ino::Vec3& jerk) const
{
    if (!closed) {
      if (at_s < 0.0 || at_s > length) {
        jerk = Vec3(0,0,0);
        jerk.isDerivative = true;

        return;
      }
    }

    int lowidx, hghidx;
    double rel_parm;

    findPointPair(at_s, lowidx, hghidx, rel_parm);

    Vec3 j1,j2,a1,a2,v1,v2,p1,p2;
    trk[lowidx]->getJerk(rel_parm,j1);
    trk[lowidx]->getAcc(rel_parm,a1);
    trk[lowidx]->getDir(rel_parm,v1);
    trk[lowidx]->getPoint(rel_parm,p1);

    trk[hghidx]->getJerk(rel_parm-1.0,j2);
    trk[hghidx]->getAcc(rel_parm-1.0,a2);
    trk[hghidx]->getDir(rel_parm-1.0,v2);
    trk[hghidx]->getPoint(rel_parm-1.0,p2);

    j1 *= (1.0-rel_parm); j2 *= rel_parm;
    jerk = j1; jerk += j2; jerk += (a2 - a1)*3.0;

    a1 *= (1.0-rel_parm); a2 *= rel_parm;

    Vec3 acc = a1; acc += a2; acc += (v2 - v1)*2.0;

    v1 *= (1.0-rel_parm); v2 *= rel_parm;

    Vec3 dir = v1; dir += v2; dir += (p2 - p1);

    double len = dir.len3();
    dir /= len;

    double proj = acc * dir;

    acc -= dir*proj;
    acc /= (len*len);

    double len3 = len*len*len;

    jerk /= len3;
    proj *= 3.0;
    jerk -= (acc * proj);

    proj = jerk*dir + acc*acc;

    jerk -= (dir * proj);
}

//---------------------------------------------------------------------------

void ArcLinTrack::getXDir(double at_s, Vec3& x, Vec3& xDer) const
{
  x.isDerivative = false;
  xDer.isDerivative = true;

  int sz = trk.size();

  if (!closed) {
    if (at_s < 0.0) {
      x = trk[0]->getZDir();
      xDer = Vec3(0,0,0);
      return;
    }
    else if (at_s > length) {
      int idx = sz - 1;
      if (idx < 0) idx = 0;

      x = trk[idx]->getZDir();
      xDer = Vec3(0,0,0);

      return;
    }
  }

  int lowidx, hghidx;
  double rel_parm;

  findPointPair(at_s, lowidx, hghidx, rel_parm);

  Vec3 a1,a2,v1,v2,p1,p2;
  trk[lowidx]->getAcc(rel_parm,a1);
  trk[lowidx]->getDir(rel_parm,v1);
  trk[lowidx]->getPoint(rel_parm,p1);

  trk[hghidx]->getAcc(rel_parm-1.0,a2);
  trk[hghidx]->getDir(rel_parm-1.0,v2);
  trk[hghidx]->getPoint(rel_parm-1.0,p2);

  a1 *= (1.0-rel_parm); a2 *= rel_parm;
  Vec3 acc = a1; acc += a2; acc += (v2 - v1)*2.0;

  v1 *= (1.0-rel_parm); v2 *= rel_parm;

  Vec3 dir = v1; dir += v2; dir += (p2 - p1);

  double len = dir.len3();
  dir /= len;

  double proj = acc * dir;

  acc -= (dir*proj);
  acc /= (len*len);

  Vec3 z  = trk[lowidx]->getZDir();
  Vec3 z2 = trk[hghidx]->getZDir();

  Vec3 dz = z2; dz -= z; dz /= trk[lowidx]->maxS;
 
  z *= (1.0-rel_parm);
  z2 *= rel_parm;

  z += z2;

  x = dir.outer(z);

  double xLen = x.len3();

  x /= xLen;

  xDer = acc.outer(z); xDer += dir.outer(dz); xDer /= xLen;

  Vec3 dx2 = x; dx2 *= (x*xDer);

  xDer -= dx2;
}

//---------------------------------------------------------------------------

static double along(const Vec3& p1, const Vec3& p2, const Vec3& p,
                                                    Vec3& prjP, double &dist)
{
  Vec3 dir(p2); dir -= p1; dir.unitLen3();

  Vec3 pp(p); pp -= p1;

  double s = dir * pp;

  prjP = dir; prjP *= s; prjP += p1;

  dir.rot90();

  dist = fabs(dir * pp);

  return s;
}

//---------------------------------------------------------------------------

double ArcLinTrack::findPoint(const Vec3& p) const
{
  Vec3 trkPt;

  return findPoint(p,trkPt);
}

// --------------------------------------------------------------------------
// --- Currently interpolated as a polyline (to be improved) ----------------
// --------------------------------------------------------------------------

double ArcLinTrack::findPoint(const Vec3& p, Vec3& trkPt) const
{
  int sz = trk.size();

  if (sz < 2) return 0.0;

  int idx = 0;
  double minDist = 0.0;

  for (int i=0; i<sz; i++) {
    const Vec3& trkP = trk[i]->getPoint();

    double dist = p.distTo3(trkP);

    if (i < 1 || dist < minDist) {
      minDist = dist;
      idx = i;
    }
  }

  double s1=0.0, minDist1 = minDist;
  Vec3 trkPt1;

  int idx1 = idx-1, idx2 = idx;
  if (idx1 < 0) idx1 = sz-1;

  if (idx < 1 && !closed) {
    s1 =  along(trk[0]->getPoint(),trk[1]->getPoint(),p,trkPt1,minDist1);
    s1 += trk[0]->getS();
  }
  else {
    s1 =  along(trk[idx1]->getPoint(),trk[idx2]->getPoint(),p,trkPt1,minDist1);
    s1 += trk[idx1]->getS();
  }


  double s2=0.0, minDist2 = minDist;
  Vec3 trkPt2;

  idx1 = idx; idx2 = idx+1;
  if (idx2 >= sz) idx2 = 0;

  if (idx >= sz-1 && !closed) {
    s2 =  along(trk[sz-2]->getPoint(),trk[sz-1]->getPoint(),p,trkPt2,minDist2);
    s2 += trk[sz-2]->getS();
  }
  else {
    s2 =  along(trk[idx1]->getPoint(),trk[idx2]->getPoint(),p,trkPt2,minDist2);
    s2 += trk[idx1]->getS();
  }

  if (minDist1 < minDist2) {
    trkPt = trkPt1;
    return s1;
  }
  else {
    trkPt = trkPt2;
    return s2;
  }
}

//---------------------------------------------------------------------------

double ArcLinTrack::findPoint(const Vec3& p, double minS, double maxS,
                                                        Vec3& trkPt) const
{
  int sz = trk.size();

  if (sz < 2) return 0.0;

  int idx = 0;
  double minDist = 0.0;
  bool fst = true;

  for (int i=0; i<sz; i++) {
    const Vec3& trkP = trk[i]->getPoint();

    double s = trk[i]->getS();

    if (minS <= maxS) {
      if (s < minS || s > maxS) continue;
    }
    else {
      if (s < minS && s > maxS) continue;
    }

    double dist = p.distTo3(trkP);

    if (fst || dist < minDist) {
      minDist = dist;
      idx = i;
      fst = false;
    }
  }

  if (fst) return 0.0;

  double s1=0.0, minDist1 = minDist;
  Vec3 trkPt1;

  int idx1 = idx-1, idx2 = idx;
  if (idx1 < 0) idx1 = sz-1;

  if (idx < 1 && !closed) {
    s1 =  along(trk[0]->getPoint(),trk[1]->getPoint(),p,trkPt1,minDist1);
    s1 += trk[0]->getS();
  }
  else {
    s1 =  along(trk[idx1]->getPoint(),trk[idx2]->getPoint(),p,trkPt1,minDist1);
    s1 += trk[idx1]->getS();
  }


  double s2=0.0, minDist2 = minDist;
  Vec3 trkPt2;

  idx1 = idx; idx2 = idx+1;
  if (idx2 >= sz) idx2 = 0;

  if (idx >= sz-1 && !closed) {
    s2 =  along(trk[sz-2]->getPoint(),trk[sz-1]->getPoint(),p,trkPt2,minDist2);
    s2 += trk[sz-2]->getS();
  }
  else {
    s2 =  along(trk[idx1]->getPoint(),trk[idx2]->getPoint(),p,trkPt2,minDist2);
    s2 += trk[idx1]->getS();
  }

  if (minDist1 < minDist2) {
    trkPt = trkPt1;
    return s1;
  }
  else {
    trkPt = trkPt2;
    return s2;
  }
}

} // namespace

// Interface Section

void* ArcLinTrackNew(bool trkClosed, double trackPipeDiameter) {
  InoKin::ArcLinTrack* trk = new InoKin::ArcLinTrack(trkClosed, trackPipeDiameter);

  return trk;
}

void ArcLinTrackClear(void* track) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->clear();
}

bool ArcLinTrackSetTrack(void* track, const Ino::Vec3* ptList, int ptSz, bool trackClosed,
  double trackPipeDiameter)
{
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return false;
}


int ArcLinTrackAddTrackPoint(void* track, double x, double y, double z)
{
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->addPoint(Vec3(x, y, z));
}

void ArcLinTrackSetCoTrack(void* track, void* coTrack, bool reverseDir, double maxSDiff)   {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;
  InoKin::ArcLinTrack* coTrk = (InoKin::ArcLinTrack*)coTrack;

  trk->setCoTrack(*coTrk, reverseDir, maxSDiff);
}

int ArcLinTrackGetSize(void* track) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->size();
}

double ArcLinTrackGetMaxS(void* track)   {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->getMaxS();
}

bool ArcLinTrackIsClosed(void* track)   {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->isClosed();
}

void ArcLinTrackSetClosed(void* track, bool closed)   {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->setClosed(closed);
}

void ArcLinTrackSetRelations(void* track)
{
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->setRelations();
}

void ArcLinTrackValidate(void* track) // Also set Length
{
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->validate();
}

double ArcLinTrackGetPipeRadius(void* track) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->getPipeRadius();
}

void ArcLinTrackSetPipeRadius(void* track, double r) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->setPipeRadius(r);
}

void ArcLinTrackGetPoint(void* track, int idx, Vec3& v) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  v = trk->getPoint(idx);
}

void ArcLinTrackCalcCentroid(void* track, Vec3& centroid) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->calcCentroid();
}

void ArcLinTrackTranslate(void* track, Vec3 offset) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->translate(offset);
}

double ArcLinTrackGetLength(void* track) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->getLength();
}

void ArcLinTrackGetPointAndDir(void* track, double at_s, Vec3& pnt, Vec3& dir) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->getPointAndDir(at_s, pnt, dir);
}

void ArcLinTrackGetAccAndJerk(void* track, double at_s, Vec3& acc, Vec3& jerk) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->getAcc(at_s, acc);
  trk->getJerk(at_s, jerk);
}

void ArcLinTrackGetXDir(void* track, double at_s, Vec3& x, Vec3& xDir) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  trk->getXDir(at_s, x, xDir);
}

double ArcLinTrackFindPoint(void* track, Vec3 p, Vec3& trkPt) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->findPoint(p, trkPt);
}

double ArcLinTrackFindPoint2(void* track, Vec3 p, double minS, double maxS, Vec3& trkPt) {
  InoKin::ArcLinTrack* trk = (InoKin::ArcLinTrack*)track;

  return trk->findPoint(p, minS, maxS, trkPt);
}

// End Interface Section

//---------------------------------------------------------------------------
