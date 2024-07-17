//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- 1D Function Interpolator Based on Track.cpp ----------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinTableArc.h"

#include "KinArcLinTrack.h"

#include "Vec.h"
#include "Exceptions.h"

#include <cmath>

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

void TableArc::setupTrack() const
{
  if (trk) {
    delete trk;
    trk = NULL;
  }

  int sz = size();
  if (sz < 1) return;

  Vec3 *ptLst = (Vec3 *)alloca((sz+2) * sizeof(Vec3));

  Vec3 p(xVal(0)-1.0,yVal(0));
  ptLst[0] = p;

  p.x = xVal(0);
  ptLst[1] = p;

  for (int i=1; i<sz; i++) {
    p.x = xVal(i);
    p.y = yVal(i);

    ptLst[i+1] = p;
  }

  p.x += 1.0;
  ptLst[sz+1] = p;

  ArcLinTrack *linTrk = new ArcLinTrack();
  linTrk->setTrack(ptLst,sz+2,true);

  trk = linTrk;
}

//---------------------------------------------------------------------------

TableArc::TableArc()
: TableFunction(), trk(NULL)
{
}

//---------------------------------------------------------------------------

TableArc::TableArc(int initCap, bool closed, double totalLength)
: TableFunction(initCap, closed, totalLength), trk(NULL)
{
}

//---------------------------------------------------------------------------

TableArc::TableArc(const TableArc& cp)
: TableFunction(cp), trk(NULL)
{
}

TableArc::~TableArc()
{
  delete trk;
}

//---------------------------------------------------------------------------

TableArc& TableArc::operator=(const TableArc& src)
{
  TableFunction::operator=(src);

  delete trk;
  trk = NULL;

  return *this;
}

//---------------------------------------------------------------------------

double TableArc::startOfRange() const
{
  if (size() > 0) return xLst[0];
  return 0.0;
}

//---------------------------------------------------------------------------

double TableArc::endOfRange() const
{
  int sz = xLst.size();

  if (sz > 0) {
    if (closed) return xLength;
    else        return xLst[sz-1];
  }

  return 0.0;
}

//---------------------------------------------------------------------------

double TableArc::getValueAt(double x) const
{
  int sz = xLst.size();

  if (sz < 1) return 0.0;

  if (!trk) setupTrack();
  if (!trk) throw NullPointerException("TableArc::getValueAt");

  if (closed) {
    double lx = fmod(x - xLst[0],xLength);
    if (lx < 0.0) lx += xLength;

    x = xLst[0] + lx;
  }

  double lwb = 0.0;
  double upb = trk->getMaxS();

  Vec3 pos;
  trk->getPoint(lwb,pos);
  if (pos.x >= x) return pos.y;

  trk->getPoint(upb,pos);
  if (pos.x <= x) return pos.y;

  for (;;) {
    double s = (lwb + upb)/2.0;

    trk->getPoint(s,pos);

    if (pos.x < x - 1.0e-4) lwb = s;
    else if (pos.x > x + 1.0e-4) upb = s;
    else break;
  }

  return pos.y;
}

//---------------------------------------------------------------------------

double TableArc::getDerivativeAt(double x) const
{
  int sz = xLst.size();

  if (sz < 1) return 0.0;

  if (!trk) setupTrack();
  if (!trk) throw NullPointerException("TableArc::getValueAt");

  if (closed) {
    double lx = fmod(x - xLst[0],xLength);
    if (lx < 0.0) lx += xLength;

    x = xLst[0] + lx;
  }

  double lwb = 0.0;
  double upb = trk->getMaxS();

  Vec3 pos;
  trk->getPoint(lwb,pos);
  if (pos.x >= x) return 0.0;

  trk->getPoint(upb,pos);
  if (pos.x <= x) return 0.0;

  Vec3 dir;

  for (;;) {
    double s = (lwb + upb)/2.0;

    trk->getPoint(s,pos);
    trk->getDir(s,dir);

    if (pos.x < x - 1.0e-4) lwb = s;
    else if (pos.x > x + 1.0e-4) upb = s;
    else break;
  }

  return dir.y/dir.x;
}

//---------------------------------------------------------------------------

double TableArc::getSecDerivativeAt(double x) const
{
  int sz = xLst.size();

  if (sz < 1) return 0.0;

  if (!trk) setupTrack();
  if (!trk) throw NullPointerException("TableArc::getValueAt");

  if (closed) {
    double lx = fmod(x - xLst[0],xLength);
    if (lx < 0.0) lx += xLength;

    x = xLst[0] + lx;
  }

  double lwb = 0.0;
  double upb = trk->getMaxS();

  Vec3 pos;
  trk->getPoint(lwb,pos);
  if (pos.x >= x) return 0.0;

  trk->getPoint(upb,pos);
  if (pos.x <= x) return 0.0;

  Vec3 dir,acc;
  double s;

  for (;;) {
    s = (lwb + upb)/2.0;

    trk->getPoint(s,pos);

    if (pos.x < x - 1.0e-4) lwb = s;
    else if (pos.x > x + 1.0e-4) upb = s;
    else break;
  }

  trk->getDir(s,dir);
  trk->getAcc(s,acc);

  return (acc.y - dir.y/dir.x*acc.x)/sqr(dir.x);
}

} // namespace

//---------------------------------------------------------------------------
