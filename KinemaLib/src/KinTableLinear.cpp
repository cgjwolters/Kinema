//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- 1D Linear Function Interpolator (Polygon model) ------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinTableLinear.h"

#include <cmath>

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

TableLinear::TableLinear()
: TableFunction()
{
}

//---------------------------------------------------------------------------

TableLinear::TableLinear(int initCap, bool closed, double totalLength)
: TableFunction(initCap, closed, totalLength)
{
}

//---------------------------------------------------------------------------

TableLinear::TableLinear(const TableLinear& cp)
: TableFunction(cp)
{
}

//---------------------------------------------------------------------------

TableLinear& TableLinear::operator=(const TableLinear& src)
{
  TableFunction::operator=(src);
  return *this;
}

//---------------------------------------------------------------------------

double TableLinear::startOfRange() const
{
  if (xLst.size() > 0) return xLst[0];

  return 0.0;
}

//---------------------------------------------------------------------------

double TableLinear::endOfRange() const
{
  int sz = xLst.size();

  if (sz > 0) {
    if (closed) return xLength;
    else        return xLst[sz-1];
  }

  return 0.0;
}

//---------------------------------------------------------------------------

double TableLinear::getValueAt(double x) const
{
  int sz = xLst.size();

  if (sz < 1) return 0.0;

  if (closed) {
    double lx = fmod(x - xLst[0],xLength);
    if (lx < 0.0) lx += xLength;

    x = xLst[0] + lx;
  }

  int hghIdx = find(x);
  int lowIdx = hghIdx-1;

  if (lowIdx < 0) {
    if (!closed) return yLst[0];
    lowIdx = sz-1;
  }

  if (hghIdx >= sz) {
    if (!closed) return yLst[sz-1];
    hghIdx = 0;
  }

  double xRel = (x - xLst[lowIdx])/(xLst[hghIdx] - xLst[lowIdx]);

  return yLst[hghIdx]*xRel + yLst[lowIdx]*(1.0-xRel);
}

//---------------------------------------------------------------------------

double TableLinear::getDerivativeAt(double x) const
{
  int sz = xLst.size();

  if (sz < 1) return 0.0;

  if (closed) {
    double lx = fmod(x - xLst[0],xLength);
    if (lx < 0.0) lx += xLength;

    x = xLst[0] + lx;
  }

  int hghIdx = find(x);
  int lowIdx = hghIdx-1;

  if (lowIdx < 0) {
    if (!closed) return 0.0;
    lowIdx = sz-1;
  }

  if (hghIdx >= sz) {
    if (!closed) return 0.0;
    hghIdx = 0;
  }

  return (yLst[hghIdx]-yLst[lowIdx])/(xLst[hghIdx]-xLst[lowIdx]);
}

//---------------------------------------------------------------------------

double TableLinear::getSecDerivativeAt(double /*x*/) const
{
  return 0.0;
}

} // namespace

//-------------------------------------------------------------------------------
