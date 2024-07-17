//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//-------------- Base Class for all 1D Table Interpolators ------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#include "KinTableFunction.h"

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

int TableFunction::find(double x) const
{
  int lwb = 0, upb = xLst.size()-1;

  while (lwb <= upb) {
    int cur = (lwb+upb)/2;

    double val = xLst[cur];

    if (val < x) lwb = cur+1;
    else upb = cur-1;
  }

  return lwb;
}

//---------------------------------------------------------------------------

TableFunction::TableFunction()
: xLst(10), yLst(10), closed(false), xLength(0.0)
{
}

//---------------------------------------------------------------------------

TableFunction::TableFunction(int initCap, bool fncClosed, double totalLength)
: xLst(initCap), yLst(initCap),
  closed(fncClosed), xLength(totalLength)
{
}

//---------------------------------------------------------------------------

TableFunction::TableFunction(const TableFunction& cp)
: xLst(cp.xLst), yLst(cp.yLst),
  closed(cp.closed), xLength(cp.xLength)
{
}

//---------------------------------------------------------------------------

TableFunction::~TableFunction()
{
}

//---------------------------------------------------------------------------

TableFunction& TableFunction::operator=(const TableFunction& src)
{
  xLst   = src.xLst;
  yLst   = src.yLst;
  closed = src.closed;
  xLength = src.xLength;

  return *this;
}

//---------------------------------------------------------------------------

void TableFunction::setClosed(bool fncClosed, double totalLength)
{
  closed = fncClosed;
  xLength = totalLength;
}

//---------------------------------------------------------------------------

void TableFunction::addValue(double x, double y)
{
  xLst.add(x);
  yLst.add(y);
}

//---------------------------------------------------------------------------

double TableFunction::xVal(int idx) const
{
  if (idx < 0 || idx >= xLst.size()) idx = 0;

  return xLst[idx];
}

//---------------------------------------------------------------------------

double& TableFunction::xVal(int idx)
{
  if (idx < 0 || idx >= xLst.size()) idx = 0;

  return xLst[idx];
}

//---------------------------------------------------------------------------

double TableFunction::yVal(int idx) const
{
  if (idx < 0 || idx >= yLst.size()) idx = 0;

  return yLst[idx];
}

//---------------------------------------------------------------------------

double& TableFunction::yVal(int idx)
{
  if (idx < 0 || idx >= yLst.size()) idx = 0;

  return yLst[idx];
}

} // namespace

//-------------------------------------------------------------------------------
