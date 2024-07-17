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

#ifndef INOKIN_TABLELINEAR_INC
#define INOKIN_TABLELINEAR_INC

#include "KinTableFunction.h"

namespace InoKin {

//---------------------------------------------------------------------------

class TableLinear : public TableFunction
{
public:
  explicit TableLinear();
  explicit TableLinear(const TableLinear& cp);
  explicit TableLinear(int initCap, bool closed, double totalLength);

  virtual TableLinear& operator=(const TableLinear& src);

  virtual double startOfRange() const;
  virtual double endOfRange() const;

  virtual double getValueAt(double x) const;
  virtual double getDerivativeAt(double x) const;
  virtual double getSecDerivativeAt(double x) const;
};

} // namespace

//---------------------------------------------------------------------------
#endif
