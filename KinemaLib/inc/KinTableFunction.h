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

#ifndef INOKIN_TABLEFUNCTION_INC
#define INOKIN_TABLEFUNCTION_INC

#include "Array.h"

namespace InoKin {

//-------------------------------------------------------------------------------

class TableFunction : public Ino::ArrayElem
{
protected:
  Ino::Array<double> xLst;
  Ino::Array<double> yLst;

  bool closed;
  double xLength;

  int find(double x) const;

public:
  explicit TableFunction();
  explicit TableFunction(int initCap, bool fncClosed, double totalLength = 0);
  explicit TableFunction(const TableFunction& cp);

  virtual ~TableFunction();

  virtual TableFunction& operator=(const TableFunction& src);

  int size() const { return xLst.size(); }

  bool isClosed() const { return closed; }
  void setClosed(bool fncClosed, double totalLength=0.0);

  void addValue(double x, double y);

  double xVal(int idx) const;
  double yVal(int idx) const;

  double& xVal(int idx);
  double& yVal(int idx);

  virtual double startOfRange() const = 0;
  virtual double endOfRange() const = 0;

  virtual double getValueAt(double x) const = 0;
  virtual double getDerivativeAt(double x) const = 0;
  virtual double getSecDerivativeAt(double x) const = 0;
};

} // namespace

//-------------------------------------------------------------------------------
#endif
