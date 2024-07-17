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

#ifndef INOKIN_TABLEARC_INC
#define INOKIN_TABLEARC_INC

#include "KinTableFunction.h"

namespace InoKin {

//---------------------------------------------------------------------------

class AbstractTrack;

class TableArc : public TableFunction
{
  mutable AbstractTrack *trk;

  void setupTrack() const;

public:
  explicit TableArc();
  explicit TableArc(const TableArc& cp);
  explicit TableArc(int initCap, bool closed, double totalLength);
  ~TableArc();

  virtual TableArc& operator=(const TableArc& src);

  virtual double startOfRange() const;
  virtual double endOfRange() const;

  virtual double getValueAt(double x) const;
  virtual double getDerivativeAt(double x) const;
  virtual double getSecDerivativeAt(double x) const;
};

} // namespace

//---------------------------------------------------------------------------
#endif
