//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---- Basic 3D B-SPline ----------------------------------------------------
//---------------------------------------------------------------------------
//---- Copyright Inofor Hoek Aut BV 2006-2013 -------------------------------
//---------------------------------------------------------------------------
//---- C. Wolters Sep 2006 --------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef SPLINE3D_INC
#define SPLINE3D_INC

// ATTN: Untested code!

#include "Vec.h"
#include "Array.h"

namespace Ino {
  class Trf3;
  class Matrix;
}

//---------------------------------------------------------------------------

class Spline3D
{
  static const double MinTmDiff;  // == 0.01

  short splineDegree;
  bool splineClosed;

  Ino::Array<double> knotLst;
  Ino::Array<Ino::Vec3> controlLst;

  bool buildBaseKnotList(int controlSz, const Ino::Vec3 *ptLst, int ptLstSz,
                                                Ino::Array<double>& parLst);

  bool buildBaseMat(int controlSz, const Ino::Vec3 *ptLst, int ptLstSz,
                    const Ino::Array<double>& parLst,
                    Ino::Matrix& mat, Ino::Matrix& rhs);

  bool buildSimpleSpline(int controlSz, const Ino::Vec3 *ptLst, int ptLstSz);

  double deBoor1D(int degree, int idx, double *p, int steps, double tm) const;
  void deBoor(int degree, int idx, Ino::Vec3 *p, int steps, double tm) const;

  int multiplicity(int knotIdx) const;
//  bool removeKnotHgh(int idx);

public:
  Spline3D();

  void ensureControlCapacity(int minCap);

  void clear();
  bool isEmpty() const { return controlSz() < 1; }
  bool isClosed() const { return splineClosed; }
  short degree() const { return splineDegree; }

  int  knotSz() const { return knotLst.size(); }
  int  controlSz() const {return controlLst.size(); }

  double knot(int idx) const;
  int knotIndex(double u) const;
  int knotIndex(double u, double& ur) const;

  double abcis(int idx) const;
  int controlIdx(double u) const;

  void transform(const Ino::Trf3& trf);

  void controlDot(int idx, double& u, Ino::Vec3& v) const;
  bool setControlDot(int idx, const Ino::Vec3& newVal);

  void controlPt(int idx, double& u, Ino::Vec3& cpl) const;
  void setControlPt(int idx, const Ino::Vec3& newCp);

  int pickControl(double u);

  double minU() const;
  double maxU() const;

  void setMinU(double newMinU);
  bool point(double u, Ino::Vec3& pt) const;

  bool derivativeAt(double u, Ino::Vec3& derPt) const;
  bool derivativeAt(int der, double u, Ino::Vec3& derPt) const;

  bool derivative(Spline3D& dst) const;
  bool integral(Spline3D& dst,
                const Ino::Vec3& startPt=Ino::Vec3(0,0,0)) const;

  enum BuildMode { BuildSimple, BuildCoMonoTone,
                   BuildCoConvex, BuildCoMonotoneConvex };

  bool build(int degree, bool closed, int controlSz,
             double maxAvgDist, BuildMode mode, 
             const Ino::Vec3 *ptLst, int ptLstSz,
             double& rmsDist, double& maxDist);
};

//---------------------------------------------------------------------------
#endif
