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

// ATTN: Untested code!

#define _CRT_SECURE_NO_WARNINGS

#include "Spline3D.h"

#include "Exceptions.h"

#include "Vec.h"
#include "Trf.h"
#include "Matrix.h"

#include <cmath>

#ifdef _WIN32
#include <malloc.h>
#else
#include <alloca.h>
#endif

using namespace Ino;

//---------------------------------------------------------------------------

const double Spline3D::MinTmDiff = 0.01;

//---------------------------------------------------------------------------

Spline3D::Spline3D()
: splineDegree(0), splineClosed(),
  knotLst(), controlLst()
{
}

//---------------------------------------------------------------------------

void Spline3D::ensureControlCapacity(int minCap)
{
  if (minCap < controlLst.size()) minCap = controlLst.size();

  knotLst.ensureCapacity(minCap+splineDegree-1);
  controlLst.ensureCapacity(minCap);
}

//---------------------------------------------------------------------------

void Spline3D::clear()
{
  splineDegree = 0;
  splineClosed = false;

  knotLst.clear();
  controlLst.clear();
}

//---------------------------------------------------------------------------

double Spline3D::knot(int idx) const
{
  if (idx < 0 || idx >= knotLst.size())
    throw IndexOutOfBoundsException("Spline3D::knot");

  return knotLst[idx];
}

//---------------------------------------------------------------------------

static int knotIdx(double u, int degree, const Ino::Array<double>& knotLst)
{
  int minIdx = degree-1;
  int maxIdx = knotLst.size()-degree;

  double minU = knotLst[minIdx];
  double maxU = knotLst[maxIdx];
  double len  = maxU - minU;

  u = fmod(u-minU,len) + minU;

  int lwb = degree, upb = maxIdx;

  while (lwb <= upb) {
    int idx = (lwb+upb)/2;

    if (knotLst[idx] <= u) lwb = idx+1;
    else upb = idx-1;
  }

  while (lwb < maxIdx && fabs(knotLst[lwb+1] - u) <= 1e-12) ++lwb;

  if (lwb > maxIdx) lwb = maxIdx;

  return lwb;
}

//---------------------------------------------------------------------------

int Spline3D::knotIndex(double u) const
{
  return knotIdx(u,splineDegree,knotLst);
}

//---------------------------------------------------------------------------

int Spline3D::knotIndex(double u, double& ur) const
{
  ur = u;

  int idx = knotIndex(u);
  if (idx < 0) return idx;

  if (idx <= splineDegree) ur = knotLst[idx];
  else {
    int maxIdx = knotLst.size()-splineDegree;
    if (idx >= maxIdx) ur = knotLst[maxIdx];
  }

  return idx;
}

//---------------------------------------------------------------------------

double Spline3D::abcis(int idx) const
{
  if (idx < 0 || idx >= controlLst.size())
    throw IndexOutOfBoundsException("Spline3D::abcis");

  double a = 0.0;

  for (int i=idx; i<idx+splineDegree; ++i) a += knotLst[i];

  return a/splineDegree;
}

//---------------------------------------------------------------------------

int Spline3D::controlIdx(double u) const
{
  //Return the index of the first controlPoint that has 
  // a parameter value SMALLER than u.

  int ctrlSz = controlLst.size();

  // First educated guess
  int idx = knotIndex(u);
  if (idx < splineDegree) return -1;

  int newIdx = 0;

  double diff = 1e10;

  for (int i = idx - splineDegree; i < idx; ++i) {
    double ai = abcis(i);

    if (ai > u) return newIdx;

    double aiu = fabs(ai - u);

    if (aiu < diff) {
      diff = aiu;
      newIdx = i;
    }
  }

  return newIdx;
}

//---------------------------------------------------------------------------

//bool Spline3D::buildBaseMatA(Matrix2& mat, Matrix2& rhs) const
//{
//  if (!ptLst) return false;
//
//  double *p = (double *)alloca((splineDegree+1)*sizeof(double));
//
//  // Build matrix
//
//  int ptSz = ptLst->size();
//
//  Vec3 pt;
//
//  for (int i=0; i<ptSz; ++i) {
//    ptLst->get(i,pt);
//    rhs(i,0) = pt.x;
//    rhs(i,1) = pt.y;
//    rhs(i,2) = pt.z;
//
//    double curLen = ptLst->par(i);
//
//    int idx = knotIndex(curLen);
//    if (idx < splineDegree) idx = splineDegree;
//    if (idx >= ctrlSz) idx = ctrlSz-1;
//
//    //Find the value of Ni,p(u) using De Boor's algorithm
//
//    for (int j=0; j<splineDegree+1; ++j) {
//      memset(p,0,(splineDegree+1)*sizeof(double));
//      p[j] = 1.0;
//      double m = deBoor1D(splineDegree,idx,p,splineDegree,curLen);
//      mat(i,idx-splineDegree+j) = m;
//    }
//  }
//
//  return true;
//}

//---------------------------------------------------------------------------

void Spline3D::transform(const Trf3& trf)
{
  int sz = controlLst.size();

  for (int i=0; i<sz; ++i) controlLst[i].transform3(trf);
}

//---------------------------------------------------------------------------

void Spline3D::controlDot(int idx, double& u, Vec3& v) const
{
  if (idx < 0 || idx >= controlLst.size())
                  throw IndexOutOfBoundsException("Spline3D::controlPt");

  u = abcis(idx);
  point(u,v);
}

//---------------------------------------------------------------------------

bool Spline3D::setControlDot(int idx, const Vec3& newVal)
{
  int ctrlSz = controlLst.size();

  if (idx < 0 || idx >= ctrlSz)
    throw IndexOutOfBoundsException("Spline3D::setControlVal");

  if (idx < 1 || idx == ctrlSz-1) {
    controlLst[idx] = newVal;
    return true;
  }

  double u = abcis(idx);
  int valIdx = knotIndex(u);

  int bSz = (splineDegree+1)*sizeof(double);
  double *p = (double *)_malloca(bSz);
  memset(p,0,bSz);

  p[splineDegree-valIdx+idx] = 1.0;

  deBoor1D(splineDegree,valIdx,p,splineDegree,u);
  
  if (fabs(p[splineDegree]) <= 1e-12) return false;

  Vec3 uv; point(u,uv);
  uv *= -1.0; uv += newVal; uv /= p[splineDegree];

  controlLst[idx] += uv;

  return true;
}

//---------------------------------------------------------------------------

void Spline3D::controlPt(int idx, double& u, Vec3 & val) const
{
  if (idx < 0 || idx >= controlLst.size())
                  throw IndexOutOfBoundsException("Spline3D::controlPt");

  val = controlLst[idx];
  u   = abcis(idx);
}

//---------------------------------------------------------------------------

void Spline3D::setControlPt(int idx, const Vec3& newVal)
{
  if (idx < 0 || idx >= controlLst.size())
                throw IndexOutOfBoundsException("Spline3D::setControlVal");

  controlLst[idx] = newVal;
}

//---------------------------------------------------------------------------

int Spline3D::pickControl(double u)
{
  int idx = knotIndex(u);

  if (idx < splineDegree) return 0;

  if (fabs(knotLst[idx-1]-u) < fabs(knotLst[idx]-u)) return idx-1;

  return idx;
}

//---------------------------------------------------------------------------

double Spline3D::minU() const
{
  if (knotLst.size() < splineDegree) return 0.0;

  return knotLst[splineDegree-1];
}

//---------------------------------------------------------------------------

double Spline3D::maxU() const
{
  int maxIdx = knotLst.size()-splineDegree;

  if (maxIdx < 0) return 0.0;

  return knotLst[maxIdx];
}

//---------------------------------------------------------------------------

static double locDeBoor1D(int degree, const Ino::Array<double>& knotLst,
                         int idx, double *p, int steps, double u)
{
  for (int i=0; i<steps; ++i) {
    for (int j=degree-1; j>=i; --j) {
      double fstKnot = knotLst[idx+j-degree];
      double lstKnot = knotLst[idx+j-i];
      
      p[j+1] = (p[j]*(lstKnot-u) + p[j+1]*(u-fstKnot))/(lstKnot-fstKnot);
    }
  }

  return p[degree];
}

//---------------------------------------------------------------------------

double Spline3D::deBoor1D(int degree, int idx, double *p,
                                                 int steps, double u) const
{
  return locDeBoor1D(degree,knotLst,idx,p,steps,u);
}

//---------------------------------------------------------------------------

void Spline3D::deBoor(int degree, int idx, Vec3 *p,
                                                 int steps, double u) const
{
  for (int i=0; i<steps; ++i) {
    for (int j=degree-1; j>=i; --j) {
      double fstKnot = knotLst[idx+j-degree];
      double lstKnot = knotLst[idx+j-i];
      
      p[j+1] = (p[j]*(lstKnot-u) + p[j+1]*(u-fstKnot))/(lstKnot-fstKnot);
    }
  }
}

//---------------------------------------------------------------------------

void Spline3D::setMinU(double newMinU)
{
  int kSz = knotLst.size();
  if (kSz < splineDegree)return;

  double diff = newMinU - knotLst[splineDegree-1];

  for (int i=0; i<kSz; ++i) knotLst[i] += diff;
}

//---------------------------------------------------------------------------

int Spline3D::multiplicity(int knotIdx) const
{
  double v = knotLst[knotIdx];
  int kntSz = knotSz();

  while (knotIdx < kntSz-1 && fabs(knotLst[knotIdx+1]-v) <= 1e-12) ++knotIdx;

  int m = 1;

  for (int i=knotIdx-1; i>=0; --i) {
    if (fabs(knotLst[i]-v) > 1e-12) break;

    ++m;
  }

  return m;
}

//---------------------------------------------------------------------------

bool Spline3D::point(double u, Vec3& pt) const
{
  return derivativeAt(0,u,pt);
}

//---------------------------------------------------------------------------

bool Spline3D::derivativeAt(double u, Vec3& derPt) const
{
  return derivativeAt(1,u,derPt);
}

//---------------------------------------------------------------------------

bool Spline3D::derivativeAt(int der, double u, Vec3& derPt) const
{
  if (der > splineDegree) return false;

  int ctrlSz = controlLst.size();
  if (ctrlSz <= splineDegree) return false;

  int knotSz = knotLst.size();
  if (knotSz < ctrlSz+splineDegree-1) return false;

  if (splineClosed) {
    double mnU = minU();
    double parLen = maxU() - minU();

    u -= mnU;
    u = fmod(u,parLen);
    if (u < 0.0) u += parLen;
    u += mnU;
  }
  else {
    if (u < minU()) u = minU();
    else if (u > maxU()) u = maxU();
  }

  int idx = knotIndex(u);

  int pSz = splineDegree+1;
  Vec3 *p = (Vec3 *)_malloca(pSz*sizeof(Vec3));

  int cIdx = idx-splineDegree;
  if (cIdx < 0) cIdx += ctrlSz;

  for (int i=0; i<pSz; ++i) p[i] = controlLst[(cIdx+i)% ctrlSz];

  for (int i=0; i<der; ++i) {
    for (int j=0; j<splineDegree-i; ++j) {
      cIdx = j+idx-(splineDegree-i);

      p[j] *= -1.0; p[j] += p[j+1];
      p[j] /= knotLst[cIdx+splineDegree-i]-knotLst[cIdx];
      p[j] *= splineDegree-i;
    }
  }

  deBoor(splineDegree-der,idx,p,splineDegree-der,u);
  derPt = p[splineDegree-der];

  derPt.isDerivative = der > 0;

  return true;
}

//---------------------------------------------------------------------------

bool Spline3D::derivative(Spline3D& dst) const
{
  int ctrlSz = controlLst.size();

  if (&dst == this || splineDegree < 2 || ctrlSz <= splineDegree) return false;

  int knotSz = knotLst.size();
  if (knotSz < ctrlSz+splineDegree-1) return false;

  dst.splineDegree = splineDegree-1;
  dst.splineClosed = splineClosed;

  dst.ensureControlCapacity(ctrlSz-1);

  dst.knotLst = knotLst;
  dst.knotLst.remove(0);

  dst.controlLst.clear();

  for (int i=0; i<ctrlSz-1; ++i) {
    dst.controlLst.add(controlLst[i+1]);

    Vec3& cp = dst.controlLst[i];
    cp -= controlLst[i];
    cp /= knotLst[i+splineDegree]-knotLst[i];
    cp *= splineDegree;
  }

  return true;
}

//---------------------------------------------------------------------------
// Is niet af voor gesloten spline

bool Spline3D::integral(Spline3D& dst, const Vec3& startPt) const
{
  int ctrlSz = controlLst.size();

  if (&dst == this || splineDegree < 1 || ctrlSz <= splineDegree) return false;

  int knotSz = knotLst.size();
  if (knotSz < ctrlSz+splineDegree-1) return false;

  dst.splineDegree = splineDegree+1;
  dst.splineClosed = false;

  dst.ensureControlCapacity(ctrlSz+1);

  dst.knotLst = knotLst;
  dst.knotLst.insert(0,knotLst[0]);
  dst.knotLst[dst.knotSz()-1] = dst.knotLst[dst.knotSz()-2];

  dst.controlLst.clear();
  dst.controlLst.add(startPt);

  for (int i=0; i<ctrlSz; ++i) {
    dst.controlLst.add(controlLst[i]);
    Vec3& cp = dst.controlLst[i+1];
    cp /= dst.splineDegree;
    cp *= dst.knotLst[i+dst.splineDegree]-dst.knotLst[i];
    cp += dst.controlLst[i];
  }

  return true;
}

//---------------------------------------------------------------------------

static bool buildOpenBaseKnotLst(int controlSz, int degree,
                                 const Vec3 *ptLst, int ptLstSz,
                                 Ino::Array<double>& parLst,
                                 Ino::Array<double>& knotLst)
{
  double d = ptLstSz-1;
  int segments = controlSz-degree;

  d /= segments;

  if (d < 1.0) return false;

  knotLst.clear();
  parLst.clear();

  knotLst.ensureCapacity(controlSz + degree - 1);
  parLst.ensureCapacity(ptLstSz);

  double par = 0.0;

  parLst.add(0.0);

  for (int i=1; i<ptLstSz; ++i) {
    par += ptLst[i-1].distTo3(ptLst[i]);
    parLst.add(par);
  }
 
  for (int i=0; i<degree; ++i) knotLst.add(0.0);

  for (int i=1; i<segments; ++i) {
    double a = i*d;

    int idx = (int)a;
    a -= idx;

    knotLst.add((1.0-a)*parLst[idx] + a*parLst[idx+1]);
  }

  knotLst.add(parLst[ptLstSz-1]);

  for (int i=0; i<degree-1; ++i) knotLst.add(knotLst[controlSz-1]);

  return true;
}

//---------------------------------------------------------------------------

static bool buildClosedBaseKnotLst(int controlSz, int degree,
                                   const Vec3 *ptLst, int ptLstSz,
                                   Ino::Array<double>& parLst,
                                   Ino::Array<double>& knotLst)
{
  double d = ptLstSz-1;
  d /= controlSz;

  if (d < 1.0) return false;

  knotLst.clear();
  parLst.clear();

  knotLst.ensureCapacity(controlSz + 2*degree - 2);
  parLst.ensureCapacity(ptLstSz+1);

  double par = 0.0;

  parLst.add(0.0);

  for (int i=1; i<ptLstSz; ++i) {
    par += ptLst[i-1].distTo3(ptLst[i]);
    parLst.add(par);
  }

  parLst.add(ptLst[ptLstSz-1].distTo3(ptLst[0]));

  for (int i=0; i<degree; ++i) knotLst.add(0.0);

  for (int i=1; i<controlSz; ++i) {
    double a = i*d;

    int idx = (int)a;
    a -= idx;

    knotLst.add((1.0-a)*parLst[idx] + a*parLst[idx+1]);
  }

  double len = parLst[ptLstSz];
  for (int i=0; i<degree-1; ++i) knotLst.add(knotLst[i]+len);

  int endIdx = controlSz + degree - 2;
  for (int i=1; i<degree; ++i) knotLst[degree-1-i] = knotLst[endIdx-i]-len;

  FILE *fd = fopen("C:\\Temp\\closedknots.csv","w");
  for (int i=0; i<knotLst.size(); i++) fprintf(fd,"%d;%.5f\n",i,knotLst[i]);
  fclose(fd);

  fd = fopen("C:\\Temp\\closedknotdists.csv","w");

  double l = 0;

  fprintf(fd,"0;0.0\n");

  for (int i=1; i<ptLstSz; ++i) {
    l += ptLst[i-1].distTo3(ptLst[i]);
    fprintf(fd,"%d;%.6f\n",i,l);
  }

  l += ptLst[ptLstSz-1].distTo3(ptLst[0]);
  fprintf(fd,"%d;%.6f\n",ptLstSz,l);

  fclose(fd);

  return true;
}

//---------------------------------------------------------------------------

bool Spline3D::buildBaseKnotList(int controlSz, const Vec3 *ptLst, int ptLstSz,
                                 Ino::Array<double>& parLst)
{
  knotLst.clear();

  if (splineClosed) 
       return buildClosedBaseKnotLst(controlSz,splineDegree,
                                     ptLst,ptLstSz,parLst,knotLst);
  else return buildOpenBaseKnotLst(controlSz,splineDegree,
                                   ptLst,ptLstSz,parLst,knotLst);
}

//---------------------------------------------------------------------------

static bool buildOpenBaseMat(int ctrlSz, int degree,
                             const Vec3 *ptLst, int ptLstSz,
                             const Ino::Array<double>& parLst,
                             const Ino::Array<double>& knotLst,
                             Matrix& mat, Matrix& rhs)
{
  if (!ptLst) return false;

  int pSz = degree+1;

  double *p = (double *)_malloca(pSz*sizeof(double));
  double *m = (double *)_malloca(pSz*sizeof(double));

  // Build matrix

  for (int i=0; i<ptLstSz; ++i) {
    const Vec3& pt = ptLst[i];
    double par = parLst[i];

    int idx = knotIdx(par,degree,knotLst);

    //Find the value of Ni,p(u) using De Boor's algorithm

    for (int j=0; j<pSz; ++j) {
      memset(p,0,pSz*sizeof(double));
      p[j] = 1.0;
      m[j] = locDeBoor1D(degree,knotLst,idx,p,degree,par);
    }

    idx -= degree;

    for (int j=0; j<pSz; ++j) {
      int row = idx + j;
      if (row >= ctrlSz) row -= ctrlSz;

      for (int k=j; k<pSz; ++k) mat(row,k-j) += m[j] * m[k];

      Vec3 rp(pt);
      rp *= m[j];

      rhs(row,0) += rp.x;
      rhs(row,1) += rp.y;
      rhs(row,2) += rp.z;
    }
  }

  return true;
}

//---------------------------------------------------------------------------

static bool buildClosedBaseMat(int ctrlSz, int degree,
                               const Vec3 *ptLst, int ptLstSz,
                               const Ino::Array<double>& parLst,
                               const Ino::Array<double>& knotLst,
                               Matrix& mat, Matrix& rhs)
{
  if (!ptLst) return false;

  int pSz = degree+1;

  double *p = (double *)_malloca(pSz*sizeof(double));
  double *m = (double *)_malloca(pSz*sizeof(double));

  // Build matrix

//  FILE *fd = fopen("C:\\Temp\\Index.csv","w");

  for (int i=0; i<ptLstSz; ++i) {
    const Vec3& pt = ptLst[i];
    double par = parLst[i];

    int idx = knotIdx(par,degree,knotLst);

//    fprintf(fd,"%d;%d",i,idx);

    //Find the value of Ni,p(u) using De Boor's algorithm

    for (int j=0; j<pSz; ++j) {
      memset(p,0,pSz*sizeof(double));
      p[j] = 1.0;
      m[j] = locDeBoor1D(degree,knotLst,idx,p,degree,par);
//      fprintf(fd,";%.10f",m[j]);
    }

//    fprintf(fd,"\n");

    idx -= degree;

    for (int j=0; j<pSz; ++j) {
      int row = idx + j;
      if (row >= ctrlSz) row -= ctrlSz;

      for (int k=j; k<pSz; ++k) mat(row,k-j) += m[j] * m[k];

      Vec3 rp(pt);
      rp *= m[j];

      rhs(row,0) += rp.x;
      rhs(row,1) += rp.y;
      rhs(row,2) += rp.z;
    }

    if (i < ptLstSz-1) par += ptLst[i+1].distTo3(pt);
  }

//  fclose(fd);

  double *row = new double[ctrlSz];

  int rowIdx = ctrlSz - degree;

  for (int i=0; i<degree; ++i) {
    memset(row,0,ctrlSz * sizeof(double));

    for (int j=0; j<=degree; ++j) {
      int col = rowIdx + j;
      if (col >= ctrlSz) col -= ctrlSz;

      row[col] = mat(rowIdx,j);
    }

    for (int j=0; j<rowIdx; ++j) {
      double pivot = row[j]/mat(j,0);

      for (int k=0; k<=degree; ++k) {
        int col = j + k;
        if (col >= ctrlSz) break;

        double diff = pivot * mat(j,k);
        row[col] -= pivot * mat(j,k);
      }

      for (int k=0; k<3; ++k) rhs(rowIdx,k) -= pivot * rhs(j,k);
    }

    for (int j=0; j<degree-i; ++j) mat(rowIdx,j) = row[rowIdx+j];

    for (int j=degree-i; j<=degree; ++j) mat(rowIdx,j) = 0.0;

    ++rowIdx;
  }

  delete[] row;

  //fd = fopen("C:\\Temp\\Pts.csv","w");
  //for (int i=0; i<ptLstSz; ++i) fprintf(fd,"%.5f;%.5f;%.5f\n",ptLst[i].x,ptLst[i].y,ptLst[i].z);
  //fclose(fd);
  //
  //fd = fopen("C:\\Temp\\mat.csv","w");

  //for (int i=0; i<mat.getRows(); ++i) {
  //  fprintf(fd,"%d;%.9f",i,mat(i,0));
  //  for (int j=1; j<5; ++j) fprintf(fd,";%.9f",mat(i,j));
  //  fprintf(fd,"\n");
  //}

  //fclose(fd);

  return true;
}

//---------------------------------------------------------------------------

bool Spline3D::buildBaseMat(int ctrlSz, const Vec3 *ptLst, int ptLstSz,
                            const Ino::Array<double>& parLst,
                            Matrix& mat, Matrix& rhs)
{
  if (!ptLst) return false;

  if (splineClosed)
       return buildClosedBaseMat(ctrlSz,splineDegree,ptLst,ptLstSz,
                                 parLst,knotLst,mat,rhs);
  else return buildOpenBaseMat(ctrlSz,splineDegree,ptLst,ptLstSz,
                               parLst,knotLst,mat,rhs);
}

//---------------------------------------------------------------------------

bool Spline3D::buildSimpleSpline(int controlSz, const Vec3 *ptLst, int ptLstSz)
{
  if (!ptLst) return false;

  Ino::Array<double> parLst;

  if (!buildBaseKnotList(controlSz,ptLst,ptLstSz,parLst)) return false;

  controlLst.clear();

  Matrix mat(controlSz,splineDegree+1), rhs(controlSz,3);
  if (!buildBaseMat(controlSz,ptLst,ptLstSz,parLst,mat,rhs)) return false;

  mat.solveLDLT(rhs);

  controlLst.ensureCapacity(controlSz);

  for (int i=0; i<controlSz; ++i) controlLst.add(Vec3(rhs(i,0),rhs(i,1),rhs(i,2)));

  //FILE *fd = fopen("C:\\Temp\\control.csv","w");

  //for (int i=0; i<controlSz; ++i) {
  //  const Vec3& c = controlLst[i];
  //  fprintf(fd,"%d;%.6f;%.6f;%.6f\n",i,c.x,c.y,c.z);
  //}

  //fclose(fd);

  //fd = fopen("C:\\Temp\\spline.csv","w");

  //double par = 0.0;
  //Vec3 p;

  //for (int i=0; i<ptLstSz; ++i) {
  //  if (!point(par,p)) break;

  //  fprintf(fd,"%d;%.6f;%.6f;%.6f;%.6f;;%.6f;%.6f;%.6f;;%.7f;",i,par,p.x,p.y,p.z,ptLst[i].x,ptLst[i].y,ptLst[i].z,p.distTo3(ptLst[i]));

  //  if (!derivativeAt(1,par,p)) break;
  //  fprintf(fd,";%.6f;%.6f;%.6f;;%.6f;",p.x,p.y,p.z,p.len3());

  //  if (!derivativeAt(2,par,p)) break;
  //  fprintf(fd,";%.6f;%.6f;%.6f;;%.6f\n",p.x,p.y,p.z,p.len3());

  //  double nxtPar = par;

  //  if (i < ptLstSz-1) {
  //    nxtPar += ptLst[i].distTo3(ptLst[i+1]);

  //    //double mPar = (par + nxtPar)/2.0;
  //    //if (!point(mPar,p)) break;

  //    //Vec3 mPt(ptLst[i]); mPt += ptLst[i+1]; mPt /= 2.0;
  //    //fprintf(fd,"%d;%.6f;%.6f;%.6f;%.6f;;%.6f;%.6f;%.6f;;%.7f;",i,mPar,p.x,p.y,p.z,mPt.x,mPt.y,mPt.z,p.distTo3(mPt));

  //    //if (!derivativeAt(1,mPar,p)) break;
  //    //fprintf(fd,";%.6f;%.6f;%.6f;;%.6f;",p.x,p.y,p.z,p.len3());

  //    //if (!derivativeAt(2,mPar,p)) break;
  //    //fprintf(fd,";%.6f;%.6f;%.6f;;%.6f\n",p.x,p.y,p.z,p.len3());

  //    par = nxtPar;
  //  }
  //}

  //fclose(fd);

  return true;
}

//========================

class VecTable
{
  Vec3 *const rawLst;
  Vec3 * *const table;

  VecTable(const VecTable& cp);             // No copying
  VecTable& operator=(const VecTable& src); // No Assignment

public:
  VecTable(int colSz, int rowSz);
  ~VecTable();

  Vec3& operator()(int col, int row) { return table[col][row]; }
  const Vec3& operator()(int col, int row) const { return table[col][row]; }

  Vec3 *const column(int col) { return table[col]; }
  const Vec3 *const column(int col) const { return table[col]; }
};

//---------------------------------------------------------------------------

VecTable::VecTable(int colSz, int rowSz)
: rawLst(new Vec3[colSz*rowSz]),
  table(new Vec3*[colSz])
{
  Vec3 *v = rawLst;

  for (int i=0; i<colSz; ++i) {
    table[i] = v;
    v += rowSz;
  }
}

//---------------------------------------------------------------------------

VecTable::~VecTable()
{
  delete[] table;
  delete[] rawLst;
}

//---------------------------------------------------------------------------

static void buildSmooth(int degree, bool closed, int controlSz,
                        const Vec3 *ptLst, int ptLstSz,
                        double maxAvgDist, VecTable& smoothTable)
{
}

//---------------------------------------------------------------------------

static void costatini(int degree, bool closed, int controlSz,
                      Spline3D::BuildMode mode, VecTable& smootTable)
{
}

//---------------------------------------------------------------------------

static void buildSpline(int degree, bool closed, int controlSz,
                        Spline3D::BuildMode mode, VecTable& smoothTable,
                        Array<double> knotLst,
                        Array<Ino::Vec3> controlLst)
{
}

//---------------------------------------------------------------------------

bool Spline3D::build(int degree, bool closed, int controlSz,
                     double maxAvgDist, BuildMode mode,
                     const Ino::Vec3 *ptLst, int ptLstSz,
                     double& rmsDist, double& maxDist)
{
  clear();

  rmsDist = 0.0;
  maxDist = 0.0;

  if (degree < 1)
    throw IllegalArgumentException("Spline3D::build (degree < 1)");

  if (ptLstSz <= degree)
    throw IllegalArgumentException("Spline3D::build (ptLstSz <= degree)");

  if (controlSz <= degree)
    throw IllegalArgumentException("Spline3D::build (controlSz <= degree)");

  if (!ptLst) throw NullPointerException("Spline3D::build (ptLst=NULL)");
    
  if (controlSz > ptLstSz)
    throw IllegalArgumentException("Spline3D::build (controlSz > ptLstSz)");

  splineDegree = degree;
  splineClosed = closed;

#if 0
  if (degree < 2 && controlSz == ptLstSz) {
    knotLst.ensureCapacity(controlSz);
    controlLst.ensureCapacity(controlSz);

    controlLst.add(ptLst[0]);
    knotLst.add(0.0);

    double par = 0.0;

    for (int i=1; i<ptLstSz; ++i) {
      controlLst.add(ptLst[i]);

      par += ptLst[i].distTo3(ptLst[i-1]);
      knotLst.add(par);
    }

    rmsDist = 0.0;
    maxDist = 0.0;

    return true;
  }
#endif

  buildSimpleSpline(controlSz,ptLst,ptLstSz);

  //VecTable smoothTable(6,minCap);

  //buildSmooth(degree,closed,controlSz,ptLst,ptLstSz,maxAvgDist,smoothTable);

  //costatini(degree,closed,controlSz, mode,smoothTable);

  //buildSpline(degree,closed,controlSz,mode,smoothTable, knotLst, controlLst);

  return true;
}

//---------------------------------------------------------------------------
