//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- 3D arc-linear Track Interpolator -------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_ARCLIN_TRACK_INC
#define INOKIN_ARCLIN_TRACK_INC

#include "KinAbstractTrack.h"

#include "Vec.h"
#include "Trf.h"
#include "Array.h"

namespace InoKin {

//---------------------------------------------------------------------------

class ArcLinTrackPt : public Ino::Vec3, public Ino::ArrayElem
{
    ArcLinTrackPt *prv, *nxt;
    bool isArc;
    double s, minS, maxS, rad, start, end;
    Ino::Vec3 norm, center;
    Ino::Vec3& zDir;
    Ino::Trf3 ltrf;

    void calcArc();
    void getPoint(double atRel, Vec3& p) const;
    void getDir(double atRel, Vec3& v) const;
    void getAcc(double atRel, Vec3& a) const;
    void getJerk(double atRel, Vec3& j) const;

    ArcLinTrackPt(const ArcLinTrackPt& cp) = delete;             // No Copying
    ArcLinTrackPt& operator=(const ArcLinTrackPt& src) = delete; // No Assignment

  public:
    explicit ArcLinTrackPt(const Ino::Vec3& v);
    ~ArcLinTrackPt();

    void setPoint(const Ino::Vec3& v);
    const Ino::Vec3& getPoint() const { return *this; }

    void setZDir(const Ino::Vec3& zd) { zDir = zd; }
    const Ino::Vec3& getZDir() const { return zDir; }

    bool getIsArc() const { return isArc; }
    double getRad() const { return rad; }
    double getStart() const { return start; }
    double getEnd() const   { return end; }

    const Ino::Vec3& getNorm() const { return norm; }
    const Ino::Vec3& getCenter() const { return center; }

    double getS() const { return s; }
    double getMinS() const { return minS; }
    double getMaxS() const { return maxS; }

  friend class ArcLinTrack;
};

//---------------------------------------------------------------------------

class ArcLinTrack : public ::InoKin::AbstractTrack
{
  Ino::Array<ArcLinTrackPt *> trk;
  bool closed;
  double trackPipeRadius;
  double length;

  void setRelations();
  void validate();

  void findPointPair(double at_s, int& lowidx,
                              int& hghidx, double& relParm) const;
  int findUpper(double at_s) const;

  ArcLinTrack& operator=(const ArcLinTrack& src) = delete; // No assignment

public:
  explicit ArcLinTrack(bool trkClosed = false, double trackPipeDiameter = 0.0);
  explicit ArcLinTrack(const ArcLinTrack& cp);
  ~ArcLinTrack();

  int size() const { return trk.size(); }

  virtual void clear();
  virtual bool isClosed() const { return closed; }

  void setTrack(const Ino::Vec3 *ptLst, int ptSz, bool trackClosed = true,
                                               double trackPipeDiameter = 0.0);
  void setClosed(bool clsd);

  const Ino::Vec3& getPoint(int idx) const;
  const ArcLinTrackPt& getTrackPt(int idx) const;

  virtual Ino::Vec3 calcCentroid() const;
  virtual void translate(const Ino::Vec3& offset); 

  void setCoTrack(const ArcLinTrack& coTrk, bool reverseDir, double maxSDiff);

  double getLength() const { return length; }

  double getMaxS() const;

  double getPipeRadius() const { return trackPipeRadius; }   // Track pipe radius
  double getPipeDiameter() const { return trackPipeRadius * 2.0; }

  void setPipeRadius(double radius) { trackPipeRadius = radius; }
  void setPipeDiameter(double diameter) { trackPipeRadius = diameter/2.0; }

  virtual void getPoint(double at_s, Ino::Vec3& p) const;
  virtual void getPointAndDir(double at_s, Ino::Vec3& pnt, Ino::Vec3& dir) const;
  virtual void getDir(double at_s, Ino::Vec3& v) const;
  virtual void getAcc(double at_s, Ino::Vec3& acc) const;
  virtual void getJerk(double at_s, Ino::Vec3& jerk) const;

  virtual void getXDir(double at_s, Ino::Vec3& x, Ino::Vec3& xDer) const;

  double findPoint(const Ino::Vec3& p) const;
  double findPoint(const Ino::Vec3& p, Ino::Vec3& trkPt) const;
  double findPoint(const Ino::Vec3& p, double minS, double maxS,
                                                Ino::Vec3& trkPt) const;
};

} // namespace

// Import Section

extern "C" __declspec(dllexport) void* ArcLinTrackNew(bool trkClosed = false, double trackPipeDiameter = 0.0);

extern "C" __declspec(dllexport) void ArcLinTrackSetCoTrack(void *track, void *coTrack, bool reverseDir, double maxSDiff);

extern "C" __declspec(dllexport)  int ArcLinTrackGetSize(void *track);

extern "C" __declspec(dllexport)  double ArcLinTrackGetMaxS(void *track);

extern "C" __declspec(dllexport)  void ArcLinTrackClear(void *track);

extern "C" __declspec(dllexport)  bool ArcLinTrackIsClosed(void *track);

extern "C" __declspec(dllexport)  void ArcLinTrackSetClosed(void *track, bool closed);

extern "C" __declspec(dllexport)  double ArcLinTrackGetPipeRadius(void *track);

extern "C" __declspec(dllexport)  void ArcLinTrackSetPipeRadius(void *track, double r);

extern "C" __declspec(dllexport)  void ArcLinTrackGetPoint(void *track, int idx, Ino::Vec3& v);

extern "C" __declspec(dllexport)  void ArcLinTrackCalcCentroid(void *track, Ino::Vec3& centroid);

extern "C" __declspec(dllexport)  void ArcLinTrackTranslate(void *track, Ino::Vec3 offset);

extern "C" __declspec(dllexport)  double ArcLinTrackGetLength(void *track);

extern "C" __declspec(dllexport)  void ArcLinTrackGetPointAndDir(void *track, double at_s, Ino::Vec3& pnt, Ino::Vec3& dir);

extern "C" __declspec(dllexport)  void ArcLinTrackGetAccAndJerk(void *track, double at_s, Ino::Vec3& acc, Ino::Vec3& jerk);

extern "C" __declspec(dllexport)  void ArcLinTrackGetXDir(void *track, double at_s, Ino::Vec3& x, Ino::Vec3& xDir);

extern "C" __declspec(dllexport)  double ArcLinTrackFindPoint(void *track, Ino::Vec3 p, Ino::Vec3& trkPt);

extern "C" __declspec(dllexport)  double ArcLinTrackFindPoint2(void *track, Ino::Vec3 p, double minS, double maxS, Ino::Vec3& trkPt);

// End Import Section

//---------------------------------------------------------------------------
#endif
