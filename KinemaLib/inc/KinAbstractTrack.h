//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Abstract 3D Track Interpolator ---------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_ABSTRACTTRACK_INC
#define INOKIN_ABSTRACTTRACK_INC

//---------------------------------------------------------------------------

namespace Ino
{
  class Vec3;
  class Trf3;
}

namespace InoKin {

class AbstractTrack
{
  AbstractTrack& operator=(const AbstractTrack& src) = delete; // No assignment

public: 
  explicit AbstractTrack();
  explicit AbstractTrack(const AbstractTrack& cp);
  virtual ~AbstractTrack() {}

  virtual void clear() = 0;

  // virtual void reverse() = 0;

  virtual bool isClosed() const = 0;

  virtual double getLength() const = 0;
  virtual double getMaxS() const = 0;

  virtual double getPipeRadius() const = 0;   // Track pipe radius
  virtual double getPipeDiameter() const = 0; // Track pipe diameter

  virtual void setPipeRadius(double radius) = 0;
  virtual void setPipeDiameter(double diameter) = 0;

  virtual Ino::Vec3 calcCentroid() const = 0;
  virtual void translate(const Ino::Vec3& offset) = 0;

  virtual void getPoint(double at_s, Ino::Vec3& p) const = 0;
  virtual void getPointAndDir(double at_s, Ino::Vec3& pnt, Ino::Vec3& dir) const = 0;
  virtual void getDir(double at_s, Ino::Vec3& dir) const = 0;
  virtual void getAcc(double at_s, Ino::Vec3& acc) const = 0;
  virtual void getJerk(double at_s, Ino::Vec3& jerk) const = 0;

  virtual void getXDir(double at_s, Ino::Vec3& x, Ino::Vec3& xDer) const = 0;

  virtual double findPoint(const Ino::Vec3& p) const = 0;
};

} // namespace

//---------------------------------------------------------------------------
#endif
