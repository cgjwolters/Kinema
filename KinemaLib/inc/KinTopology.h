//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Topology of bodies and grips -----------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_TOPOLOGY_INC
#define INOKIN_TOPOLOGY_INC

#include "KinGrip.h"
#include "KinSequence.h"

#include "Array.h"

namespace Ino {
  class Trf3;
  class Matrix;
  class Vector;
}

namespace InoKin {

class Model;
class Body;
class BodyList;
class AbstractJoint;

//---------------------------------------------------------------------------

class Topology : public LoopList // A List of Grip loops
{
  Model *model;

  BodyList& topoBodyLst;
  GripList& topoGripLst;

  int rowSz;
  int colSz;
  int varSz;
  int fixedSz;

  bool *angularVar;

  bool posValid, speedValid, accelValid, jerkValid;

  SequenceList seqLst;

  Ino::Matrix& prepMat;
  Ino::Matrix& solMat;
  Ino::Vector& rhs;

  Ino::Matrix solMat2;
  Ino::Matrix speedMat2;
  Ino::Vector solRhs2;
  Ino::Vector speedRhs2;

  void updateJointTransforms();
  void loopScan(LoopList& loopLst, int idx);
  void analyzeLoops();
  void bodyScan(int idx);
  void buildLoop(Grip *grp);
  void assignVarIndices(LoopList& loopLst);
  void setLoopCounts();
  void sizeMats();

  bool composePosMatrixRow(const GripList& grpLst,
                           double& maxRot, double& maxDist);
  bool composePosEq(double& maxRot, double& maxDist, int& maxIdx);
  void limitSolution(Ino::Vector& sol);

  bool composeSpeedMatrixRow(const GripList& grpLst);
  bool composeSpeedEq();

  bool composeAccelMatrixRow(const GripList& grpLst);
  bool composeAccelEq();

  bool composeJerkMatrixRow(const GripList& grpLst);
  bool composeJerkEq();

  explicit Topology(Model& mdl, const Topology& cp, bool withSequences=false);

  Topology(const Topology& cp) = delete;             // No copying
  Topology& operator=(const Topology& src) = delete; // No assignment

public:
  explicit Topology(Model& mdl);
  ~Topology();

  void clear();
  bool isEmpty() const { return size() < 1; }

  int getVarSz() const   { return varSz; }
  int getFixedSz() const { return fixedSz; }
  int getRowSz() const   { return rowSz; }
  int getColSz() const   { return colSz; }

  Model *getModel() const { return model; }
  const BodyList& getBodyList() const { return topoBodyLst; }
  const GripList& getGripList() const { return topoGripLst; }

  void prepare(Body *fstBody);

  const SequenceList& getSeqLst() const { return seqLst; }
  Sequence& newSequence(const wchar_t *name);

  bool solvePos(int maxIter, double rotTol, double posTol,
                                            Ino::Vector& varPosVec, int& iter);

  const Ino::Matrix& getPosMat() const { return solMat2; }
  const Ino::Matrix& getSpeedMat() const { return speedMat2; }
  const Ino::Vector& getSolRhs() const { return solRhs2; }
  const Ino::Vector& getSpeedRhs() const { return speedRhs2; }

  bool getPosValid() const { return posValid; }
  bool getSpeedValid() const { return speedValid; }
  bool getAccelValid() const { return accelValid; }
  bool getJerkValid() const { return jerkValid; }

  bool getPosVector(Ino::Vector& posVec, bool fixed=false) const;
  void setPosVector(const Ino::Vector& posVec, bool fixed=false);
  void setPosVectors(const Ino::Vector& varPosVec,
                                 const Ino::Vector& fixedPosVec);
  bool updatePositions() const;

  // Must have called solvePos first:
  bool solveSpeed(Ino::Vector& speedVec, bool write = false);

  bool getSpeedVector(Ino::Vector& posVec, bool fixed=false) const;
  void setSpeedVector(const Ino::Vector& posVec, bool fixed=false);
  void setSpeedVectors(const Ino::Vector& varSpeedVec,
                                 const Ino::Vector& fixedSpeedVec);

  // Must have called updatePositions first!
  bool updateSpeeds();

  // Must have called solvePos and solveSpeed first:
  bool solveAccel(Ino::Vector& accelVec);

  bool getAccelVector(Ino::Vector& accelVec, bool fixed=false) const;
  void setAccelVector(const Ino::Vector& accelVec, bool fixed=false);
  void setAccelVectors(const Ino::Vector& varAccelVec,
                                 const Ino::Vector& fixedAccelVec);

  // Must have called updatePositions & updateSpeeds first!
  bool updateAccels();

  // Must have called solvePos, solveSpeed and solveAccel first:
  bool solveJerk(Ino::Vector& jerkVec);

  bool getJerkVector(Ino::Vector& jerkVec, bool fixed=false) const;
  void setJerkVector(const Ino::Vector& jerkVec, bool fixed=false);
  void setJerkVectors(const Ino::Vector& varJerkVec,
                                 const Ino::Vector& fixedJerkVec);

  // Must have called updatePositions,updateSpeeds and updateAccels first!
  bool updateJerks();

  void transform(const Ino::Trf3& trf) const;

  friend class Model;
};

//---------------------------------------------------------------------------

class TopologyList : public Ino::Array<Topology *>
{
  Model *model;

  TopologyList(const TopologyList& cp);             // No copying
  TopologyList& operator=(const TopologyList& src); // No assignment

public:
  TopologyList() : Ino::Array<Topology *>(true), model(NULL) {}

  Model *getModel() const { return model; }
  void setModel(Model& mdl) { model = &mdl; }

  bool isEmpty() const { return size() < 1; }

  bool prepareAll();

  void transform(const Ino::Trf3& trf) const;
};

} // namespace

//---------------------------------------------------------------------------
#endif
