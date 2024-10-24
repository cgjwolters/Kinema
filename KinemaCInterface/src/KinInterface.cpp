#include "KinInterface.h"
#include "KinModel.h"
#include "KinArcLinTrack.h"
// #include "stdio.h"

void* NewModel(const wchar_t* name) {
  //fprintf(stdout, "Cpp name: %ls\n", name);
  //fflush(stdout);

  InoKin::Model* mdl = new InoKin::Model(name);

  return mdl;
}

void* NewArcLinTrack(bool trkClosed, double trackPipeDiameter) {
  InoKin::ArcLinTrack* trk = new InoKin::ArcLinTrack(trkClosed, trackPipeDiameter);

  return trk;
}