#include "KinInterface.h"
#include "KinModel.h"

#include "stdio.h"

using namespace InoKin;

void* NewModel(const wchar_t* name)
{
  fprintf(stdout,"Cpp name: %ls\n", name);
  fflush(stdout);

  InoKin::Model *mdl = new InoKin::Model(name);

  return mdl;
}