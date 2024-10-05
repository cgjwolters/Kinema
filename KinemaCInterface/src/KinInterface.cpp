#include "KinInterface.h"
#include "KinModel.h"

#include "stdio.h"

using namespace InoKin;

void* NewModel(const wchar_t* name)
{
  fprintf(stdout,"Cpp name: %ls\n", name);
  fflush(stdout);

  return (void*)new InoKin::Model(name);
}