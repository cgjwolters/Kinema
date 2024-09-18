#include "KinInterface.h"
#include "KinModel.h"

using namespace InoKin;

//void* newModel(const wchar_t* name)
void* NewModel()
{
  return (void*)new InoKin::Model();
}