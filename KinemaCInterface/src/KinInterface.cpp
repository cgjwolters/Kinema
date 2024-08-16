#include "KinInterface.h"


InoKin::Model* newModel(const wchar_t* name)
{
  return new InoKin::Model(name);
}