#include "KinInterface.h"


Model* newModel(const wchar_t* name)
{
  return new Model(name);
}