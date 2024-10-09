//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Basic Abstract Object class ------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_ABSTRACTOBJ_INC
#define INOKIN_ABSTRACTOBJ_INC

#include "Array.h"

namespace InoKin {

//---------------------------------------------------------------------------

class Model;

class Object : public Ino::ArrayElem
{
  wchar_t *nam;
  mutable int id;

  Object(const Object& cp) = delete;             // No copying
  Object& operator=(const Object& src) = delete; // No Assignment

public:
  Model& model;

  explicit Object(Model& kmodel, const wchar_t *name);
  explicit Object(Model& kmodel, const Object& cp);
  virtual ~Object();

  const wchar_t *getName() const { return nam; }
  void setName(const wchar_t *newName);

  int getId() const { return id; }
  void setId(int newId) const { id = newId; }

  void setModelTopoModified() const;
  void setModelModified() const;
};

} // namespace

//---------------------------------------------------------------------------
#endif
