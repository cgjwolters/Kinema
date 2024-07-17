//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Kinema: Kinematic Simulation Program ---------------------------
//---------------------------------------------------------------------------
//-------------------- Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//-------------------------------------------------- C.Wolters --------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------- Abstract function ----------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

#ifndef INOKIN_FUNCTION_INC
#define INOKIN_FUNCTION_INC

#include "KinObject.h"
#include "KinObjList.h"

namespace InoKin {

//---------------------------------------------------------------------------

class Grip;
class TableFunction;

class Function : public Object
{
  Function(const Function& cp) = delete;             // No copying
  Function& operator=(const Function& src) = delete; // No assignment

public:
  Grip& output;
  const long outputIdx;

  Object& input;
  const TableFunction& fctTable;

  Function(const wchar_t *name, Grip& out_grp, int out_idx,
                                   Object& in_obj, const TableFunction& table);
  virtual ~Function();

  void updatePos() const;
  void updateDer() const;
  void updateSecDer() const;

  virtual double getInput() const = 0;
  virtual double getInputDerivative() const = 0;
  virtual double getInputSecDerivative() const = 0;
};

//---------------------------------------------------------------------------

class FunctionList : public ObjList<Function>
{
  FunctionList(const FunctionList& cp);
  FunctionList& operator=(const FunctionList& src);

public:
  FunctionList(bool owner=true) : ObjList<Function>(owner) {}

  Function *findByConnection(const Object& obj) const;

  void updateAllPos() const;
  void updateAllDer() const;
  void updateAllSecDer() const;
};

} // namespace

//---------------------------------------------------------------------------
#endif
