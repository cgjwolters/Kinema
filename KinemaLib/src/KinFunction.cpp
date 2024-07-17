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

#include "KinFunction.h"

#include "KinGrip.h"
#include "KinAbstractJoint.h"
#include "KinModel.h"

#include "KinTableFunction.h"

using namespace Ino;

namespace InoKin {

//---------------------------------------------------------------------------

Function::Function(const wchar_t *name, Grip& outGrp, int outIdx,
                                   Object& in_obj, const TableFunction& table)
: Object(outGrp.model,name),
  output(outGrp), outputIdx(outIdx),
  input(in_obj), fctTable(table)
{
  model.adopt(this);
}

//---------------------------------------------------------------------------

Function::~Function()
{
  model.remove(this);
}

//---------------------------------------------------------------------------

void Function::updatePos() const
{
  AbstractJoint *jnt = output.getJoint();
  if (!jnt || outputIdx < 0 || outputIdx >= jnt->getVarCnt()) return;

  double x = getInput();
  double y = fctTable.getValueAt(x);

  jnt->setVal(outputIdx,y);
}

//---------------------------------------------------------------------------

void Function::updateDer() const
{
  AbstractJoint *jnt = output.getJoint();
  if (!jnt || outputIdx < 0 || outputIdx >= jnt->getVarCnt()) return;

  double x    = getInput();
  double xder = getInputDerivative();

  double y = fctTable.getDerivativeAt(x) * xder;

  jnt->setSpeed(outputIdx,y);
}

//---------------------------------------------------------------------------

void Function::updateSecDer() const
{
  AbstractJoint *jnt = output.getJoint();
  if (!jnt || outputIdx < 0 || outputIdx >= jnt->getVarCnt()) return;

  double x    = getInput();
  double xder = getInputDerivative();
  double xsec = getInputSecDerivative();

  double y = fctTable.getSecDerivativeAt(x) * sqr(xder);
         y += (fctTable.getDerivativeAt(x) * xsec);

  jnt->setAccel(outputIdx,y);
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

Function *FunctionList::findByConnection(const Object& obj) const
{
  int fnc_cnt = size();

  for (int i=0; i<fnc_cnt; i++) {
    Function *func = operator[](i);
    if (&func->output == &obj) return func;
    if (&func->input  == &obj) return func;
  }

  return NULL;
}

//---------------------------------------------------------------------------

void FunctionList::updateAllPos() const
{
  int func_cnt = size();

  for (int i=0; i<func_cnt; i++) {
    Function *func = operator[](i);
    func->updatePos();
  }
}

//---------------------------------------------------------------------------

void FunctionList::updateAllDer() const
{
  int func_cnt = size();

  for (int i=0; i<func_cnt; i++) operator[](i)->updateDer();
}

//---------------------------------------------------------------------------

void FunctionList::updateAllSecDer() const
{
  int func_cnt = size();

  for (int i=0; i<func_cnt; i++) operator[](i)->updateSecDer();
}

} // namespace

//---------------------------------------------------------------------------
