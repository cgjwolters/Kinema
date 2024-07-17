//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------- Kinema: Kinematic Simulation Program ---------------------------
//-------------------------------------------------------------------------------
//------------------------ Copyright Inofor Hoek Aut BV Dec 1999-2013 -----------
//------------------------------------------------------ C.Wolters --------------
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------- List of Object derived from AbstractJoint ----------------------
//-------------------------------------------------------------------------------

#ifndef KINOBJLIST_INC
#define KINOBJLIST_INC

#include "Array.h"

namespace InoKin {

//---------------------------------------------------------------------------

template <class T> class ObjList : public Ino::Array<T *>
{
protected:
  ObjList& operator=(const ObjList& src);

public:
  explicit ObjList(bool owner=false) : Ino::Array<T *>(owner) {}
  explicit ObjList(const ObjList& cp) : Ino::Array<T *>(cp) {}
  ~ObjList() {}

  using Array::operator[];

  T *operator[](const wchar_t *name) const;

  int find(const T *obj) const;
  bool remove(const T *obj);

  void reverseListOrder();

  void setIds() const;

  T *byId(int id) const;
};

//-------------------------------------------------------------------------------
// Template instantiations

template <class T> ObjList<T>& ObjList<T>::operator=(const ObjList<T>& src)
{
  Array<T *>::operator =(src);

  return *this;
}

//-------------------------------------------------------------------------------

template <class T> T *ObjList<T>::operator[](const wchar_t *name) const
{
  int sz = size();

  for (int i=0; i<sz; ++i) {
    T *t = get(i);
    Object* obj = dynamic_cast<Object *>(t);
    if (!obj) return NULL;

    if (!compareStr(name,obj->getName())) return t;
  }

  return NULL;
}

//---------------------------------------------------------------------------

template <class T> int ObjList<T>::find(const T *obj) const
{
  if (!obj) return size();

  int sz = size();

  for (int i=0; i<sz; ++i) {
    if (get(i) == obj) return i;
  }

  return size();
}

//---------------------------------------------------------------------------

template <class T> bool ObjList<T>::remove(const T *obj)
{
  int idx = find(obj);

  if (idx >= size()) return false;

  Ino::Array<T *>::remove(idx);

  return true;
}

//---------------------------------------------------------------------------

template <class T> void ObjList<T>::reverseListOrder()
{
  int lwb = 0; upb = size()-1;

  while (lwb < upb) {
    T *t = get(lwb);
    set(lwb,get(upb));
    set(upb,t);

    ++lwb;
    --upb;
  }
}

//---------------------------------------------------------------------------

template <class T> void ObjList<T>::setIds() const
{
  if (size() < 1) return;

  if (!dynamic_cast<Object *>(get(0))) 
                   throw WrongTypeException("ObjList<T>::byId");

  int sz = size();

  for (int i=0; i<sz; ++i) {
    const Object *obj = (const Object *)get(i);

    if (obj) obj->setId(i);
  }
}

//---------------------------------------------------------------------------

template <class T> T *ObjList<T>::byId(int id) const
{
  if (size() < 1) throw IndexOutOfBoundsException("ObjList<T>::byId");
  if (!dynamic_cast<Object *>(get(0))) 
                   throw WrongTypeException("ObjList<T>::byId");

  if (id >= 0 && id < size()) { // Give it a try
    Object *obj = get(id);

    if (obj->getId() == id) return (T *)obj;
  }

  int lwb = 0, upb = size()-1;

  while (lwb <= upb) {
    int idx = (lwb + upb)/2;

    Object *obj = get(idx);

    int oid = obj->getId();

    if (oid == id) return (T *)obj;
    
    if (oid < id) lwb = idx + 1;
    else upb = idx - 1;
  }

  return NULL;
}

} // namespace

//-------------------------------------------------------------------------------
#endif

