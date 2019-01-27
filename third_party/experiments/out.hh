#pragma once

#include <type_traits>
#include <vector>

//
// Retains no ownership over the object
//
template <class T>
struct Out {
  explicit Out(T& obj) : _obj(obj) {
  }
  T& operator*() {
    return _obj;
  }
  T* operator->() {
    return &_obj;
  }

 private:
  T& _obj;
};

template <class T>
Out<T> out(T& obj) {
  return Out<T>(obj);
}

//
// Retains no ownership over the object
//
template <typename T>
struct Vout {
  explicit Vout(T& obj) : _obj(obj) {
  }

  T& operator*() {
    return _obj;
  }

  T* operator->() {
    return &_obj;
  }

  const T& operator*() const {
    return _obj;
  }

  typename T::value_type& operator[](int i) {
    return _obj[i];
  }

 private:
  T& _obj;
};

template <typename T>
Vout<T> vout(T& obj) {
  return Vout<T>(obj);
}

template <typename T>
T& is_out(T& arg) {
  return arg;
}
