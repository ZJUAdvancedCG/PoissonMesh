// Hash class for STL containers
#pragma once
#include "hash.h"

struct Hasher {
  template<class T> size_t operator()(T const& x) const {
    return geode::hash(x);
  }

  bool operator==(Hasher h) const {
    return true;
  }

  bool operator!=(Hasher h) const {
    return false;
  }
};
