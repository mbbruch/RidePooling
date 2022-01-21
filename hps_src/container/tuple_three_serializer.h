#ifndef JL2922_HPS_TUPLETHREE_SERIALIZER_H
#define JL2922_HPS_TUPLETHREE_SERIALIZER_H

#include <iostream>
#include <utility>
#include "../basic_type/uint_serializer.h"
#include "../serializer.h"

namespace hps {

template <class T1, class B>
class Serializer<std::tuple<T1, T1, T1>, B> {
 public:
  static void serialize(const std::tuple<T1, T1, T1>& container, B& ob) {
    Serializer<T1, B>::serialize(std::get<0>(container), ob);
    Serializer<T1, B>::serialize(std::get<1>(container), ob);
    Serializer<T1, B>::serialize(std::get<2>(container), ob);
  }

  static void parse(std::tuple<T1, T1, T1>& container, B& ib) {
    Serializer<T1, B>::parse(std::get<0>(container), ib);
    Serializer<T1, B>::parse(std::get<1>(container), ib);
    Serializer<T1, B>::parse(std::get<2>(container), ib);
  }
};

}  // namespace hps
#endif
