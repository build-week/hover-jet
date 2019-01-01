#pragma once

#include "third_party/nop/serializer.h"
#include "third_party/nop/structure.h"
#include "third_party/nop/utility/stream_reader.h"
#include "third_party/nop/utility/stream_writer.h"

#include <iostream>
#include <sstream>

#define MESSAGE(type, ...) SERIALIZABLE_STRUCTURE(type, header, __VA_ARGS__)

#define SERIALIZABLE_STRUCTURE(type, ...)                                       \
  virtual void deserialize(const std::string& data) {                           \
    nop::Deserializer<nop::StreamReader<std::stringstream>> deserializer{data}; \
    deserializer.Read(this);                                                    \
  }                                                                             \
  virtual void serialize(std::string& data) {                                   \
    using Writer = nop::StreamWriter<std::stringstream>;                        \
    nop::Serializer<Writer> serializer;                                         \
    serializer.Write(*this);                                                    \
    data = serializer.writer().take().str();                                    \
  }                                                                             \
  NOP_STRUCTURE(type, __VA_ARGS__)
