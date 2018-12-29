#pragma once

#define MESSAGE(type, ...)                                                      \
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
  NOP_STRUCTURE(type, header, __VA_ARGS__)
