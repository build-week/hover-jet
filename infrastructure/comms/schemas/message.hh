#pragma once

#include "message_header.hh"

namespace jet {

namespace {
const uint16_t MESSAGE_SERIZLIED_SIZE_BYTES = 26;
}

struct Message {
  MessageHeader header;

  // Temporary fix to a deserialization problem:
  // Message requires a custom deserializer to support deserialization of derived types as base
  // message types. NOP's wire format is as follows:
  // [type specifier byte] [number of fields to follow (up to four bytes)] [fields ...]
  // For a struct, the first byte will always be 0xb9. As long as we have fewer than 128 fields, the second byte
  // indicates the number of fields to deserialize.
  // A serialized derived message's second byte will represent the number of fields in the derived
  // message.
  // If you try to deserialize a derived message as a base Message type, NOP will fail to
  // deserialize properly because the number of fields in the Message struct won't match the number
  // of fields indicated in the second byte of the serialized derived message.
  //
  // Quick, temporary solution: When deserializing a message as type <Message>, force the number of
  // fields indicated in the serialized message to 1 so that NOP is tricked into interpreting the serialized
  // message as a <Message> type.
  virtual void deserialize(const std::string& data) {
    if (data[1] != 0x1) {
      std::string data_copy = data.substr(0, MESSAGE_SERIZLIED_SIZE_BYTES);
      data_copy[1] = 0x1;
      nop::Deserializer<nop::StreamReader<std::stringstream>> deserializer{data_copy};
      deserializer.Read(this);
    } else {
      nop::Deserializer<nop::StreamReader<std::stringstream>> deserializer{data};
      deserializer.Read(this);
    }
  }

  virtual void serialize(std::string& data) {
    using Writer = nop::StreamWriter<std::stringstream>;
    nop::Serializer<Writer> serializer;
    serializer.Write(*this);
    data = serializer.writer().take().str();
  }

  NOP_STRUCTURE(Message, header);
};

}  //  namespace jet
