#pragma once

#include "message_header.hh"

#include "third_party/nop/serializer.h"
#include "third_party/nop/structure.h"
#include "third_party/nop/utility/stream_reader.h"
#include "third_party/nop/utility/stream_writer.h"

#include <iostream>
#include <sstream>

struct Message {
  MessageHeader header;

  MESSAGE(Message);
};
