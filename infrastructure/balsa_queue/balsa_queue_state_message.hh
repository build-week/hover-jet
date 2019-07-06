#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/time/duration.hh"

#include <string>

namespace jet {

struct ExecutionTime {
  Duration max_time;
  Duration min_time;
  Duration median_time;

  SERIALIZABLE_STRUCTURE(ExecutionTime, max_time, min_time, median_time);
};

struct BQStateMessage : Message {
  std::string bq_instance_name;
  ExecutionTime loop_execution_times;

  MESSAGE(BQStateMessage, bq_instance_name, loop_execution_times);
};

}  //  namespace jet
