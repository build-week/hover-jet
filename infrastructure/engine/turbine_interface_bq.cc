//%bin(turbine_interface_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/engine/turbine_interface_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <unistd.h>
#include <iostream>

namespace jet {

TurbineInterfaceBQ::TurbineInterfaceBQ() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void TurbineInterfaceBQ::init(int argc, char *argv[]) {
  turbine_ptr_ = std::make_unique<JetCatTurbine>();
  bool success = turbine_ptr_->start_engine();
  if (!success) {
    std::cerr << "Failed to start engine!" << std::endl;
  } else {
    std::cout << "Successful engine start." << std::endl;
  }

  usleep(1000000);
  std::optional<uint32_t> rpm = turbine_ptr_->get_RPM();
  if (!rpm) {
    std::cerr << "Could not read RPM from turbine." << std::endl;
  } else {
    std::cout << "Turbine speed is " << *rpm << " RPM" << std::endl;
  }

  usleep(1000000);
  success = turbine_ptr_->stop_engine();
  if (!success) {
    std::cerr << "Failed to stop engine!" << std::endl;
  } else {
    std::cout << "Successful engine stop." << std::endl;
  }
}

void TurbineInterfaceBQ::loop() {
  std::optional<uint32_t> rpm = turbine_ptr_->get_RPM();
  if (!rpm) {
    std::cerr << "Could not read RPM from turbine." << std::endl;
  } else {
    std::cout << "Turbine speed is " << *rpm << " RPM" << std::endl;
  }

  std::optional<JetCat::FuelInfo> fuel_info = turbine_ptr_->get_fuel_info();
  if (!fuel_info) {
    std::cerr << "Could not read fuel flow from turbine." << std::endl;
  } else {
    std::cout << "Fuel flow is " << fuel_info->actual_fuel_flow << std::endl;
  }

  std::optional<JetCat::TurbineInfo> turbine_info = turbine_ptr_->get_turbine_info();
  if (!turbine_info) {
    std::cerr << "Could not read turbine info." << std::endl;
  } else {
    std::cout << "Turbine type is: " << turbine_info->turbine_type << std::endl;
  }
}

void TurbineInterfaceBQ::shutdown() {
  bool success = turbine_ptr_->stop_engine();
  if (!success) {
    std::cerr << "Failed to stop engine!" << std::endl;
  } else {
    std::cout << "Successful engine stop." << std::endl;
  }
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::TurbineInterfaceBQ)
