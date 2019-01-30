#pragma once

#include <string>

namespace jet {

class GoNoGo {
  public: 
   GoNoGo();
   void go(std::string message);
   void nogo(std::string message);
   bool isReady();
   std::string getMessage();

  private: 
   bool ready;
   std::string message;
};

} // namespace jet