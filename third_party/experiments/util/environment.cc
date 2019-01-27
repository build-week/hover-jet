#include "util/environment.hh"

namespace jcc {

std::string Environment::asset_path() {
  return std::string(ASSET_PATH) + "/";
}

}  // namespace jcc
