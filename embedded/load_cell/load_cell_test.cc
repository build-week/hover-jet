#include "embedded/load_cell/load_cell.hh"

#include <cassert>

int main(int argc, char *argv[]) {
  assert(argc == 2);
  auto jet_receiver = jet::LoadCellReceiver(argv[1]);

  while (1) {
    jet_receiver.receive();
  }

  return 0;
}
