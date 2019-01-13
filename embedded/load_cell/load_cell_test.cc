#include "embedded/load_cell/load_cell.hh"

int main()
{
    auto jet_receiver = jet::LoadCellReceiver();
    
    while(1)
    {
        jet_receiver.receive();
    }  
    
    return 0;
}
