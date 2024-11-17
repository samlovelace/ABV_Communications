
#include <iostream>
#include "abv_comms/ABV_Comms.h"
#include "abv_idl/msg/abv_command.hpp"

void signalHandler(int signal)
{
    printf("recvd %d", signal); 
    exit(1); 
}

int main()
{
    std::signal(SIGINT, signalHandler); 
    // Setup the library 
    ABV_Comms::init(); 
    std::unique_ptr<ABV_Comms> abv_comms = std::make_unique<ABV_Comms>(); 
    abv_comms->spinNode(); 

    std::string cmdType = "Thruster"; 
    std::vector<float> controlInput = {0.5, 0.7, 1}; 
    int counter = 0; 
    while(true)
    {   
        abv_comms->publishCommand(cmdType, controlInput); 

        counter++; 
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }


}