/*******************************************************************************
 * 
 *      File:    main.cpp
 *      
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Application entry point
 *
 *******************************************************************************
 */

#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>

#include "Driver.h"

static Driver theDriver;

void exitHandler(int signum)
{
    std::cout << "\n[main] Received signal: " << signum << std::endl << std::flush;
    if (theDriver.IsStopping() == false)
    {
        theDriver.Stop();
        std::cout << "[main] Stop process started, Please wait a moment ..." << std::endl;
    }
    else
        std::cout << "[main] Stop process already started" << std::endl;
}

sint32 setSignals()
{
    signal(SIGINT, exitHandler);
    signal(SIGHUP, exitHandler);
    signal(SIGTERM, exitHandler);
    signal(SIGQUIT, exitHandler);

    // Ignore broken pipe
    signal(SIGPIPE, SIG_IGN);
    return 0;
}

void showUsage(const char* programName)
{
    std::cout << "\nUsage: " << programName << " [options] driverIndex" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  -d<level>   Debug level (0-3, default 0)" << std::endl;
    std::cout << "  -a<addr>    Escalator address (can be specified multiple times)" << std::endl;
    std::cout << "  -h          Show this help message" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << programName << " -d1 -a100 -a101 0" << std::endl << std::endl;
}

int main(int argc, char *argv[])
{
    setSignals();

    std::cout << "========================================" << std::endl;
    std::cout << "escDriver_to_MQTT Application" << std::endl;
    std::cout << "========================================" << std::endl;

    uint32 debugLevel = 0;
    bool argsOK = true;
    uint32 driverIndex = 0;
    UIntData_t addresses;

    if (argc <= 1)
        argsOK = false;
    else
    {
        uint32 numArgs = static_cast<uint32>(argc);
        for (uint32 index = 1; index < numArgs && argsOK; ++index)
        {
            std::string str = argv[index];

            if (str == "-h" || str == "--help")
            {
                showUsage(argv[0]);
                return 0;
            }
            else if (str.length() >= 2 && str[0] == '-' && str[1] == 'd')
            {
                std::stringstream ss(argv[index]);
                ss.ignore(2);
                ss >> debugLevel;
                if (debugLevel > 3) {
                    std::cerr << "ERROR: Debug level must be 0-3" << std::endl;
                    argsOK = false;
                }
            }
            else if (str.length() >= 2 && str[0] == '-' && str[1] == 'a')
            {
                uint32 id;
                std::stringstream ss(argv[index]);
                ss.ignore(2);
                ss >> id;
                addresses.push_back(id);
            }
            else if (index == numArgs - 1)
            {
                // Last argument is the driver index
                std::stringstream ss(argv[index]);
                ss >> driverIndex;
            }
            else
            {
                argsOK = false;
            }
        }
    }

    if (!argsOK)
    {
        showUsage(argv[0]);
        return 1;
    }

    std::cout << "[main] Initializing driver..." << std::endl;
    std::cout << "       Driver Index: " << driverIndex << std::endl;
    std::cout << "       Debug Level: " << debugLevel << std::endl;
    std::cout << "       Number of addresses: " << addresses.size() << std::endl;

    if (!theDriver.Setup(driverIndex, debugLevel, addresses))
    {
        std::cerr << "[main] ERROR: Driver Setup failed" << std::endl;
        theDriver.Stop();
        return 1;
    }

    if (!theDriver.Start())
    {
        std::cerr << "[main] ERROR: Driver Start failed" << std::endl;
        theDriver.Stop();
        return 1;
    }

    std::cout << "[main] Driver running, press Ctrl+C to stop..." << std::endl;

    // Main processing loop
    while (!theDriver.IsStopping())
    {
        theDriver.ProcessData();
    }

    std::cout << "[main] Application terminated" << std::endl;
    return 0;
}
