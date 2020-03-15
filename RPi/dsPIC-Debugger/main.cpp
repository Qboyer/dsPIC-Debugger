#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <vector>
#include <queue>
#include "web.hpp"
#include "dspic.hpp"

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <fstream>
#include <sstream>

#define DEBUG_ENABLE_PRINT			1

#if(DEBUG_ENABLE_PRINT)
#define DEBUG_PRINT(x) 				std::cout << "DEBUG_PRINT>> " << x << std::endl;
#else
#define DEBUG_PRINT(x)				
#endif

int main()
{
	std::cout << std::endl << " dsPIC-Debugger " << std::endl << std::endl;
    std::cout << "Initialisation..." << std::endl;
    wiringPiSetup();
    DsPIC dspic;
    dspic.reset();
    delay(250); //wait for reset
    dspic.async_read(); //flush rx buffer

    Web web(&dspic);

    puts("Hello human ! I, your fervent robot, am initialised. Press <ENTER> to continue.");
	
    dspic.startThreadReception();
    web.startThread();
    //delay(500);
    dspic.setVar8(CODE_VAR_VERBOSE,1);
    puts("verbose set to 1");
    puts("Press <ENTER> to activate UI32 debug.");
    getchar();
    dspic.configureDebugVar(0, ID_VAR_TEST_UI32, VAR_32b, 1, 250, 0, 0);
    puts("Press <ENTER> to deactivate UI32 debug.");
    getchar();
    dspic.configureDebugVar(0, ID_VAR_TEST_UI32, VAR_32b, 0, 250, 0, 0);
    puts("Press <ENTER> to activate UI8 debug.");
    getchar();
    dspic.configureDebugVar(1, ID_VAR_TEST_UI8, VAR_8b, 1, 500, 0, 0);
    puts("Press <ENTER> to shut up.");
    getchar();
	dspic.setVar8(CODE_VAR_VERBOSE,0);
    puts("Press <ENTER> to exit.");
    getchar();
    dspic.stopThreadReception();
	puts("verbose set to 0");
    web.stopThread();

    puts("exiting...");
	delay(200);
	
    return 0;
}