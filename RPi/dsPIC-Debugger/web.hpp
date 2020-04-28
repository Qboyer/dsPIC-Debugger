#ifndef WEB_H
#define WEB_H

#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <string.h>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include "dspic.hpp"

#include "global.hpp"
#include "data.hpp"

#define DEBUG_PID	0

struct pointFloat2d{
  float x;
  float y;
};
/*struct s_debugValue{
  uint8_t id;
  uint8_t value;
};
struct s_debugName{
  uint8_t id;
  std::string name;
};*/
void* thread_HandleConnnection(void *threadid);
std::string simulateResponse(double i);
class Web
{
    public:
        Web(DsPIC *ds, Data *p_data);
        virtual ~Web();
        bool acceptClient();
		void closeClient();
        bool sendMsg(std::string message);
		std::string receiveMsg();
        bool startThread();
        void stopThread();
        bool isContinueThread();
		
        std::string s;
		DsPIC *dspic;
		bool waitingResponsePID = false;
		
		void addLidarPoints(float x, float y);
		void addLidarPoints(pointFloat2d fp);
		void addLidarPoints(std::vector<pointFloat2d> vect_fp);
		void clearLidarPoints();
		bool m_clearLidarPoints = false;
		bool m_radarScan = true;
		std::queue<pointFloat2d> lidarPoints;
        
        
        std::queue<s_debugValue> q_DebugValue;
        std::queue<s_debugName> q_DebugName;
        
        void updateDebugValue(uint8_t id,uint32_t value);
        void updateDebugName(uint8_t id,std::string name);
		
		void addPlot(point p);
        void clearPlots();
        std::queue<point> getPlots();
        bool isUpdatedPlots();
		
    protected:
        int socket_listen;
        int socket_client;
        pthread_t threads;
		bool m_continueThread;
		std::mutex m_mutex;
		Data *m_p_data;
        
    private:
};

std::string realResponse(Web *w);

#endif // WEB_H
