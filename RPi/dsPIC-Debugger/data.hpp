#ifndef DATA_H
#define DATA_H

#include <queue>
#include <mutex>

struct point{
  uint8_t id;
  uint32_t x;
  int32_t y;
};

class Data
{
    public:
        Data();
        virtual ~Data();
		
		void addPlot(point p);
        void clearPlots();
        std::queue<point> getPlots();
        bool isUpdatedPlots();
		
    protected:
        std::mutex m_mutex;
		
		std::queue<point> plots;
		bool updatedPlots = false;
		
    private:
};

#endif // DATA_H