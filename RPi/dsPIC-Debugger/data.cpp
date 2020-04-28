#include "data.hpp"

Data::Data(){
}
Data::~Data(){
}
void Data::addPlot(point p){
    m_mutex.lock();
	plots.push(p);
	updatedPlots = true;
	m_mutex.unlock();
}
void Data::clearPlots(){
    m_mutex.lock();
	std::queue<point> empty;
    std::swap( plots, empty );
	m_mutex.unlock();
}
std::queue<point> Data::getPlots(){
    m_mutex.lock();
    std::queue<point> plots = this->plots;
    updatedPlots = false;
	m_mutex.unlock();
    return plots;
}
bool Data::isUpdatedPlots(){
    m_mutex.lock();
	bool b = this->updatedPlots;
	m_mutex.unlock();
	return b;
}