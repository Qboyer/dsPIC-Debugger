debugger : web.o dspic.o main.o data.o
	g++ -o debugger web.o dspic.o data.o main.o -Wall -lwiringPi -pthread 

web.o: web.cpp web.hpp global.hpp
	g++ -o web.o -c web.cpp -Wall -lwiringPi -pthread 

dspic.o: dspic.cpp dspic.hpp global.hpp
	g++ -o dspic.o -c dspic.cpp -Wall -lwiringPi -pthread

data.o: data.cpp data.hpp
	g++ -o data.o -c data.cpp -Wall -lwiringPi -pthread
	
main.o: main.cpp web.hpp dspic.hpp
	g++ -o main.o -c main.cpp -Wall -lwiringPi -pthread 
	

