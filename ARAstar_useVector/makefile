exe: main.o ARAstar_algorithm.o grid_input.o
	g++ -o exe.out -std=c++11 main.o ARAstar_algorithm.o grid_input.o
	
main.o: main.cpp
	g++ -c main.cpp -std=c++11

ARAstar_algorithm.o: include/ARAstar_algorithm.cpp
	g++ -c include/ARAstar_algorithm.cpp -std=c++11

grid_input.o: include/grid_input.cpp
	g++ -c include/grid_input.cpp

clean:
	rm -f *.o
