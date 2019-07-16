exe: main.o ARAstar_algorithm.o grid_input.o
	g++ -o exe.out main.o ARAstar_algorithm.o grid_input.o

main.o: main.cpp
	g++ -c main.cpp -std=c++11

ARAstar_algorithm.o: ARAstar_algorithm.cpp
	g++ -c ARAstar_algorithm.cpp -std=c++11

grid_input.o: grid_input.cpp
	g++ -c grid_input.cpp

clean:
	rm -f *.o
