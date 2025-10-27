CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

# Direct include paths since pkg-config has issues with OMPL's Eigen3 dependency
INCLUDE_FLAGS=-I/usr/local/include/ompl-1.7 -I/usr/include/eigen3
# Linker options
LD_FLAGS=-L/usr/local/lib -lompl -lboost_system -lboost_filesystem -lboost_serialization

# The c++ compiler to invoke
CXX=c++
all: Project4Car Project4Pendulum

clean:
	rm -f *.o
	rm -f Project4Car Project4Pendulum

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

Project4Pendulum: Project4Pendulum.o RG-RRT.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)

Project4Car: Project4Car.o RG-RRT.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)
