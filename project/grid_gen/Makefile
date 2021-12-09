APP_NAME=generategrid
OBJS := main.o generategrid.o

CXX = mpic++
CXXFLAGS = -I. -O3 #-Wall -Wextra

default: $(APP_NAME) 

$(APP_NAME): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) 

%.o: %.cpp
	$(CXX) $< $(CXXFLAGS) -c -o $@ 

clean:
	/bin/rm -rf *~ *.o $(APP_NAME) *.class
