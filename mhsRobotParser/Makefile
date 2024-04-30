CXX = g++
TARGET = test
OBJ = main.o getToken.o SyntaxAnalysis.o

$(TARGET): $(OBJ)
	$(CXX) -o $(TARGET) $(OBJ)

main.o: main.cpp
	$(CXX) -c main.cpp

getToken.o: getToken.cpp
	$(CXX) -c getToken.cpp

SyntaxAnalysis.o: SyntaxAnalysis.cpp
	$(CXX) -c SyntaxAnalysis.cpp

clean:
	rm -f *.o $(TARGET)
