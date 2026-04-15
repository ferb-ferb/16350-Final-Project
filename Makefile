CXX = g++
CXXFLAGS = -Wall -Iinclude 
TARGET = planner

SRC_d = src
BUILD_d = build

SRC = $(wildcard $(SRC_d)/*.cpp)
OBJ = $(SRC:$(SRC_d)/%.cpp=$(BUILD_d)/%.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET)

$(BUILD_d)/%.o: $(SRC_d)/%.cpp
	@mkdir -p $(BUILD_d)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(TARGET)
