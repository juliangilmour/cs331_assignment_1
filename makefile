 # the compiler: gcc for C program, define as g++ for C++
CC = g++

  # compiler flags:
  #  -g    adds debugging information to the executable file
  #  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -o

SRC = animal_search.cpp
BIN = $(patsubst %.cpp,%,$(SRC))

all: $(BIN)

$(BIN): %: %.cpp
	$(CC) -std=c++11 $(CFLAGS) $@ $<

clean:
	rm -f $(BIN)

.PHONY: all clean