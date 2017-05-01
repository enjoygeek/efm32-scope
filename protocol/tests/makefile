RM := rm -rf
CC := gcc
CPP := g++

C_SRC = ../protocol.c

TESTS = catch_main.cpp test_strings.cpp
EXECUTABLE = catch_tests

C_OBJS := $(C_SRC:.c=.o)
CPP_OBJS := $(TESTS:.cpp=.obj)
OBJS = $(C_OBJS) $(CPP_OBJS)

CATCH_HEADER = ./catch.hpp
CATCH_PCH = $(CATCH_HEADER).gch

INCLUDE_DIRS = -I./

CFLAGS := -O2 -Wall -c -fmessage-length=0 -std=c11
CFLAGS += $(INCLUDE_DIRS)

CPPFLAGS := -O2 -Wall -c -std=c++11
CPPLAGS += $(INCLUDE_DIRS)

%.o: %.c
	@echo 'Building object: $(abspath $@)'
	$(CC) $(CFLAGS) $^ -o $@


%.obj: %.cpp $(CATCH_PCH)
	@echo 'Building object: $(abspath $@)'
	$(CPP) $(CPPFLAGS) $(filter %.cpp,$^) -o $@


release: all

# All Target
all: $(EXECUTABLE)

# Tool invocations
$(EXECUTABLE): $(OBJS)
	@echo 'Building target: $@'
	$(CPP) -lstdc++ -o $@ $^ 
	@echo 'Finished building target: $@'
	@echo ' '

$(CATCH_PCH): $(CATCH_HEADER)
	$(CPP) $^ -o $@

# Other Targets
clean:
	-$(RM) $(OBJS) $(EXECUTABLE) $(CATCH_PCH)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: