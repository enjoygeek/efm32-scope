RM := rm -rf
CC := gcc
CPP := g++

C_SRC = ../protocol.c

TESTS = pack_unpack.cpp packet_sizes.cpp

C_OBJS := $(C_SRC:.c=.o)
CPP_OBJS := $(TESTS:.cpp=.obj)
OBJS = $(C_OBJS) $(CPP_OBJS)
EXECUTABLES := $(TESTS:.cpp=)
INTERMEDIATE := $(TESTS:.cpp=.out)

INCLUDE_DIRS = -I./

CFLAGS := -g -gdwarf-2 -O0 -Wall -c -fmessage-length=0 -std=c11
CFLAGS += $(INCLUDE_DIRS)

CPPFLAGS := -g -gdwarf-2 -O0 -Wall -c -std=c++11
CPPLAGS += $(INCLUDE_DIRS)

%.o: %.c
	@echo 'Building object: $(abspath $@)'
	$(CC) $(CFLAGS) $^ -o $@


%.obj: %.cpp
	@echo 'Building object: $(abspath $@)'
	$(CPP) $(CPPFLAGS) $^ -o $@

release: all

# All Target
all: $(INTERMEDIATE)
#all: pack_unpack 

# Tool invocations
%.out: %.obj $(C_OBJS)
	@echo 'Building target: $@'
	$(CPP) -lstdc++ -o $@ $^ 
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) $(EXECUTABLES) $(INTERMEDIATE)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

 