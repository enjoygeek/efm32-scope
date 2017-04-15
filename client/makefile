RM := rm -rf
CC := gcc
CPP := g++
EXECUTABLE := just_print

C_SRC += ../protocol/protocol.c \
#		 ./just_print.c
		 
CPP_SRC = ./just_print.cpp

C_OBJS := $(C_SRC:.c=.o)
CPP_OBJS := $(CPP_SRC:.cpp=.obj)
S_OBJS := $(S_SRC:.s=.a)
OBJS = $(C_OBJS) $(S_OBJS) $(CPP_OBJS)

INCLUDE_DIRS = -I./

CFLAGS := -g -gdwarf-2 -O0 -Wall -c -fmessage-length=0 -std=c11
CFLAGS += $(INCLUDE_DIRS)

%.o: %.c
	@echo 'Building object: $(abspath $@)'
	$(CC) $(CFLAGS) $^ -o $@
	
%.obj: %.cpp
	@echo 'Building object: $(abspath $@)'
	$(CPP) $(CFLAGS) $^ -o $@


%.a: %.s
	@echo 'Building object: $(abspath $@)'
	$(CC) $(CFLAGS) $^ -o $@


release: all

# All Target
all: $(EXECUTABLE)

# Tool invocations
just_print: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	$(CC) -g -gdwarf-2 -o $@ $(OBJS)
	@echo 'Finished building target: $@'
	@echo ' '


# Other Targets
clean:
	-$(RM) $(OBJS) $(EXECUTABLE)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
