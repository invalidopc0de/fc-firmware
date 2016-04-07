CC=gcc

PWD = $(shell pwd)

LIB_PATH =$(PWD)/libs

# Simulation defines
AVISIM_BIN=$(LIB_PATH)/AVISIM/src
AVISIM_INC=$(LIB_PATH)/AVISIM/src
AVISIM_OBJS=$(AVISIM_BIN)/avisim.o $(AVISIM_BIN)/devicelib.o $(AVISIM_BIN)/cJSON/cJSON.o

CFLAGS=-c -Wall -g -I$(AVISIM_INC)

all: sim_test

sim_test: sim_test.o $(LIB_PATH)/control_loop.o $(AVISIM_OBJS) 
	$(CC) sim_test.o control_loop.o $(AVISIM_OBJS) -o sim_test -lm

sim_test.o: sim_test.c
	$(CC) $(CFLAGS) sim_test.c

$(LIB_PATH)/control_loop.o: $(LIB_PATH)/control_loop.c
	$(CC) $(CFLAGS) $(LIB_PATH)/control_loop.c

#avisim:
	# Build AVISIM here

clean:
	rm *.o sim_test
