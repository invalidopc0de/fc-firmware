CC=gcc

PWD = $(shell pwd)
AVISIM_BIN=$(PWD)/AVISIM/src
AVISIM_INC=$(PWD)/AVISIM/src
AVISIM_OBJS=$(AVISIM_BIN)/avisim.o $(AVISIM_BIN)/devicelib.o $(AVISIM_BIN)/cJSON/cJSON.o

CFLAGS=-c -Wall -g -I$(AVISIM_INC)

all: sim_test

sim_test: sim_test.o control_loop.o $(AVISIM_OBJS) 
	$(CC) sim_test.o control_loop.o $(AVISIM_OBJS) -o sim_test -lm

sim_test.o: sim_test.c
	$(CC) $(CFLAGS) sim_test.c

control_loop.o: control_loop.c
	$(CC) $(CFLAGS) control_loop.c

#avisim:
	# Build AVISIM here

clean:
	rm *.o sim_test
