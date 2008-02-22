CC = gcc
CFLAGS = -Wall
LIBS = -lglut
SRC_DIR = src

SWARM_SRC = $(SRC_DIR)/graphics.c $(SRC_DIR)/input.c $(SRC_DIR)/swarm.c

all : swarm

swarm :
	$(CC) $(CFLAGS) $(LIBS) $(SWARM_SRC) -o swarm

.PHONY : clean

clean :
	-rm -rf swarm
