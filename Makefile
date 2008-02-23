CC = gcc
CFLAGS = -Wall -D_GNU_SOURCE
SWARM_LIBS = -lglut
ANALYSIS_LIBS = -lm
SRC_DIR = src

SWARM_SRC = $(SRC_DIR)/graphics.c $(SRC_DIR)/input.c $(SRC_DIR)/swarm.c \
$(SRC_DIR)/graphics.h $(SRC_DIR)/input.h $(SRC_DIR)/swarm.h $(SRC_DIR)/defs.h 

ANALYSIS_SRC = $(SRC_DIR)/analysis.c $(SRC_DIR)/analysis.h

all : swarm analysis

swarm :
	$(CC) $(CFLAGS) $(SWARM_LIBS) $(SWARM_SRC) -o swarm

analysis :
	$(CC) $(CFLAGS) $(ANALYSIS_LIBS) $(ANALYSIS_SRC) -o analysis


.PHONY : clean

clean :
	-rm -rf swarm analysis
