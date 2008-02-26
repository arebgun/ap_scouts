CC = gcc
CFLAGS = -Wall
SWARM_GUI_LIBS = -lglut
SWARM_CLI_LIBS = -lm
ANALYSIS_LIBS = -lm
SRC_DIR = src

SWARM_GUI_SRC = $(SRC_DIR)/definitions.c $(SRC_DIR)/graphics.c $(SRC_DIR)/input.c $(SRC_DIR)/swarm.c $(SRC_DIR)/swarm_gui.c \
$(SRC_DIR)/definitions.h $(SRC_DIR)/graphics.h $(SRC_DIR)/input.h $(SRC_DIR)/swarm.h $(SRC_DIR)/swarm_gui.h

SWARM_CLI_SRC = $(SRC_DIR)/definitions.c $(SRC_DIR)/swarm.c $(SRC_DIR)/swarm_cli.c \
$(SRC_DIR)/definitions.h $(SRC_DIR)/swarm.h $(SRC_DIR)/swarm_gui.h

ANALYSIS_SRC = $(SRC_DIR)/analysis.c $(SRC_DIR)/analysis.h

all : swarm-gui swarm-cli analysis

swarm-gui :
	$(CC) $(CFLAGS) $(SWARM_GUI_LIBS) $(SWARM_GUI_SRC) -o swarm-gui

swarm-cli :
	$(CC) $(CFLAGS) $(SWARM_CLI_LIBS) $(SWARM_CLI_SRC) -o swarm-cli

analysis :
	$(CC) $(CFLAGS) $(ANALYSIS_LIBS) $(ANALYSIS_SRC) -o analysis


.PHONY : clean

clean :
	-rm -rf swarm-gui swarm-cli analysis
