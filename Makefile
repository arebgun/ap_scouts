CC = gcc
CFLAGS = -Wall
SWARM_GUI_LIBS = -lglut
SWARM_CLI_LIBS = -lm
ANALYSIS_LIBS = -lm
VPATH = src

SWARM_GUI_OBJ = definitions.o graphics.o input.o swarm.o swarm_gui.o
SWARM_CLI_OBJ = definitions.o swarm.o swarm_cli.o
ANALYSIS_OBJ = analysis.o

all : swarm-gui swarm-cli analysis

swarm-gui : $(SWARM_GUI_OBJ)
	$(CC) $(CFLAGS) $(SWARM_GUI_LIBS) $(SWARM_GUI_OBJ) -o swarm-gui

swarm-cli : $(SWARM_CLI_OBJ)
	$(CC) $(CFLAGS) $(SWARM_CLI_LIBS) $(SWARM_CLI_OBJ) -o swarm-cli

analysis : $(ANALYSIS_OBJ)
	$(CC) $(CFLAGS) $(ANALYSIS_LIBS) $(ANALYSIS_OBJ) -o analysis

analysis.o : analysis.h
definitions.o : definitions.h
graphcis.o : definitions.h graphics.h
input.o : graphics.h input.h swarm.h
swarm_cli.o : swarm.h swarm_cli.h
swarm_gui.o : graphics.h input.h swarm.h swarm_gui.h
swarm.o : definitions.h swarm.h

.PHONY : all clean

clean :
	-rm -rf swarm-gui swarm-cli analysis $(SWARM_GUI_OBJ) $(SWARM_CLI_OBJ) $(ANALYSIS_OBJ)
