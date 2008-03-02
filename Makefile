CC     = gcc
VPATH  = src
CFLAGS = -Wall

analysis_libs  = -lm
swarm_gui_libs = -lglut
swarm_cli_libs = -lm

analysis_obj  = analysis.o
swarm_gui_obj = definitions.o graphics.o input.o swarm.o swarm_gui.o
swarm_cli_obj = definitions.o swarm.o swarm_cli.o

all: analysis swarm-gui swarm-cli

analysis: $(analysis_obj)
	$(CC) $(CFLAGS) $(analysis_libs) $^ -o $@

swarm-gui: $(swarm_gui_obj)
	$(CC) $(CFLAGS) $(swarm_gui_libs) $^ -o $@

swarm-cli: $(swarm_cli_obj)
	$(CC) $(CFLAGS) $(swarm_cli_libs) $^ -o $@

clean:
	-rm -f analysis swarm-gui swarm-cli $(analysis_obj) $(swarm_gui_obj) $(swarm_cli_obj)

analysis.o : analysis.h
definitions.o : definitions.h
graphcis.o : definitions.h graphics.h
input.o : graphics.h input.h swarm.h
swarm.o : definitions.h swarm.h
swarm_gui.o : graphics.h input.h swarm.h swarm_gui.h
swarm_cli.o : swarm.h swarm_cli.h

.PHONY: all clean
