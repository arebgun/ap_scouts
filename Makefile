CC     = gcc
VPATH  = src
CFLAGS = -Wall

config_editor_cflags = $(CFLAGS) `pkg-config --cflags gtk+-2.0`

analysis_libs      = -lgsl -lgslcblas -lm
config_editor_libs = `pkg-config --libs gtk+-2.0`
swarm_gui_libs     = -lglut
swarm_cli_libs     = -lm

analysis_obj      = analysis.o
config_editor_obj = config_editor.o
swarm_gui_obj     = definitions.o graphics.o input.o swarm.o swarm_gui.o
swarm_cli_obj     = definitions.o swarm.o swarm_cli.o

all: analysis config-editor swarm-gui swarm-cli

analysis: $(analysis_obj)
	$(CC) $(analysis_libs) $^ -o $@

config-editor: $(config_editor_obj)
	$(CC) $(config_editor_libs) $^ -o $@

swarm-gui: $(swarm_gui_obj)
	$(CC) $(swarm_gui_libs) $^ -o $@

swarm-cli: $(swarm_cli_obj)
	$(CC) $(swarm_cli_libs) $^ -o $@

clean:
	-rm -f $(analysis_obj) $(config_editor_obj) $(swarm_gui_obj) $(swarm_cli_obj)

dist-clean: clean
	-rm -f analysis config-editor swarm-gui swarm-cli

analysis.o: analysis.h
config_editor.o: config_editor.c
	$(CC) $(config_editor_cflags) -c $^ -o $@
definitions.o: definitions.h
graphcis.o: definitions.h graphics.h
input.o: graphics.h input.h swarm.h
swarm.o: definitions.h swarm.h
swarm_gui.o: graphics.h input.h swarm.h swarm_gui.h
swarm_cli.o: swarm.h swarm_cli.h

.PHONY: all clean
