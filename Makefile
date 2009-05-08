#
# This file is part of Robotic Swarm Simulator.
# 
# Copyright (C) 2007, 2008, 2009 Antons Rebguns.
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or (at
# your option) any later version.
# 
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
#

CC     = gcc
VPATH  = src
CFLAGS = -Wall

config_editor_cflags = $(CFLAGS) `pkg-config --cflags gtk+-2.0`

common_libs        = -lgsl -lgslcblas -lpthread
analysis_libs      = $(common_libs) -lm
config_editor_libs = `pkg-config --libs gtk+-2.0`
swarm_gui_libs     = $(common_libs) -lglut
swarm_cli_libs     = $(common_libs) -lm

analysis_obj      = analysis.o
config_editor_obj = config_editor.o
swarm_gui_obj     = definitions.o threading.o queue.o graphics.o input.o swarm.o swarm_gui.o
swarm_cli_obj     = definitions.o threading.o queue.o swarm.o swarm_cli.o

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
threading.o: threading.h
queue.o: queue.h
graphcis.o: definitions.h graphics.h
input.o: graphics.h input.h swarm.h
swarm.o: definitions.h swarm.h
swarm_gui.o: graphics.h input.h swarm.h swarm_gui.h
swarm_cli.o: swarm.h swarm_cli.h

.PHONY: all clean
