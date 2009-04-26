/*
 * This file is part of Robotic Swarm Simulator.
 *
 * Copyright (C) 2007, 2008, 2009 Antons Rebguns.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <stdlib.h>

#include "definitions.h"

float agent_color[3] = { 0.0f, 0.2f, 1.0f };
float agent_color_coll[3] = { 1.0f, 0.0f, 0.0f };
float agent_color_conn[3] = { 0.8f, 0.8f, 0.8f };

float goal_color[3] = { 1.0f, 0.0f, 0.2f };
float obstacle_color[3] = { 0.0f, 0.4f, 0.0f };

Parameters params;
Statistics stats;

Agent **agents = NULL;
Obstacle **obstacles = NULL;
Goal *goal = NULL;

bool ( *agent_reached_goal )( Agent * ) = NULL;

bool running = false;
