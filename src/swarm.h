/*
 * This file is part of Robotic Swarm Simulator.
 * 
 * Copyright (C) 2007, 2008 Antons Rebguns <anton at cs dot uwyo dot edu>.
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

#ifndef SWARM_H_
#define SWARM_H_

#include <stdbool.h>
#include <stdio.h>

#include "definitions.h"

int read_config_file( char *p_filename );
void output_simulation_parameters( FILE *output );
int create_goal( void );
void deploy_agent( Agent *agent );
Agent *create_agent( int id );
int create_swarm( void );
Obstacle *create_obstacle( int id, bool random_radius, float radius_range );
int create_obstacle_course( void );
void free_memory( void );
void reset_statistics( void );
bool agent_reached_goal_actual( Agent *agent );
bool agent_reached_goal_radius( Agent *agent );
bool agent_reached_goal_chain( Agent *agent );
void initialize_simulation( void );
int save_scenario( char *filename );
int load_scenario( char *filename );
void restart_simulation( void );
int change_agent_number( int agent_number );
int change_obstacle_number( int obstacle_number );
float calculate_force( Agent *agent, void *object, ObjectType obj_type );
void move_agents( void );
void update_reach(void);

#endif /*SWARM_H_*/
