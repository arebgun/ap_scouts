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

#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include "definitions.h"

extern int help_area_height;
extern int stats_area_width;

extern bool inside_window;
extern bool selection_active;
extern int selected_obstacle_id;

extern int increments[6];
extern int cur_inc_index;

extern char *selections[3];
extern int cur_sel_index;

extern bool show_connectivity;

void initialize_graphics( void );
void draw_all( void );
inline void draw_string( char *s );
inline void draw_goal( Goal *goal );
inline void draw_agent( Agent *agent );
inline void draw_agent_connectivity( void );
inline void draw_obstacle( Obstacle *obstacle );
inline void draw_params_stats( void );
inline void draw_instructions( void );

#endif /*GRAPHICS_H_*/
