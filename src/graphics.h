/*
 ============================================================================
 Name        : graphics.h
 Author      : Antons Rebguns
 Version     : 0.4.0
 Copyright   : Copyright(c) 2007, 2008
 Description : Robotic swarm simulator (OpenGL)
 ============================================================================
 */

#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include "defs.h"

/******** EXTERNAL VARIABLES ***********/
extern Parameters params;
extern Statistics stats;

extern Agent **agents;
extern Obstacle **obstacles;
extern Goal *goal;

extern bool running;
/***************************************/

int help_area_height;
int stats_area_width;

GLfloat agent_color[3];
GLfloat agent_color_coll[3];
GLfloat agent_color_conn[3];
GLfloat goal_color[3];
GLfloat obstacle_color[3];

bool inside_window;
bool selection_active;
int selected_obstacle_id;

int increments[6];
int cur_inc_index;

char *selections[3];
int cur_sel_index;

bool show_connectivity;

void draw_all( void );
inline void draw_string( char *s );
inline void draw_goal( Goal *goal );
inline void draw_agent( Agent *agent );
inline void draw_agent_connectivity( void );
inline void draw_obstacle( Obstacle *obstacle );
inline void draw_params_stats( void );
inline void draw_instructions( void );

#endif /*GRAPHICS_H_*/
