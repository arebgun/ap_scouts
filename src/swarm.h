/*
 ============================================================================
 Name        : swarm.h
 Author      : Antons Rebguns
 Version     : 0.4.0
 Copyright   : Copyright(c) 2007, 2008
 Description : Robotic swarm simulator (OpenGL)
 ============================================================================
 */

#ifndef SWARM_H_
#define SWARM_H_

#include <stdio.h>
#include <stdbool.h>

#include "defs.h"

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
void initialize_graphics( void );
int change_agent_number( int agent_number );
int change_obstacle_number( int obstacle_number );
float calculate_force( Agent *agent, void *object, ObjectType obj_type );
void move_agents( void );
double approximation( double a, double b, double ( *func ) ( double ) );
double gaussian_quadrature( double a, double b, int n, double ( *func ) ( double ) );
double gammaln( double xx );    
double beta_function( double z, double w );
double incomplete_beta( double t );
double f( double p );
void process_normal_keys( unsigned char key, int x, int y );
void process_special_keys( int key, int x, int y );
void process_mouse_buttons( int button, int state, int x, int y );
void process_mouse_entry( int state );
void process_mouse_active_motion( int x, int y );
void update_reach(void);
void run_gui( int time );
void run_cli( int argc, char **argv );
void print_usage( char *program_name );
int main ( int argc, char **argv );

#endif /*SWARM_H_*/
