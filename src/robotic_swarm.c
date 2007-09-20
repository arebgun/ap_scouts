/*
 ============================================================================
 Name        : robotic_swarm.cpp
 Author      : Anton Rebguns
 Version     : 0.0.5
 Copyright   : Copyright(c) 2007
 Description : Robotic swarm simulator (OpenGL)
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "GL/gl.h"
#include "GL/glut.h"

/*************** TEMPORARY *************************/
double alpha;
double beta;
double ppHat;
int yy;
int kk;
int nn;
/***************************************************/

typedef enum e_viewmode
{
	CLI,
	GUI,
	
} Viewmode;

typedef enum e_obj_type
{
	AGENT,
	OBSTACLE,
	GOAL,
	
} ObjectType;

/**
 * \struct Vector2f
 * \brief  Represents a vector in 2D space.
 */
typedef struct s_vec2f
{
	GLfloat x;
	GLfloat y;
	
} Vector2f;

/**
 * \struct Agent
 * \brief  Represents a robot.
 */
typedef struct s_agent
{
	int id;
	
	Vector2f i_position;	// Robot deployment position
	Vector2f position;		// Robot position vector
	Vector2f velocity;		// Robot velocity vector

	float radius;			// Robot size
	float mass;				// Robot mass
	
	bool goal_reached;		// true, if reached goal, false otherwise
	
	GLfloat color[3];
	
} Agent;

/**
 * \struct Goal
 * \brief  Represents a goal.
 */
typedef struct s_goal
{
	int id;
	
	Vector2f position;
	
	float width;
	float mass;

	GLfloat color[3];
	
} Goal;

/**
 * \struct Obstacle
 * \brief  Represents an obstacle.
 */
typedef struct s_obstacle
{
	int id;
	
	Vector2f position;
	
	float radius;
	float mass;
	
	GLfloat color[3];
	
} Obstacle;

typedef enum e_quadrant
{
	NW, N, NE,
	W,  C, E,
	SW, S, SE,
	
} Quadrant;

typedef enum e_force_law
{
	NEWTONIAN,
	LENNARD_JONES,
	
} ForceLaw;

typedef struct s_params
{
	int world_width;
	int world_height;
	int timer_delay_ms;
	
	unsigned int goal_random_seed;
	float goal_width;
	float goal_mass;
	Quadrant goal_quadrant;
	
	unsigned int agent_random_seed;
	int agent_number;
	float agent_radius;
	float agent_mass;
	int deployment_width;
	int deployment_height;
	Quadrant deployment_quadrant;	
	
	unsigned int obstacle_random_seed;
	int obstacle_number;
	float obstacle_radius;
	float obstacle_radius_min;
	float obstacle_radius_max;
	float obstacle_mass;
	
	bool enable_agent_goal_f;
	bool enable_agent_obstacle_f;
	bool enable_agent_agent_f;
	
	float max_f_agent_goal;
	float max_f_agent_obstacle;
	float max_f_agent_agent;
	
	float R;
	float range_coefficient;
	ForceLaw force_law;
	
	float max_V;	// Maximum agent velocity 
	float G;		// Gravitational force
	float p;		// power to which distance is raised to
	
	int time_limit;
	int runs_number;
	
	char *results_filename;
	
	int n_number;
	int k_number;
	int a_b_number;
	int *n_array;
	int *k_array;
	double *alpha_array;
	double *beta_array;
	
	double alpha;
	double beta;
	
} Parameters;

typedef struct s_statistics
{
	int timeStep;
	int reached_goal;
	float reach_ratio;
	
} Statistics;

int help_area_height = 100;
int stats_area_width = 150;

GLfloat agent_color[3] = { 0.0f, 0.2f, 1.0f };
GLfloat goal_color[3] = { 1.0f, 0.0f, 0.2f };
GLfloat obstacle_color[3] = { 0.0f, 0.4f, 0.0f };

Viewmode mode;

Parameters params;
Statistics stats;

Agent **agents = NULL;
Obstacle **obstacles = NULL;
Goal *goal = NULL;

bool running;

int increments[6] = { 1, 5, 10, 20, 50, 100 };
int cur_inc_index = 0;

char *selections[2] = { "AGENT", "OBSTACLE" };
int cur_sel_index = 0;

/**
 * \fn int read_config_file( char *p_filename )
 * \brief parses configuration file
 * \param p_filename to configuration file name
 * \return 0 on success, -1 on failure
 */
int read_config_file( char *p_filename )
{
	FILE *p_config;
	p_config = fopen( p_filename, "r" );
	
	if ( p_config != NULL )
	{
		char parameter[100];
		char value[100];
		
		while ( fscanf( p_config, "%100s%100s%*[^\n]", parameter, value ) != EOF )
		{
			if ( strcmp( "view_mode", parameter ) == 0 )
			{
				mode = atoi( value );
			}
			else if ( strcmp( "world_width", parameter ) == 0 )
			{
				params.world_width = atoi( value );
			}
			else if ( strcmp( "world_height", parameter ) == 0 )
			{
				params.world_height = atoi( value );
			}
			else if ( strcmp( "timer_delay_ms", parameter ) == 0 )
			{
				params.timer_delay_ms = atoi( value );
			}
			else if ( strcmp( "goal_random_seed", parameter ) == 0 )
			{
				params.goal_random_seed = atoi( value );
			}
			else if ( strcmp( "goal_width", parameter ) == 0 )
			{
				params.goal_width = atof( value );
			}
			else if ( strcmp( "goal_mass", parameter ) == 0 )
			{
				params.goal_mass = atof( value );
			}
			else if ( strcmp( "goal_quadrant", parameter ) == 0 )
			{
				params.goal_quadrant = atoi( value );
			}
			else if ( strcmp( "agent_random_seed", parameter ) == 0 )
			{
				params.agent_random_seed = atoi( value );
			}
			else if ( strcmp( "agent_number", parameter ) == 0 )
			{
				params.agent_number = atoi( value );
			}
			else if ( strcmp( "agent_radius", parameter ) == 0 )
			{
				params.agent_radius = atof( value );
			}
			else if ( strcmp( "agent_mass", parameter ) == 0 )
			{
				params.agent_mass = atof( value );
			}
			else if ( strcmp( "deployment_width", parameter ) == 0 )
			{
				params.deployment_width = atoi( value );
			}
			else if ( strcmp( "deployment_height", parameter ) == 0 )
			{
				params.deployment_height = atoi( value );
			}
			else if ( strcmp( "deployment_quadrant", parameter ) == 0 )
			{
				params.deployment_quadrant = atoi( value );
			}
			else if ( strcmp( "obstacle_random_seed", parameter ) == 0 )
			{
				params.obstacle_random_seed = atoi( value );
			}
			else if ( strcmp( "obstacle_number", parameter ) == 0 )
			{
				params.obstacle_number = atoi( value );
			}
			else if ( strcmp( "obstacle_radius", parameter ) == 0 )
			{
				params.obstacle_radius = atof( value );
			}
			else if ( strcmp( "obstacle_radius_min", parameter ) == 0 )
			{
				params.obstacle_radius_min = atof( value );
			}
			else if ( strcmp( "obstacle_radius_max", parameter ) == 0 )
			{
				params.obstacle_radius_max = atof( value );
			}
			else if ( strcmp( "obstacle_mass", parameter ) == 0 )
			{
				params.obstacle_mass = atof( value );
			}
			else if ( strcmp( "enable_agent_goal_f", parameter ) == 0 )
			{
				params.enable_agent_goal_f = atoi( value );
			}
			else if ( strcmp( "enable_agent_obstacle_f", parameter ) == 0 )
			{
				params.enable_agent_obstacle_f = atoi( value );
			}
			else if ( strcmp( "enable_agent_agent_f", parameter ) == 0 )
			{
				params.enable_agent_agent_f = atoi( value );
			}
			else if ( strcmp( "max_f_agent_goal", parameter ) == 0 )
			{
				params.max_f_agent_goal = atof( value );
			}
			else if ( strcmp( "max_f_agent_obstacle", parameter ) == 0 )
			{
				params.max_f_agent_obstacle = atof( value );
			}
			else if ( strcmp( "max_f_agent_agent", parameter ) == 0 )
			{
				params.max_f_agent_agent = atof( value );
			}
			else if ( strcmp( "R", parameter ) == 0 )
			{
				params.R = atof( value );
			}
			else if ( strcmp( "range_coefficient", parameter ) == 0 )
			{
				params.range_coefficient = atof( value );
			}
			else if ( strcmp( "force_law", parameter ) == 0 )
			{
				params.force_law = atoi( value );
			}
			else if ( strcmp( "max_V", parameter ) == 0 )
			{
				params.max_V = atof( value );
			}
			else if ( strcmp( "G", parameter ) == 0 )
			{
				params.G = atof( value );
			}
			else if ( strcmp( "p", parameter ) == 0 )
			{
				params.p = atof( value );
			}
			else if ( strcmp( "time_limit", parameter ) == 0 )
			{
				params.time_limit = atoi( value );
			}
			else if ( strcmp( "runs_number", parameter ) == 0 )
			{
				params.runs_number = atoi( value );
			}
			else if ( strcmp( "results_filename", parameter ) == 0 )
			{
				params.results_filename = strdup( value );
			}
			else if ( strcmp( "n_number", parameter ) == 0 )
			{
				params.n_number = atoi( value );
			}
			else if ( strcmp( "k_number", parameter ) == 0 )
			{
				params.k_number = atoi( value );
			}
			else if ( strcmp( "a_b_number", parameter ) == 0 )
			{
				params.a_b_number = atoi( value );
			}
			else if ( strcmp( "n_array", parameter ) == 0 )
			{
				params.n_array = ( int * ) calloc( params.n_number, sizeof( int ) );
				
				if ( params.n_array == NULL )
				{
					printf( "Error while allocating memory for n_array!" );
					return -1;
				}
				
				int i;
				
				params.n_array[0] = atoi( strtok( value, "," ) );
				
				for ( i = 1; i < params.n_number; i++ )
				{
					params.n_array[i] = atoi( strtok( NULL, "," ) );
				}
			}
			else if ( strcmp( "k_array", parameter ) == 0 )
			{
				params.k_array = ( int * ) calloc( params.k_number + 1, sizeof( int ) );

				if ( params.k_array == NULL )
				{
					printf( "Error while allocating memory for k_array!" );
					return -1;
				}
				
				int i;
				
				params.k_array[0] = atoi( strtok( value, "," ) );
				
				for ( i = 1; i < params.k_number; i++ )
				{
					params.k_array[i] = atoi( strtok( NULL, "," ) );
				}
			}
			else if ( strcmp( "alpha_array", parameter ) == 0 )
			{
				params.alpha_array = ( double * ) calloc( params.a_b_number, sizeof( double ) );

				if ( params.alpha_array == NULL )
				{
					printf( "Error while allocating memory for alpha_array!" );
					return -1;
				}
				
				int i;
				
				params.alpha_array[0] = atof( strtok( value, "," ) );
				
				for ( i = 1; i < params.a_b_number; i++ )
				{
					params.alpha_array[i] = atof( strtok( NULL, "," ) );
				}
			}
			else if ( strcmp( "beta_array", parameter ) == 0 )
			{
				params.beta_array = ( double * ) calloc( params.a_b_number, sizeof( double ) );

				if ( params.beta_array == NULL )
				{
					printf( "Error while allocating memory for beta_array!" );
					return -1;
				}
				
				int i;
				
				params.beta_array[0] = atof( strtok( value, "," ) );
				
				for ( i = 1; i < params.a_b_number; i++ )
				{
					params.beta_array[i] = atof( strtok( NULL, "," ) );
				}
			}
			else if ( strcmp( "alpha", parameter ) == 0 )
			{
				params.alpha = atof( value );
			}
			else if ( strcmp( "beta", parameter ) == 0 )
			{
				params.beta = atof( value );
			}
			else
			{
				printf( "WARNING: Unknown parameter [%s]\n", parameter );
			}
		}
	}
	else
	{
		printf( "Error opening configuration file: \"%s\"!\n", p_filename );
		return -1;
	}
	
	fclose( p_config );
	
	return 0;
}

int create_goal( void )
{
	/******************** Initialize goal ********************************/
	goal = ( Goal * ) malloc( sizeof( Goal ) );
	
	if ( goal == NULL )
	{
		printf( "Error while allocating memory for goal!" );
		return -1;
	}
	
	// Initialize goal random number seed
	if ( params.goal_random_seed == 0 ) { srand( ( unsigned int ) time( NULL ) ); }
	else { srand( params.goal_random_seed ); }
	
	goal->id = 0;
	goal->mass = params.goal_mass;
	goal->width = params.goal_width;
	
	float quadrant_width = params.world_width / 3.0f;
	float quadrant_height = params.world_height / 3.0f;

	float offset_x;
	float offset_y;
	
	/*
	 *		 ------------- 
	 * 		| NW | N | NE |
	 * 		|-------------
	 * 		| N  | C | E  |
	 * 		|-------------
	 * 		| SW | S | SE |
	 * 		 -------------
	 */
	switch ( params.goal_quadrant )
	{
		case NW:
			offset_x = 0.0f;
			offset_y = quadrant_height * 2.0f;
			break;
			
		case N:
			offset_x = quadrant_width;
			offset_y = quadrant_height * 2.0f;
			break;
			
		case NE:
			offset_x = quadrant_width * 2.0f;
			offset_y = quadrant_height * 2.0f;
			break;
			
		case W:
			offset_x = 0.0f;
			offset_y = quadrant_height;
			break;
			
		case C:
			offset_x = quadrant_width;
			offset_y = quadrant_height;
			break;
			
		case E:
			offset_x = quadrant_width * 2.0f;;
			offset_y = quadrant_height * 2.0f;
			break;
			
		case SW:
			offset_x = 0.0f;
			offset_y = 0.0f;
			break;
			
		case S:
			offset_x = quadrant_width;
			offset_y = 0.0f;
			break;
			
		case SE:
			offset_x = quadrant_width * 2.0f;
			offset_y = 0.0f;
			break;
			
		default:
			offset_x = 0.0f;
			offset_y = 0.0f;
	}
	
	goal->position.x = rand() % ( int ) quadrant_width + offset_x;
	goal->position.y = rand() % ( int ) quadrant_height + offset_y;
	
	memcpy( goal->color, goal_color, 3 * sizeof( float ) );
	
	/*************************************************/
	printf( "******* GOAL %d *******\n", goal->id );
	printf( "mass\t\t%f\n", goal->mass );
	printf( "width\t\t%f\n", goal->width );
	printf( "x\t\t%f\n", goal->position.x );
	printf( "y\t\t%f\n", goal->position.y );
	printf( "*************************" );
	printf( "\n\n" );
	/*************************************************/
	
	return 0;
}

void deploy_agent( Agent *agent )
{
	float quadrant_width = params.world_width / 3.0f;
	float quadrant_height = params.world_height / 3.0f;

	float offset_x;
	float offset_y;
	
	/*
	 *		 ------------- 
	 * 		| NW | N | NE |
	 * 		|-------------
	 * 		| N  | C | E  |
	 * 		|-------------
	 * 		| SW | S | SE |
	 * 		 -------------
	 */
	switch ( params.deployment_quadrant )
	{
		case NW:
			offset_x = 10.0f;
			offset_y = params.world_height - params.deployment_height - 10.0f;
			break;
			
		case N:
			offset_x = quadrant_width + ( quadrant_width - params.deployment_height ) / 2.0f;
			offset_y = params.world_height - params.deployment_height - 10.0f;
			break;
			
		case NE:
			offset_x = params.world_width - params.deployment_width - 10.0f;
			offset_y = params.world_height - params.deployment_height - 10.0f;
			break;
			
		case W:
			offset_x = 10.0f;
			offset_y = quadrant_height + ( quadrant_height - params.deployment_height ) / 2.0f;
			break;
			
		case C:
			offset_x = quadrant_width + ( quadrant_width - params.deployment_height ) / 2.0f;
			offset_y = quadrant_height + ( quadrant_height - params.deployment_height ) / 2.0f;
			break;
			
		case E:
			offset_x = params.world_width - params.deployment_width - 10.0f;
			offset_y = quadrant_height + ( quadrant_height - params.deployment_height ) / 2.0f;
			break;
			
		case SW:
			offset_x = 10.0f;
			offset_y = 10.0f;
			break;
			
		case S:
			offset_x = quadrant_width + ( quadrant_width - params.deployment_height ) / 2.0f;
			offset_y = 10.0f;
			break;
			
		case SE:
			offset_x = params.world_width - params.deployment_width - 10.0f;
			offset_y = 10.0f;
			break;
			
		default:
			offset_x = 10.0f;
			offset_y = 10.0f;
	}
	
	agent->i_position.x = rand() % params.deployment_width + offset_x;
	agent->i_position.y = rand() % params.deployment_height + offset_y;
	
	agent->position.x = agent->i_position.x;
	agent->position.y = agent->i_position.y;
}

Agent *create_agent( int id )
{
	Agent *agent = ( Agent * ) malloc( sizeof( Agent ) );
	
	if ( agent == NULL )
	{
		printf( "Error while allocating memory for an agent!" );
		return NULL;
	}
	
	agent->id = id;
	agent->mass = params.agent_mass;
	agent->goal_reached = false;
	agent->radius = params.agent_radius;
	agent->velocity.x = 0.0f;
	agent->velocity.y = 0.0f;
	
	deploy_agent( agent );
	
	memcpy( agent->color, agent_color, 3 * sizeof( float ) );
	
	/*************************************************/
	printf( "****** AGENT %d *****\n", agent->id );
	printf( "mass\t\t%f\n", agent->mass );
	printf( "goal\t\t%d\n", agent->goal_reached );
	printf( "radius\t\t%f\n", agent->radius );
	printf( "x\t\t%f\n", agent->position.x );
	printf( "y\t\t%f\n", agent->position.y );
	printf( "vx\t\t%f\n", agent->velocity.x );
	printf( "vy\t\t%f\n", agent->velocity.y );
	printf( "***********************" );
	printf( "\n\n" );
	/*************************************************/
	
	return agent;
}

int create_swarm( void )
{
	/******************** Initialize agents ******************************/
	agents = ( Agent ** ) calloc( params.agent_number, sizeof( Agent * ) );
	
	if ( agents == NULL )
	{
		printf( "Error while allocating memory for agents array!" );
		return -1;
	}
	
	// Initialize agents random number seed
	if ( params.agent_random_seed == 0 ) { srand( ( unsigned int ) time( NULL ) ); }
	else { srand( params.agent_random_seed ); }

	int i;
	
	for ( i = 0; i < params.agent_number; i++ )
	{
		agents[i] = create_agent( i );
		
		if ( agents[i] == NULL )
		{
			printf( "Error while allocating memory for agent %d!", i );
			return -1;
		}
	}

	return 0;
}

Obstacle *create_obstacle( int id, bool random_radius, float radius_range )
{
	Obstacle *obstacle = ( Obstacle * ) malloc( sizeof( Obstacle ) );
	
	if ( obstacle == NULL )
	{
		printf( "Error while allocating memory for obstacle!" );
		return NULL;
	}
	
	obstacle->id = id;
	obstacle->mass = params.obstacle_mass;
			
	if ( random_radius )
	{
		float random = ( float ) rand() / ( float ) RAND_MAX;
		obstacle->radius = random * radius_range + params.obstacle_radius_min;
	} 
	else { obstacle->radius = params.obstacle_radius; }
	
	obstacle->position.x = rand() % ( params.world_width - 20 ) + 10;
	obstacle->position.y = rand() % ( params.world_height - 20 ) + 10;
	
	memcpy( obstacle->color, obstacle_color, 3 * sizeof( float ) );
	
	/********************************************************/
	printf( "****** OBSTACLE %d ******\n", obstacle->id );
	printf( "mass\t\t%f\n", obstacle->mass );
	printf( "radius\t\t%f\n", obstacle->radius );
	printf( "x\t\t%f\n", obstacle->position.x );
	printf( "y\t\t%f\n", obstacle->position.y );
	printf( "***************************" );
	printf( "\n\n" );
	/********************************************************/
	
	return obstacle;
}

int create_obstacle_course( void )
{
	/******************** Initialize obstacles ***************************/
	obstacles = ( Obstacle ** ) calloc( params.obstacle_number, sizeof( Obstacle * ) );
	
	if ( obstacles == NULL )
	{
		printf( "Error while allocating memory for obstacles array!" );
		return -1;
	}
	
	// Initialize obstacles random number seed
	if ( params.obstacle_random_seed == 0 ) { srand( ( unsigned int ) time( NULL ) ); }
	else { srand( params.obstacle_random_seed ); }
	
	bool random_radius = ( params.obstacle_radius == 0 ) ? true : false;
	float radius_range = params.obstacle_radius_max - params.obstacle_radius_min;

	int i;
	
	for ( i = 0; i < params.obstacle_number; i++ )
	{
		obstacles[i] = create_obstacle( i, random_radius, radius_range );
		
		if ( obstacles[i] == NULL )
		{
			printf( "Error while allocating memory for obstacle %d!", i );
			return -1;
		}
	}

	return 0;
}

void free_memory( void )
{
    /******************** Free used memory ****************/
    int i;
    
    if ( goal != NULL ) { free( goal ); }
    
    for ( i = 0; i < params.agent_number; i++ )
    {
    	if ( agents[i] != NULL ) { free( agents[i] ); }
    }
    
    if ( agents != NULL ) { free( agents ); }
    
    for ( i = 0; i < params.obstacle_number; i++ )
    {
    	if ( obstacles[i] != NULL ) { free( obstacles[i] ); }
    }
    
    if ( obstacles != NULL ) { free( obstacles ); }
    if ( params.n_array != NULL ) { free( params.n_array ); }
    if ( params.k_array != NULL ) { free( params.k_array ); }
    /*******************************************************/
}

void reset_statistics( void )
{
	stats.timeStep = 0;
	stats.reach_ratio = 0.0f;
	stats.reached_goal = 0;
}

int save_scenario( char *filename )
{
	FILE *scenario;
	scenario = fopen( filename, "w" );
	
	if ( scenario != NULL )
	{
		// World parameters
		fprintf( scenario, "%d %d %d\n", params.world_width, params.world_height, params.timer_delay_ms );
		
		// Goal parameters
		fprintf( scenario, "%d %f %f %d\n", params.goal_random_seed, params.goal_width, params.goal_mass, params.goal_quadrant );
		
		// Agent parameters
		fprintf( scenario, "%d %d %f %f ", params.agent_random_seed, params.agent_number, params.agent_radius, params.agent_mass );
		fprintf( scenario, "%d %d %d\n", params.deployment_width, params.deployment_height, params.deployment_quadrant );
		
		// Obstacle parameters
		fprintf( scenario, "%d %d %f ", params.obstacle_random_seed, params.obstacle_number, params.obstacle_mass );
		fprintf( scenario, "%f %f %f\n", params.obstacle_radius, params.obstacle_radius_min, params.obstacle_radius_max );

		// Physics parameters
		fprintf( scenario, "%d %d %d\n", params.enable_agent_goal_f, params.enable_agent_obstacle_f, params.enable_agent_agent_f );
		fprintf( scenario, "%f %f %f\n", params.max_f_agent_goal, params.max_f_agent_obstacle, params.max_f_agent_agent );
		fprintf( scenario, "%f %f %d\n", params.R, params.range_coefficient, params.force_law );
		fprintf( scenario, "%f %f %f\n", params.max_V, params.G, params.p );
		
		// Batch parameters
		fprintf( scenario, "%d %d\n", params.time_limit, params.runs_number );
		
		// Statistics
		fprintf( scenario, "%d %d %f\n", stats.timeStep, stats.reached_goal, stats.reach_ratio );
		
		/******************************* Current values for all objects **********************************************/
		fprintf( scenario, "%d %f %f %f %f\n", goal->id, goal->mass, goal->width, goal->position.x, goal->position.y );
		
		int i;
		
		for ( i = 0; i < params.agent_number; i++ )
		{
			Agent *a = agents[i];
			
			fprintf( scenario, "%d %f %f %d ", a->id, a->mass, a->radius, a->goal_reached );
			fprintf( scenario, "%f %f ", a->i_position.x, a->i_position.y );
			fprintf( scenario, "%f %f ", a->position.x, a->position.y );
			fprintf( scenario, "%f %f\n", a->velocity.x, a->velocity.y );
		}
		
		for ( i = 0; i < params.obstacle_number; i++ )
		{
			Obstacle *o = obstacles[i];
			
			fprintf( scenario, "%d %f %f ", o->id, o->mass, o->radius );
			fprintf( scenario, "%f %f\n", o->position.x, o->position.y );
		}
		/************************************************************************************************************/
	}
	else
	{
		printf( "Scenario file [%s] could not be created!", filename );
		return -1;
	}
	
	fclose( scenario );
	
	return 0;
}

int load_scenario( char *filename )
{
	FILE *scenario;
	scenario = fopen( filename, "r" );
	
	if ( scenario != NULL )
	{
		running = false;
		
		// World parameters
		fscanf( scenario, "%d %d %d", &params.world_width, &params.world_height, &params.timer_delay_ms );

		// Goal parameters
		fscanf( scenario, "%d %f %f %d", &params.goal_random_seed, &params.goal_width, &params.goal_mass, ( int * ) &params.goal_quadrant );

		// Agent parameters
		fscanf( scenario, "%d %d %f %f", &params.agent_random_seed, &params.agent_number, &params.agent_radius, &params.agent_mass );
		fscanf( scenario, "%d %d %d", &params.deployment_width, &params.deployment_height, ( int * ) &params.deployment_quadrant );

		// Obstacle parameters
		fscanf( scenario, "%d %d %f", &params.obstacle_random_seed, &params.obstacle_number, &params.obstacle_mass );
		fscanf( scenario, "%f %f %f", &params.obstacle_radius, &params.obstacle_radius_min, &params.obstacle_radius_max );

		// Physics parameters
		fscanf( scenario, "%d %d %d", ( int * ) &params.enable_agent_goal_f, ( int * ) &params.enable_agent_obstacle_f, ( int * ) &params.enable_agent_agent_f );
		fscanf( scenario, "%f %f %f", &params.max_f_agent_goal, &params.max_f_agent_obstacle, &params.max_f_agent_agent );
		fscanf( scenario, "%f %f %d", &params.R, &params.range_coefficient, ( int * ) &params.force_law );
		fscanf( scenario, "%f %f %f", &params.max_V, &params.G, &params.p );
		
		// Batch parameters
		fscanf( scenario, "%d %d", &params.time_limit, &params.runs_number );
		
		// Statistics
		fscanf( scenario, "%d %d %f", &stats.timeStep, &stats.reached_goal, &stats.reach_ratio );
		
		/******************************* Current values for all objects **********************************************/
		free_memory();
		
		// Create simulation objects
		if ( create_goal() != 0 ) { return -1; }
		if ( create_swarm() != 0 ) { return -1; }
		if ( create_obstacle_course() != 0 ) { return -1; }
		
		fscanf( scenario, "%d %f %f %f %f", &(goal->id), &(goal->mass), &(goal->width), &(goal->position.x), &(goal->position.y) );

		int i;
		
		for ( i = 0; i < params.agent_number; i++ )
		{
			Agent *a = agents[i];

			fscanf( scenario, "%d %f %f %d", &(a->id), &(a->mass), &(a->radius), ( int * ) &(a->goal_reached) );
			fscanf( scenario, "%f %f", &(a->i_position.x), &(a->i_position.y) );
			fscanf( scenario, "%f %f", &(a->position.x), &(a->position.y) );
			fscanf( scenario, "%f %f", &(a->velocity.x), &(a->velocity.y) );
		}

		for ( i = 0; i < params.obstacle_number; i++ )
		{
			Obstacle *o = obstacles[i];
			
			fscanf( scenario, "%d %f %f", &(o->id), &(o->mass), &(o->radius) );
			fscanf( scenario, "%f %f", &(o->position.x), &(o->position.y) );
		}
		/************************************************************************************************************/
	}
	else
	{
		printf( "Scenario file [%s] not found!", filename );
		return -1;
	}
	
	fclose( scenario );
	
	return 0;
}

int initialize_simulation( char *p_filename )
{
	running = false;
	
	// Initialize simulation parameters to sane defaults,
	// in case some information is missing from config file
	params.world_width = 800;
	params.world_height = 600;
	params.timer_delay_ms = 8;
	params.goal_random_seed = 0;
	params.goal_width = 15.0f;
	params.goal_mass = 10.0f;
	params.goal_quadrant = 2;
	params.agent_random_seed = 0;
	params.agent_number = 100;
	params.deployment_width = 100;
	params.deployment_height = 100;
	params.deployment_quadrant = 4;
	params.obstacle_random_seed = 0;
	params.obstacle_number = 20;
	params.obstacle_radius = 3.0f;
	params.obstacle_mass = 1.0f;
	params.enable_agent_goal_f = 1;
	params.enable_agent_obstacle_f = 1;
	params.enable_agent_agent_f = 0;
	params.max_f_agent_goal = 4.0f;
	params.max_f_agent_obstacle = 14.0f;
	params.max_f_agent_agent = 4.0f;
	params.R = 50.0f;
	params.range_coefficient = 1.5f;
	params.force_law = 0;
	params.max_V = 0.5f;
	params.G = 1000.0f;
	params.p = 2.0f;
	params.time_limit = 1000;
	params.runs_number = 10;
	params.alpha = 1.0;
	params.beta = 1.0;
	
	// Reset statistics
	reset_statistics();
	
	// Read in simulation parameters from config file
	if ( read_config_file( p_filename ) != 0 ) { return EXIT_FAILURE; }
	
	// Create simulation objects
	if ( create_goal() != 0 ) { return -1; }
	if ( create_swarm() != 0 ) { return -1; }
	if ( create_obstacle_course() != 0 ) { return -1; }
	
	return 0;
}

void restart_simulation( void )
{
	int i;
	
	running = false;
	
	// Reset statistics
	reset_statistics();
	
	for ( i = 0; i < params.agent_number; i++ )
	{
		agents[i]->position.x = agents[i]->i_position.x;
		agents[i]->position.y = agents[i]->i_position.y;
		agents[i]->velocity.x = 0.0f;
		agents[i]->velocity.y = 0.0f;
		agents[i]->goal_reached = false;
	}
}

void initialize_graphics( void )
{
    glClearColor( 1.0f, 1.0f, 1.0f, 0.0f );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    glOrtho( -stats_area_width, params.world_width, -help_area_height, params.world_height, 0.0, 100.0 );
}

bool agent_reached_goal( Agent *agent )
{
    Vector2f goal_pos = goal->position;
    Vector2f agent_pos = agent->position;

    if ( fabs( goal_pos.x - agent_pos.x ) < goal->width / 2.0f &&
         fabs( goal_pos.y - agent_pos.y ) < goal->width / 2.0f )
    {
            return true;
    }

    return false;
}

int change_agent_number( int agent_number )
{
	int i;	
	int delta = agent_number - params.agent_number;
	
	if ( delta > 0 )
	{
		agents = ( Agent ** ) realloc( agents, agent_number * sizeof( Agent * ) );
		
		if ( agents == NULL )
		{
			printf( "Error while expanding memory for agents array!" );
			return -1;
		}
		
		for ( i = params.agent_number; i < agent_number; i++ )
		{
			agents[i] = create_agent( i );
		}
		
		params.agent_number = agent_number;
	}
	else if ( delta < 0 && abs( delta ) < params.agent_number )
	{
		agents = ( Agent ** ) realloc( agents, agent_number * sizeof( Agent * ) );
		
		if ( agents == NULL )
		{
			printf( "Error while shrinking memory for agents array!" );
			return -1;
		}
		
		params.agent_number = agent_number;
	}
	else if ( delta < 0 && abs( delta ) >= params.agent_number )
	{
		agents = ( Agent ** ) realloc( agents, sizeof( Agent * ) );
		
		if ( agents == NULL )
		{
			printf( "Error while shrinking memory for agents array!" );
			return -1;
		}
		
		params.agent_number = 1;
	}
	
	return 0;
}

int change_obstacle_number( int obstacle_number )
{
	int i;	
	int delta = obstacle_number - params.obstacle_number;
	
	if ( delta > 0 )
	{
		obstacles = ( Obstacle ** ) realloc( obstacles, obstacle_number * sizeof( Obstacle * ) );
		
		if ( obstacles == NULL )
		{
			printf( "Error while expanding memory for obstacles array!" );
			return -1;
		}
		
		bool random_radius = ( params.obstacle_radius == 0 ) ? true : false;
		float radius_range = params.obstacle_radius_max - params.obstacle_radius_min;
		
		for ( i = params.obstacle_number; i < obstacle_number; i++ )
		{
			obstacles[i] = create_obstacle( i, random_radius, radius_range );
		}
		
		params.obstacle_number = obstacle_number;
	}
	else if ( delta < 0 && abs( delta ) < params.obstacle_number )
	{
		obstacles = ( Obstacle ** ) realloc( obstacles, obstacle_number * sizeof( Obstacle * ) );
		
		if ( agents == NULL )
		{
			printf( "Error while shrinking memory for obstacles array!" );
			return -1;
		}
		
		params.obstacle_number = obstacle_number;
	}
	else if ( delta < 0 && abs( delta ) >= params.obstacle_number )
	{
		obstacles = ( Obstacle ** ) realloc( obstacles, sizeof( Obstacle * ) );
		
		if ( agents == NULL )
		{
			printf( "Error while shrinking memory for obstacles array!" );
			return -1;
		}
		
		params.obstacle_number = 1;
	}
	
	return 0;
}

float calculate_force( Agent *agent, void *object, ObjectType obj_type )
{
	Vector2f agent_pos = agent->position;
	Vector2f obj_pos;
	float obj_mass;
	
	switch( obj_type )
	{
		case AGENT:
			obj_pos = ( ( Agent * ) object )->position;
			obj_mass = ( ( Agent * ) object )->mass;
			break;
			
		case OBSTACLE:
			obj_pos = ( ( Obstacle * ) object )->position;
			obj_mass = ( ( Obstacle * ) object )->mass;
			break;
			
		case GOAL:
			obj_pos = ( ( Goal * ) object )->position;
			obj_mass = ( ( Goal * ) object )->mass;
			break;
	}
	
    float f = 0.0f;
    float distance_to_obj = sqrt( pow( agent_pos.x - obj_pos.x, 2 ) + pow( agent_pos.y - obj_pos.y, 2 ) );

    if ( distance_to_obj != 0 )
    {
	    switch( params.force_law )
		{
			case NEWTONIAN:
				switch( obj_type )
				{
					case AGENT:
						f = params.G * agent->mass * obj_mass / pow( distance_to_obj, params.p );
						break;

					case GOAL:
						f = params.G * agent->mass * obj_mass / pow( distance_to_obj, params.p );
						break;
						
					case OBSTACLE:
						f = -( params.G * agent->mass * obj_mass / pow( distance_to_obj /*- ( ( Obstacle * ) object )->radius*/, params.p ) );
						break;
				}
				break;
				
			case LENNARD_JONES:
				switch( obj_type )
				{
					float epsilon, sigma;
					float c, d;
					float lhs, rhs;
					
					// agent-agent interactions, repulsive and attractive components
					case AGENT:
						epsilon = 16.5f;
						sigma = params.R / 3.0f;
						c = 0.1f;
						d = 0.1f;
						
						lhs = 2 * d * pow( params.R, 12 ) / pow( distance_to_obj, 13 );
						rhs = c * pow( params.R, 5 ) / pow( distance_to_obj, 7 );
						
						f = 24.0f * epsilon * ( lhs - rhs );
						break;
						
					// agent-obstacle interactions, repulsive component only
					case OBSTACLE:
						epsilon = 16.5f;
						sigma = ( ( Obstacle * ) object )->radius;
						c = 0.1f;
						
						rhs = 2 * c * pow( sigma, 12 ) / pow( distance_to_obj/* - ( ( Obstacle * ) object )->radius*/, 13 );
						
						f = -24.0f * epsilon * rhs;
						break;
						
					// agent-goal interactions, attractive component only
					case GOAL:
						epsilon = 16.5f;
						sigma = params.R * params.R * 5.0f;
						d = 0.1f;
						
						lhs = d * pow( sigma, 6 ) / pow( distance_to_obj, 7 );
						
						f = 24.0f * epsilon * lhs;
						break;
				}
				break;
		}
    }
	
    return f;
}

void move_agents( void )
{
	int i, j;

	for ( i = 0; i < params.agent_number; i++ )
	{
		Agent *agent = agents[i];
		
		Vector2f agent_pos = agent->position;
		Vector2f goal_pos = goal->position;
		
		float force_x = 0.0f;
		float force_y = 0.0f;
		
		/************************** Calculate force between an obstacle and an agent ***********************/
		if ( params.enable_agent_obstacle_f )
		{
			for ( j = 0; j < params.obstacle_number; j++ )
			{
				Obstacle *obs = obstacles[j];
				Vector2f obs_pos = obs->position;
				
				float angle_to_obstacle = atan2( obs_pos.y - agent_pos.y, obs_pos.x - agent_pos.x );
				//float distance = sqrt( pow( obs_pos.x - agent_pos.x, 2 ) + pow( obs_pos.y - agent_pos.y, 2 ) ) - obs->radius;
				float net_force = 0.0f;
				
		        //if ( distance <= params.range_coefficient * params.R )
		        //{
					net_force = calculate_force( agent, obs, OBSTACLE );
					
					if ( net_force < -params.max_f_agent_obstacle ) { net_force = -params.max_f_agent_obstacle; }
		        //}
		
		        force_x += net_force * cos( angle_to_obstacle );
		        force_y += net_force * sin( angle_to_obstacle );
			}
		}
        /****************************************************************************************************/
		
		/************************** Calculate force between agents *************************************************/
		if ( params.enable_agent_agent_f )
		{
			for ( j = 0; j < params.agent_number; j++ )
			{
				Agent *agent2 = agents[j];
				Vector2f agent2_pos = agent2->position;
				
				float angle_to_agent2 = atan2( agent2_pos.y - agent_pos.y, agent2_pos.x - agent_pos.x );
				float distance = sqrt( pow( agent_pos.x - agent2_pos.x, 2 ) + pow( agent_pos.y - agent2_pos.y, 2 ) );
		        float net_force = 0.0f;
	
		        if ( distance <= params.range_coefficient * params.R )
		        {
		        	net_force = calculate_force( agent, agent2, AGENT );
					
		        	if ( distance < params.R ) { net_force = -net_force; }
					if ( net_force > params.max_f_agent_agent ) { net_force = params.max_f_agent_agent; }
					if ( net_force < -params.max_f_agent_agent ) { net_force = -params.max_f_agent_agent; }
	        	}
		        
		        force_x += net_force * cos( angle_to_agent2 );
		        force_y += net_force * sin( angle_to_agent2 );
			}
		}
		/***********************************************************************************************************/
		
		/********************** Calculate force between the goal and an agent *************************/
		if ( params.enable_agent_goal_f )
		{
			float angle_to_goal = atan2( goal_pos.y - agent_pos.y, goal_pos.x - agent_pos.x );
			float net_force = 0.0f;
			
			net_force = calculate_force( agent, goal, GOAL );
			
			if ( net_force > params.max_f_agent_goal ) { net_force = params.max_f_agent_goal; }
	
	        force_x += net_force * cos( angle_to_goal );
	        force_y += net_force * sin( angle_to_goal );
		}
        /**********************************************************************************************/
        
        // update agent velocity vector
        agent->velocity.x += force_x / agent->mass;
        agent->velocity.y += force_y / agent->mass;
        
        float velocity_magnitude = sqrt( pow( agent->velocity.x, 2 ) + pow( agent->velocity.y, 2 ) );
        
        // check if new velocity exceeds the maximum
        if ( velocity_magnitude > params.max_V )
        {
        	agent->velocity.x = ( agent->velocity.x * params.max_V ) / velocity_magnitude;
        	agent->velocity.y = ( agent->velocity.y * params.max_V ) / velocity_magnitude;
        }
        
        // update agent position
        agent->position.x += agent->velocity.x;
        agent->position.y += agent->velocity.y;
        
        if ( !agent->goal_reached && agent_reached_goal( agent ) )
        {
        	agent->goal_reached = true;
        	stats.reached_goal++;
        	stats.reach_ratio = ( float ) stats.reached_goal / ( float ) params.agent_number;
        }
	}
	
	++stats.timeStep;
}

/*************************** Calculate P(y|p) by Dr. A-S formula ************************/


double approximation( double a, double b, double ( *func ) ( double ) )
{
	double c = 0.5 * ( b - a );
	double d = 0.5 * ( b + a );
   
	return c * ( ( 5.0 / 9.0 ) * func( c * ( -( sqrt( 3.0 / 5.0 ) ) ) + d ) +
		   ( 8.0 / 9.0 ) * func( d ) + ( 5.0 / 9.0 ) * func( c * sqrt ( 3.0 / 5.0 ) + d ) );
}

double gaussian_quadrature( double a, double b, int n, double ( *func ) ( double ) )
{
	double step = ( a + b ) / n;
	double upper = a + step;
	double lower = a;
	double sum = 0;

	int i;
   
	for ( i = 0; i < n; i++ )
	{
		double temp = approximation( lower, upper, func );

		lower = upper;
		upper += step;

		sum += temp;
	}
   
	return ( sum > 1.0 ) ? 1.0 : sum; 
}

double gammaln( double xx )
{
	int j;
	double x, y, tmp, ser;
	static double cof[6] = { 76.18009172947146,     -86.50532032941677,
	                         24.01409824083091,     -1.231739572450155,
	                         0.1208650973866179e-2, -0.5395239384953e-5 };
	
	y = x = xx;
	tmp = x + 5.5;
	tmp -= ( x + 0.5 ) * log( tmp );
	ser = 1.000000000190015;
	
	for ( j = 0; j <= 5; j++ )
	{
		ser += cof[j] / ++y;
	}
	
	return -tmp + log( 2.5066282746310005 * ser / x );
}
    
double beta_function( double z, double w )
{
	return exp( gammaln( z ) + gammaln( w ) - gammaln( z + w ) );
}

double incomplete_beta( double t )
{
	int y = yy;
	int n = nn;
	
	return pow( t, y - 1 ) * pow( 1 - t, n - y );
}

double f( double p )
{
	double a = alpha;
	double b = beta;
	double pHat = ppHat;
	int y = yy;
	int n = nn;
	int k = kk;
	
	double r = k * pHat + params.alpha;
	double s = k * ( 1 - pHat ) + params.beta;
	
	double b1 = beta_function( r, s );
	double b2 = beta_function( y, n - y + 1 );
	double bb = b1 * b2;
	
	double C = pow( bb, -1 );
	
	return C * pow( p, r - 1 ) * pow( 1 - p, s - 1 ) * gaussian_quadrature( 0.0, p, 100, incomplete_beta );
}

/****************************************************************************************/

double approx_fac( int n )
{
	static double pi_o_3 = M_PI / 3.0;
	static double pi_t_2 = 2.0 * M_PI;
	
	return sqrt( pi_t_2 * n + pi_o_3 ) * pow( n, n ) * pow( M_E, -n );
}

double fac( int n )
{
	double result = 1.0;
	
	int i;
	
	for ( i = 2; i <= n; i++ )
	{
		result *= i;
	}
	
	return result;
}

void draw_string( char *s )
{
	int i;
	
	for ( i = 0; i < strlen( s ); i++ )
	{
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_12, s[i] );
	}
}

void draw_goal( Goal *goal )
{
	/*************** Draw the goal **************/
	float x1 = goal->position.x - goal->width / 2;
	float y1 = goal->position.y + goal->width / 2;
	float x2 = goal->position.x + goal->width / 2;
	float y2 = goal->position.y - goal->width / 2;
		
	glColor3fv( goal->color );
	glRectf( x1, y1, x2, y2 );
	/********************************************/
}

void draw_agent( Agent *agent )
{
	if ( agent->position.x >= 0.0f && agent->position.y >= 0.0f )
	{
		/*************** Draw an agent ***********************/
		glPointSize( agent->radius );
		glColor3fv( agent->color );
	
		glBegin( GL_POINTS );
			glVertex2f( agent->position.x, agent->position.y );
		glEnd();
		/*****************************************************/
	}
}

void draw_obstacle( Obstacle *obstacle )
{
	/*************** Draw an obstacle **********************************/
	glColor3fv( obstacle->color );

	glPushMatrix();
		glTranslatef( obstacle->position.x, obstacle->position.y, 0.0f );
		glutSolidSphere( obstacle->radius, 20, 20 );
	glPopMatrix();
	/*******************************************************************/
}

void draw_params_stats( void )
{
	// Draw simulation parameters on screen
	
	char label[100];
	int line = 1;
	int line_offset = 13;
	int screen_offset_x = 10 - stats_area_width;
	int screen_offset_y = 10;
	
	glColor3f( 0.7f, 0.0f, 0.6f );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - line * line_offset );
	sprintf( label, "Agent #: %d", params.agent_number );
	draw_string( label );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "Obstacle #: %d", params.obstacle_number );
	draw_string( label );

	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "Timer Delay: %d", params.timer_delay_ms );
	draw_string( label );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "Max Velocity: %.2f", params.max_V );
	draw_string( label );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "G Force: %.2f", params.G );
	draw_string( label );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "p Power: %.2f", params.p );
	draw_string( label );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "Reached Goal #: %d", stats.reached_goal );
	draw_string( label );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "Reach Ratio: %.2f%%", stats.reach_ratio * 100.0f );
	draw_string( label );
	
	glRasterPos2i( screen_offset_x, params.world_height - screen_offset_y - ( ++line * line_offset ) );
	sprintf( label, "Time Step: %d", stats.timeStep );
	draw_string( label );
	
	glColor3f( 0.0f, 0.0f, 0.0f );
	
	glBegin( GL_LINES );
		glVertex2f( 0.0f, 0.0f );
		glVertex2f( 0.0f, params.world_height );
	glEnd();
}

void draw_instructions( void )
{
	// Draw simulation instructions (help)
	
	char label[100];
	int line = 1;
	int line_offset = 13;
	int screen_offset = 10;
	
	glColor3f( 0.0f, 0.0f, 0.0f );
	
	glRasterPos2i( screen_offset, -3 * screen_offset - line * line_offset );
	sprintf( label, "'S' -- Start/Stop the simualtion" );
	draw_string( label );
	
	glRasterPos2i( screen_offset, -3 * screen_offset - ( ++line * line_offset ) );
	sprintf( label, "'R' -- Restart the simulation" );
	draw_string( label );

	glRasterPos2i( screen_offset, -3 * screen_offset - ( ++line * line_offset ) );
	sprintf( label, "'PageUp' -- Increase timer delay (slower)" );
	draw_string( label );

	glRasterPos2i( screen_offset, -3 * screen_offset - ( ++line * line_offset ) );
	sprintf( label, "'PageDown' -- Decrease timer delay (faster)" );
	draw_string( label );
	
	line = 1;
	
	glRasterPos2i( screen_offset + 300, -3 * screen_offset - line * line_offset );
	sprintf( label, "'I' -- Change object increment/decrement" );
	draw_string( label );
	
	glRasterPos2i( screen_offset + 300, -3 * screen_offset - ( ++line * line_offset ) );
	sprintf( label, "'A' -- Selects agent # to be incremented/decremented" );
	draw_string( label );
	
	glRasterPos2i( screen_offset + 300, -3 * screen_offset - ( ++line * line_offset ) );
	sprintf( label, "'UP' -- Increments # of objects" );
	draw_string( label );
	
	glRasterPos2i( screen_offset + 300, -3 * screen_offset - ( ++line * line_offset ) );
	sprintf( label, "'DOWN' -- Decrements # of objects" );
	draw_string( label );

	
	if ( running )
	{
		glColor3f( 0.0f, 0.5f, 0.0f );
		glRasterPos2i( params.world_width - 70, -help_area_height + line_offset );
		sprintf( label, "RUNNING" );
		draw_string( label );
	}
	else
	{
		glColor3f( 0.5f, 0.0f, 0.0f );
		glRasterPos2i( params.world_width - 70, -help_area_height + line_offset );
		sprintf( label, "STOPPED" );
		draw_string( label );
	}
	
	glColor3f( 0.0f, 0.0f, 0.0f );
	glRasterPos2i( params.world_width - 170, -help_area_height + line_offset );
	sprintf( label, "%s [%3d]", selections[cur_sel_index], increments[cur_inc_index] );
	draw_string( label );
	
	glBegin( GL_LINES );
		glVertex2f( 0.0f, 0.0f );
		glVertex2f( params.world_width, 0.0f );
	glEnd();
}

void display( void )
{
	int i;
	
	glClear( GL_COLOR_BUFFER_BIT );
	
	/*************** Draw the goal **************/
	draw_goal( goal );
	/********************************************/
	
	/*************** Draw the agents ************/
	for( i = 0; i < params.agent_number; i++ )
	{
		draw_agent( agents[i] );
	}
	/********************************************/
	
	/*************** Draw the obstacles *********/
	for ( i = 0; i < params.obstacle_number; i++ )
	{
		draw_obstacle( obstacles[i] );
	}
	/********************************************/
	
	draw_params_stats();
	draw_instructions();

	glutSwapBuffers();
}

void process_normal_keys( unsigned char key, int x, int y )
{
	if ( key == 's' || key == 'S' )
	{
		running = !running;
		glutPostRedisplay();
	}
	else if ( key == 'r' || key == 'R' )
	{
		restart_simulation();
		glutPostRedisplay();
	}
	else if ( key == 'i' || key == 'I' )
	{
		++cur_inc_index;
		cur_inc_index %= 6;
		glutPostRedisplay();
	}
	else if ( key == 'a' || key == 'A' )
	{
		cur_sel_index = 0;		// AGENT
		glutPostRedisplay();
	}
	else if ( key == 'o' || key == 'O' )
	{
		cur_sel_index = 1;		// OBSTACLE
		glutPostRedisplay();
	}
	else if ( key == 'd' || key == 'D' )
	{
		if ( save_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
		glutPostRedisplay();
	}
	else if ( key == 'l' || key == 'L' )
	{
		if ( load_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
		glutPostRedisplay();
	}
	else if ( key == 'q' || key == 'Q' )
	{
		free_memory();
		exit( EXIT_SUCCESS );
	}
}

void process_special_keys( int key, int x, int y )
{
	switch ( key )
	{
		case GLUT_KEY_PAGE_UP:
			++params.timer_delay_ms;
			glutPostRedisplay();
			break;
			
		case GLUT_KEY_PAGE_DOWN:
			if ( params.timer_delay_ms > 1 )
			{
				--params.timer_delay_ms;
				glutPostRedisplay();
			}
			break;
		
		case GLUT_KEY_UP:
			if ( cur_sel_index == 0 )
			{
				if ( change_agent_number( params.agent_number + increments[cur_inc_index] ) != 0 ) { exit( EXIT_FAILURE ); }
			}
			else if ( cur_sel_index == 1 )
			{
				if ( change_obstacle_number( params.obstacle_number + increments[cur_inc_index] ) != 0 ) { exit( EXIT_FAILURE ); }
			}
			else
			{
				printf( "Unknown object index [%d]", cur_sel_index );
			}
			glutPostRedisplay();
			break;
			
		case GLUT_KEY_DOWN:
			if ( cur_sel_index == 0 )
			{
				if ( change_agent_number( params.agent_number - increments[cur_inc_index] ) != 0 ) { exit( EXIT_FAILURE ); }
			}
			else if ( cur_sel_index == 1 )
			{
				if ( change_obstacle_number( params.obstacle_number - increments[cur_inc_index] ) != 0 ) { exit( EXIT_FAILURE ); }
			}
			else
			{
				printf( "Unknown object index [%d]", cur_sel_index );
			}
			glutPostRedisplay();
			break;
	}
}

void run_gui( int time )
{
	if ( running )
	{
		move_agents();
		glutPostRedisplay();
	}
	
	glutTimerFunc( params.timer_delay_ms, run_gui, stats.timeStep );
}

void output_simulation_parameters( FILE *output )
{
	fprintf( output, "\n\n" );
	
	fprintf( output, "# world_width = %d\n", params.world_width );
	fprintf( output, "# world_height = %d\n", params.world_height );
	fprintf( output, "# timer_delay_ms = %d\n", params.timer_delay_ms );
	fprintf( output, "# goal_random_seed = %d\n", params.goal_random_seed );
	fprintf( output, "# goal_width = %.2f\n", params.goal_width );
	fprintf( output, "# goal_mass = %.2f\n", params.goal_mass );
	fprintf( output, "# goal_quadrant = %d\n", params.goal_quadrant );
	fprintf( output, "# agent_random_seed = %d\n", params.agent_random_seed );
	fprintf( output, "# agent_number = %d\n", params.agent_number );
	fprintf( output, "# agent_radius = %.2f\n", params.agent_radius );
	fprintf( output, "# agent_mass = %.2f\n", params.agent_mass );
	fprintf( output, "# deployment_width = %d\n", params.deployment_width );
	fprintf( output, "# deployment_height = %d\n", params.deployment_height );
	fprintf( output, "# deployment_quadrant = %d\n", params.deployment_quadrant );
	fprintf( output, "# obstacle_random_seed = %d\n", params.obstacle_random_seed );
	fprintf( output, "# obstacle_number = %d\n", params.obstacle_number );
	fprintf( output, "# obstacle_radius = %.2f\n", params.obstacle_radius );
	fprintf( output, "# obstacle_radius_min = %.2f\n", params.obstacle_radius_min );
	fprintf( output, "# obstacle_radius_max = %.2f\n", params.obstacle_radius_max );
	fprintf( output, "# obstacle_mass = %.2f\n", params.obstacle_mass );
	fprintf( output, "# enable_agent_goal_f = %d\n", params.enable_agent_goal_f );
	fprintf( output, "# enable_agent_obstacle_f = %d\n", params.enable_agent_obstacle_f );
	fprintf( output, "# enable_agent_agent_f = %d\n", params.enable_agent_agent_f );
	fprintf( output, "# max_f_agent_goal = %.2f\n", params.max_f_agent_goal );
	fprintf( output, "# max_f_agent_obstacle = %.2f\n", params.max_f_agent_obstacle );
	fprintf( output, "# max_f_agent_agent = %.2f\n", params.max_f_agent_agent );
	fprintf( output, "# R = %.2f\n", params.R );
	fprintf( output, "# range_coefficient = %.2f\n", params.range_coefficient );
	fprintf( output, "# force_law = %d\n", params.force_law );
	fprintf( output, "# max_V = %.2f\n", params.max_V );
	fprintf( output, "# G = %.2f\n", params.G );
	fprintf( output, "# p = %.2f\n", params.p );
	fprintf( output, "# time_limit = %d\n", params.time_limit );
	fprintf( output, "# runs_number = %d\n", params.runs_number );
	fprintf( output, "# results_filename = %s\n", params.results_filename );
	fprintf( output, "# n_number = %d\n", params.n_number );
	fprintf( output, "# k_number = %d\n", params.k_number );
	fprintf( output, "# a_b_number = %d\n", params.a_b_number );
	fprintf( output, "# n_array = ");
	
	int i;
	
	for ( i = 0; i < params.n_number; i++ )
	{
		fprintf( output, "%d,", params.n_array[i] );
	}
	
	fprintf( output, "\n# k_array = " );
	
	for ( i = 0; i < params.k_number; i++ )
	{
		fprintf( output, "%d,", params.k_array[i] );
	}
	
	fprintf( output, "\n# alpha_array = " );
	
	for ( i = 0; i < params.a_b_number; i++ )
	{
		fprintf( output, "%.2f,", params.alpha_array[i] );
	}

	fprintf( output, "\n# beta_array = " );
	
	for ( i = 0; i < params.a_b_number; i++ )
	{
		fprintf( output, "%.2f,", params.beta_array[i] );
	}

	fprintf( output, "\n# alpha = %.2f\n", params.alpha );
	fprintf( output, "# beta = %.2f\n", params.beta );
	
	fprintf( output, "\n\n" );
}

void run_cli( void )
{
	FILE *p_results;
	p_results = fopen( params.results_filename, "w+" );

	if ( p_results == NULL )
	{
		printf( "Error opening file [%s]!", params.results_filename );
		exit( EXIT_FAILURE );
	}
	
	output_simulation_parameters( p_results );
	fflush( p_results );

	double small_p = 0.0;
	
	int n;
	
	for ( n = 0; n < params.n_number; n++ )
	{
		change_agent_number( params.n_array[n] );
		
		int i;

		/***************** Calculate ground truth #2 - big_P_prime *********************************/
		int big_P_prime[params.n_array[n] + 1];
		
		for ( i = 0; i <= params.n_array[n]; i++ )
		{
			big_P_prime[i] = 0;
		}

		for ( i = 0; i < params.runs_number; i++ )
		{
			restart_simulation();
			
	        /******** initialize agents position *****************/
	        srand( ( unsigned int ) time( NULL ) );
	        
	        int agent;
	        
	        for ( agent = 0; agent < params.agent_number; agent++ )
	        {
	            deploy_agent( agents[agent] );
	        }
	        /*****************************************************/
	        
	        while ( stats.reached_goal != params.agent_number && stats.timeStep < params.time_limit )
	        {
	            move_agents();
	        }
	        
	        int j;
	        
	        for ( j = 0; j <= stats.reached_goal; j++ )
	        {
	        	++big_P_prime[j];
	        }
	        
	        small_p += stats.reached_goal;
	        
	        if ( i % 100 == 0 ) { printf( "i = %d\n", i ); }
		}
		
		small_p /= ( double ) ( params.n_array[n] * params.runs_number );
		
		printf( "small_p = %f\n", small_p );
		
		for ( i = 0; i <= params.agent_number; i++ )
		{
			printf( "big_P_prime[%d] = %d\n", i, big_P_prime[i] );
		}
		/*******************************************************************************************/
		
		params.k_array[params.k_number] = params.n_array[n];
		
		for ( i = 0; i < 5; i++ )
		{
			change_agent_number( params.k_array[i] );

			int j;
			int success_number[params.k_array[i] + 1];
					
			for ( j = 0; j <= params.k_array[i]; j++ )
			{
				success_number[j] = 0;
			}
			
			for ( j = 0; j < params.runs_number; j++ )
			{
				restart_simulation();
				
		        /******** initialize agents position *****************/
		        srand( ( unsigned int ) time( NULL ) );
		        
		        int agent;
		        
		        for ( agent = 0; agent < params.agent_number; agent++ )
		        {
		            deploy_agent( agents[agent] );
		        }
		        /*****************************************************/
		        
		        while ( stats.reached_goal != params.agent_number && stats.timeStep < params.time_limit )
		        {
		            move_agents();
		        }
		        
		        ++success_number[stats.reached_goal];
		        
		        if ( j % 100 == 0 ) { printf( "j = %d, params.k_array[i] = %d\n", j, params.k_array[i] ); }
			}
			
			nn = params.n_array[n];
			kk = params.k_array[i];
			
			double pHats[params.k_array[i] + 1];
			
			for ( j = 0; j <= params.k_array[i]; j++ )
			{
				printf( "success_number[%d] = %d\n", j, success_number[j] );
				pHats[j] = ( double ) j / ( double ) params.k_array[i];
				printf( "pHats[%d] = %f\n", j, pHats[j] );
			}
			
			fprintf( p_results, "#small_p = %f\n", small_p );
			fprintf( p_results, "#n\t\tk\t\ty\t\tbig_P\t\tbig_P_prime\t\tbig_P_hat\t\terror1\t\terror2\n" );
			
			int y;
			
			for ( y = 1; y <= nn; y++ )
			{
				yy = y;
				
				/*********************** Calculate ground truth #1, big_P **********************************/
				double big_P = 0.0;
				
				for ( j = y; j <= nn; j++ )
				{
					// exp( gammaln( n + 1 ) ) == fac( n )
					double n_choose_j =  exp( gammaln( nn + 1 ) - gammaln( j + 1 ) - gammaln( nn - j + 1 ) );
					big_P += n_choose_j * pow( small_p, j ) * pow( 1.0 - small_p, nn - j );
				}
				/*******************************************************************************************/
				
				double big_P_hat = 0.0;
				
				for ( j = 0; j <= params.k_array[i]; j++ )
				{
					ppHat = pHats[j];
					double rate = ( double ) success_number[j] / ( double ) params.runs_number;
					big_P_hat +=  rate * gaussian_quadrature( 0.0f, 1.0f, 100, f );
				}
				
				double error1 = fabs( big_P_hat - big_P );			
				double error2 = fabs( big_P_hat - ( double ) big_P_prime[y] / ( double ) params.runs_number );
				
				fprintf( p_results, "%d\t\t%d\t\t%d\t\t%f\t\t%f\t\t%f\t\t%f\t\t%f\n",
						 params.n_array[n], params.k_array[i], y, big_P, ( double ) big_P_prime[y] / ( double ) params.runs_number, big_P_hat, error1, error2 );
			}
			
			fprintf( p_results, "\n\n" );
			fflush( p_results );
		}		
	}
			
	fclose( p_results );
}

int main ( int argc, char **argv )
{
	if ( argc == 1 )
	{
		printf( "usage: %s config_filename\n", argv[0] );
		return EXIT_FAILURE;
	}
	
	if ( initialize_simulation( argv[1] ) != 0 ) { return EXIT_FAILURE; }
	
	output_simulation_parameters( stdout );
	
	if ( mode == GUI )
	{
	    glutInit( &argc, argv );
	    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB );
	    glutInitWindowSize( params.world_width + stats_area_width, params.world_height + help_area_height );
	    glutInitWindowPosition( 100, 100 );
	    glutCreateWindow( "Robotic Swarm Simulation" );
	    
	    initialize_graphics();
	    
	    glutDisplayFunc( display );
	    glutKeyboardFunc( process_normal_keys );
	    glutSpecialFunc( process_special_keys );
	    glutTimerFunc( params.timer_delay_ms, run_gui, stats.timeStep );
	
	    glutMainLoop();
	}
	else if ( mode == CLI )
	{
		run_cli();
	}
	else
	{
		printf( "Unknown view mode [%d], exiting!", mode );
		return EXIT_FAILURE;
	}
	
	free_memory();
    
	return EXIT_SUCCESS;
}
