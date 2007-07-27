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

double ppHat;
int yy;
int kk;
int nn;

int increments[6] = { 1, 5, 10, 20, 50, 100 };
int cur_inc_index = 0;

char *selections[2] = { "AGENT", "OBSTACLE" };
int cur_sel_index = 0;

/***************************************************/

typedef enum e_viewmode
{
	CLI,
	GUI,
	
} Viewmode;

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
	int deployment_width;
	int deployment_height;
	Quadrant deployment_quadrant;	
	
	unsigned int obstacle_random_seed;
	int obstacle_number;
	float obstacle_radius;
	float obstacle_radius_min;
	float obstacle_radius_max;
	float obstacle_mass;
	
	float max_V;	// Maximum agent velocity 
	float G;		// Gravitational force
	float p;		// power to which distance is raised to
	
} Parameters;

typedef struct s_statistics
{
	int timeStep;
	int reached_goal;
	float reach_ratio;
	
} Statistics;

int help_area_height = 100;

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
	agent->mass = 1.0f;
	agent->goal_reached = false;
	agent->radius = 3.0f;
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

int create_obstacles( void )
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
	
	bool random_radii = ( params.obstacle_radius == 0 ) ? true : false;
	float radius_range = params.obstacle_radius_max - params.obstacle_radius_min;

	int i;
	
	for ( i = 0; i < params.obstacle_number; i++ )
	{
		obstacles[i] = ( Obstacle * ) malloc( sizeof( Obstacle ) );
		
		if ( obstacles[i] == NULL )
		{
			printf( "Error while allocating memory for obstacle %d!", i );
			return -1;
		}
		
		obstacles[i]->id = i;
		obstacles[i]->mass = params.obstacle_mass;
				
		if ( random_radii )
		{
			float random = ( float ) rand() / ( float ) RAND_MAX;
			obstacles[i]->radius = random * radius_range + params.obstacle_radius_min;
		} 
		else { obstacles[i]->radius = params.obstacle_radius; }
		
		obstacles[i]->position.x = rand() % ( params.world_width - 20 ) + 10;
		obstacles[i]->position.y = rand() % ( params.world_height - 20 ) + 10;
		
		memcpy( obstacles[i]->color, obstacle_color, 3 * sizeof( float ) );
		
		/********************************************************/
		printf( "****** OBSTACLE %d ******\n", obstacles[i]->id );
		printf( "mass\t\t%f\n", obstacles[i]->mass );
		printf( "radius\t\t%f\n", obstacles[i]->radius );
		printf( "x\t\t%f\n", obstacles[i]->position.x );
		printf( "y\t\t%f\n", obstacles[i]->position.y );
		printf( "***************************" );
		printf( "\n\n" );
		/********************************************************/
	}

	return 0;
}

void reset_statistics( void )
{
	stats.timeStep = 0;
	stats.reach_ratio = 0.0f;
	stats.reached_goal = 0;
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
	params.max_V = 0.5f;
	params.G = 1000.0f;
	params.p = 2.0f;
	
	// Reset statistics
	reset_statistics();
	
	// Read in simulation parameters from config file
	if ( read_config_file( p_filename ) != 0 ) { return EXIT_FAILURE; }
	
	// Create simulation objects
	if ( create_goal() != 0 ) { return -1; }
	if ( create_swarm() != 0 ) { return -1; }
	if ( create_obstacles() != 0 ) { return -1; }
	
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
    glOrtho( 0.0, params.world_width, -help_area_height, params.world_height, 0.0, 100.0 );
}

bool agent_reached_goal( Agent *agent )
{
    Vector2f goal_pos = goal->position;
    Vector2f agent_pos = agent->position;

    if ( abs( goal_pos.x - agent_pos.x ) < goal->width / 2.0f &&
         abs( goal_pos.y - agent_pos.y ) < goal->width / 2.0f )
    {
            return true;
    }

    return false;
}

int change_agent_number( int agent_number )
{
	if ( agent_number == params.agent_number ) { return 0; }
	
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
	else if ( abs( delta ) < params.agent_number )
	{
		agents = ( Agent ** ) realloc( agents, agent_number * sizeof( Agent * ) );
		
		if ( agents == NULL )
		{
			printf( "Error while shrinking memory for agents array!" );
			return -1;
		}
		
		params.agent_number = agent_number;
	}
	
	return 0;
}

float calculate_newtonian_force( Vector2f agent_pos, float agent_mass, Vector2f obj_pos, float obj_mass )
{
    float f = 0.0f;
    float distance_to_obj = sqrt( pow( agent_pos.x - obj_pos.x, 2 ) + pow( agent_pos.y - obj_pos.y, 2 ) );

    // Newtonian universal law of gravitaion
    if ( distance_to_obj != 0 ) { f = params.G * agent_mass * obj_mass / pow( distance_to_obj, params.p ); }

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
		
		for ( j = 0; j < params.obstacle_number; j++ )
		{
			Obstacle *obs = obstacles[j];
			Vector2f obs_pos = obs->position;
			
			/****************** Calculate force between an obstacle and an agent ********************/
			float angle_to_obstacle = atan2( obs_pos.y - agent_pos.y, obs_pos.x - agent_pos.x );
	        float net_force = -calculate_newtonian_force( agent_pos, agent->mass, obs_pos, obs->mass );

	        force_x += net_force * cos( angle_to_obstacle );
	        force_y += net_force * sin( angle_to_obstacle );
	        /****************************************************************************************/
		}
		
		/****************** Calculate force between the goal and an agent *************************/
		float angle_to_goal = atan2( goal_pos.y - agent_pos.y, goal_pos.x - agent_pos.x );
        float net_force = calculate_newtonian_force( agent_pos, agent->mass, goal_pos, goal->mass );

        force_x += net_force * cos( angle_to_goal );
        force_y += net_force * sin( angle_to_goal );
        /******************************************************************************************/
        
        // update agent velocity vector
        agent->velocity.x += force_x;
        agent->velocity.y += force_y;
        
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
        
        if ( agent_reached_goal( agent ) && !agent->goal_reached )
        {
        	agent->goal_reached = true;
        	stats.reached_goal++;
        	stats.reach_ratio = ( float ) stats.reached_goal / ( float ) params.agent_number;
        }
	}
	
	++stats.timeStep;
}

/*************************** Calculate P(y|p) by Dr. A-S formula ************************/
double gammaln( double xx )
{
	int j;
	double x, y, tmp, ser;
	double cof[6] = { 76.18009172947146,     -86.50532032941677,
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
    
double beta_distribution( double z, double w )
{
	return exp( gammaln( z ) + gammaln( w ) - gammaln( z + w ) );
}

double f( double t, double p )
{
	double pHat = ppHat;
	int y = yy;
	int n = nn;
	int k = kk;
	
	double alpha = 1.0;
	double beta = 1.0;
	
	double r = k * pHat + alpha;
	double s = k * ( 1 - pHat ) + beta;
	
	double b1 = beta_distribution( r, s );
	double b2 = beta_distribution( y, n - y + 1 );
	double bb = b1 * b2;
	
	double C = pow( bb, -1 );
	
	return C * pow( t, y - 1 ) * pow( 1 - t, n - y ) * pow( p, r - 1 ) * pow( 1 - p, s - 1 );
}

double calculate_predicted_p( double a, double b, double c, double d, double n, double m )
{
	int i, j;
	
	// approximate double integral with Composite Simpson's rule
	double h = ( b - a ) / n;
	double J1 = 0; // End terms
	double J2 = 0; // Even terms
	double J3 = 0; // Odd terms

	for ( i = 0; i <= n; i++ )
	{
		double x = a + i * h;
		//double HX = ( d - c ) / m;
		double HX = ( ( 1 - x ) - c ) / m;
		//double K1 = f( x, c ) + f( x, d ); // End terms
		double K1 = f( x, c ) + f( x, ( 1 - x ) ); // End terms
		double K2 = 0; // Even terms
		double K3 = 0; // Odd trems

		for ( j = 1; j < m; j++ )
		{
			double y = c + j * HX;
			double Q = f( x, y );
			
			if ( ( j % 2 ) == 0 ) { K2 += Q; }
			else { K3 += Q; }
		}

		double L = ( K1 + 2 * K2 + 4 * K3 ) * HX / 3;
		
		if ( ( i == 0 ) || ( i == n ) ) { J1 += L; }
		else if ( ( i % 2 ) == 0 ) { J2 += L; }
		else { J3 += L; }
    }

	double result = h * ( J1 + 2 * J2 + 4 * J3 ) / 3;
	
	return ( result > 1.0 ) ? 1.0 : result; 
}
/****************************************************************************************/

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
	/*************** Draw an agent ***********************/
	glPointSize( agent->radius );
	glColor3fv( agent->color );

	glBegin( GL_POINTS );
		glVertex2f( agent->position.x, agent->position.y );
	glEnd();
	/*****************************************************/
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
	int screen_offset = 10;
	
	glColor3f( 0.7f, 0.0f, 0.6f );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - line * line_offset );
	sprintf( label, "Agent #: %d", params.agent_number );
	draw_string( label );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "Obstacle #: %d", params.obstacle_number );
	draw_string( label );

	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "Timer Delay: %d", params.timer_delay_ms );
	draw_string( label );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "Max Velocity: %.2f", params.max_V );
	draw_string( label );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "G Force: %.2f", params.G );
	draw_string( label );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "p Power: %.2f", params.p );
	draw_string( label );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "Reached Goal #: %d", stats.reached_goal );
	draw_string( label );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "Reach Ratio: %.2f%%", stats.reach_ratio * 100.0f );
	draw_string( label );
	
	glRasterPos2i( screen_offset, params.world_height - screen_offset - ( ++line * line_offset ) );
	sprintf( label, "Time Step: %d", stats.timeStep );
	draw_string( label );
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
	glRasterPos2i( params.world_width - 400, -help_area_height + line_offset );
	sprintf( label, "Current %s number increment/decrement is [%d]", selections[cur_sel_index], increments[cur_inc_index] );
	draw_string( label );
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
	if ( key == 's' )
	{
		running = !running;
		glutPostRedisplay();
	}
	else if ( key == 'r' )
	{
		restart_simulation();
		glutPostRedisplay();
	}
	else if ( key == 'i' )
	{
		int mod = glutGetModifiers();
		
		if ( mod == GLUT_ACTIVE_ALT )
		{
			++cur_inc_index;
			cur_inc_index %= 6;
		}
		
		glutPostRedisplay();
	}
	else if ( key == 'a' )
	{
		cur_sel_index = 0;		// AGENT
		glutPostRedisplay();
	}
	else if ( key == 'o' )
	{
		cur_sel_index = 1;		// OBSTACLE
		glutPostRedisplay();
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
			else
			{
				// change obstacle number
			}
			glutPostRedisplay();
			break;
			
		case GLUT_KEY_DOWN:
			if ( cur_sel_index == 0 )
			{
				if ( change_agent_number( params.agent_number - increments[cur_inc_index] ) != 0 ) { exit( EXIT_FAILURE ); }
			}
			else
			{
				// change obstacle number
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

void run_cli( void )
{
    int time_limit = 1000;
    int trial_number = 10;
    int runs_number = 1000;

    printf( "timeLimit   = %d\n", time_limit );
    printf( "agentNumber = %d\n", params.agent_number );
    printf( "trialNumber = %d\n", trial_number );
    printf( "runsNumber  = %d\n", runs_number );

    int trial;
    
    for ( trial = trial_number; trial >= 1; trial-- )
    {
    	printf( "\nTrial %d\n", trial );
        int sample_size = trial * 10;
        printf( "\tsampleSize = %d\n", sample_size );

        // set agent number in simualtion to k (sample size)
        restart_simulation();
        change_agent_number( sample_size );

        // initialize agents position
        srand( ( unsigned int ) time( NULL ) );
        
        int i;
        
        for ( i = 0; i < params.agent_number; i++ )
        {
            deploy_agent( agents[i] );
        }

        int t = 0;

        while ( stats.reach_ratio != 1.0f && t < time_limit )
        {
            move_agents();
            t++;
        }

        double small_p_hat = stats.reach_ratio;
        printf( "\tsmallPHat  = %.2f\n", small_p_hat );

        int y = 50;
        double y_ratio = ( double ) y / params.agent_number;

        printf( "\ty = %d\n", y );
        printf( "\tyRatio = %.2f\n", y_ratio );
        printf( "\n" );

        // more than y robots made it to the goal
        int success_total = 0;
        int run;

        for ( run = 1; run <= runs_number; run++ )
        {
        	printf( "\tRun %d\n", run );
            
            restart_simulation();
            change_agent_number( params.agent_number );
            
            srand( ( unsigned int ) time( NULL ) );
            
            int i;
            
            for ( i = 0; i < params.agent_number; i++ )
            {
                deploy_agent( agents[i] );
            }

            t = 0;

            while ( stats.reach_ratio != 1.0f && t < time_limit )
            {
                move_agents();
                t++;
            }

            printf( "\t\treachRatio = %.2f\n", stats.reach_ratio );

            if ( stats.reach_ratio > y_ratio ) { success_total++; }

            printf( "\t\tsuccessTotal = %d\n", success_total );
        }

		ppHat = small_p_hat;
		yy = y;
		kk = sample_size;
		nn = params.agent_number;
		
		double estimated_p = success_total / ( ( double ) runs_number );
		double predicted_p = calculate_predicted_p( 0.0, 1.0, 0.0, 1.0, 500.0, 500.0 );
		
		printf( "\testimatedP = %.5f\n", estimated_p );
		printf( "\tpredictedP = %.5f\n", predicted_p );
		printf( "\tr. error  = %.2f\n", abs( predicted_p - estimated_p ) / estimated_p );
	}
}

int main ( int argc, char **argv )
{
	if ( argc == 1 )
	{
		printf( "usage: %s config_filename\n", argv[0] );
		return EXIT_FAILURE;
	}
	
	if ( initialize_simulation( argv[1] ) != 0 ) { return EXIT_FAILURE; }
	
	/*****************************************************************/
	printf( "world_width = %d\n", params.world_width );
	printf( "world_height = %d\n", params.world_height );
	printf( "timer_delay_ms = %d\n", params.timer_delay_ms );
	printf( "goal_random_seed = %d\n", params.goal_random_seed );
	printf( "goal_width = %.2f\n", params.goal_width );
	printf( "goal_mass = %.2f\n", params.goal_mass );
	printf( "goal_quadrant = %d\n", params.goal_quadrant );
	printf( "agent_random_seed = %d\n", params.agent_random_seed );
	printf( "agent_number = %d\n", params.agent_number );
	printf( "deployment_width = %d\n", params.deployment_width );
	printf( "deployment_height = %d\n", params.deployment_height );
	printf( "deployment_quadrant = %d\n", params.deployment_quadrant );
	printf( "obstacle_random_seed = %d\n", params.obstacle_random_seed );
	printf( "obstacle_number = %d\n", params.obstacle_number );
	printf( "obstacle_radius = %.2f\n", params.obstacle_radius );
	printf( "obstacle_radius_min = %.2f\n", params.obstacle_radius_min );
	printf( "obstacle_radius_max = %.2f\n", params.obstacle_radius_max );
	printf( "obstacle_mass = %.2f\n", params.obstacle_mass );
	printf( "max_V = %.2f\n", params.max_V );
	printf( "G = %.2f\n", params.G );
	printf( "p = %.2f\n", params.p );
	/*****************************************************************/
	
	if ( mode == GUI )
	{
	    glutInit( &argc, argv );
	    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB );
	    glutInitWindowSize( params.world_width, params.world_height + help_area_height );
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
    
    if ( agents != NULL ) { free( obstacles ); }
    /*******************************************************/
    
	return EXIT_SUCCESS;
}
