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

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <stdbool.h>

#define VERSION "v0.6.0"

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
    float x;
    float y;
    
} Vector2f;

/**
 * \struct Agent
 * \brief  Represents a robot.
 */
typedef struct s_agent
{
    int id;
    
    Vector2f i_position;    // Agent deployment (initial) position
    Vector2f i_velocity;    // Agent initial velocity
    
    Vector2f position;      // Agent current position vector
    Vector2f velocity;      // Agent current velocity vector

    Vector2f n_position;    // Agent new (for lock step) position
    Vector2f n_velocity;    // Agent new (for lock step) velocity

    float radius;           // Agent size
    float mass;             // Agent mass
    
    bool collided;          // true, if agent collided with obstacle
    bool goal_reached;      // true, if reached goal, false otherwise
    
    float color[3];
    
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

    float color[3];
    
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
    
    float color[3];
    
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
    
    int goal_random_seed;
    float goal_width;
    float goal_mass;
    Quadrant goal_quadrant;
    
    int agent_random_seed;
    int agent_number;
    float agent_radius;
    float agent_mass;
    int deployment_width;
    int deployment_height;
    Quadrant deployment_quadrant;    
    
    int obstacle_random_seed;
    int obstacle_number;
    float obstacle_radius;
    float obstacle_radius_min;
    float obstacle_radius_max;
    float obstacle_mass;
    
    bool enable_agent_goal_f;
    bool enable_agent_obstacle_f;
    bool enable_agent_agent_f;
    
    float R;
    float friction_coefficient;
    float range_coefficient;
    float max_V; 
    ForceLaw force_law;
    
    float G_agent_agent;            // Newtonian - Gravitational force of agent-agent interactions
    float G_agent_obstacle;         // Newtonian - Gravitational force of agent-obstacle interactions
    float G_agent_goal;             // Newtonian - Gravitational force of agent-goal interactions
    
    float p_agent_agent;            // Newtonian - (distance_between_objects) ^ p of agent-agent interactions
    float p_agent_obstacle;         // Newtonian - (distance_between_objects) ^ p of agent-obstacle interactions
    float p_agent_goal;             // Newtonian - (distance_between_objects) ^ p of agent-goal interactions
    
    float max_f_agent_agent_n;      // Newtonian - agent-agent force cutoff
    float max_f_agent_obstacle_n;   // Newtonian - agent-obstacle force cutoff
    float max_f_agent_goal_n;       // Newtonian - agent-goal force cutoff
    
    float epsilon_agent_agent;      // LJ - Strength of agent-agent interactions
    float epsilon_agent_obstacle;   // LJ - Strength of agent-obstacle interactions
    float epsilon_agent_goal;       // LJ - Strength of agent-goal interactions

    float c_agent_agent;            // LJ - Attractive agent-agent parameter
    float c_agent_obstacle;         // LJ - Attractive agent-obstacle parameter
    float c_agent_goal;             // LJ - Attractive agent-goal parameter

    float d_agent_agent;            // LJ - Repulsive agent-agent parameter
    float d_agent_obstacle;         // LJ - Repulsive agent-obstacle parameter
    float d_agent_goal;             // LJ - Repulsive agent-goal parameter
    
    float max_f_agent_agent_lj;     // LJ - agent-agent force cutoff
    float max_f_agent_obstacle_lj;  // LJ - agent-obstacle force cutoff
    float max_f_agent_goal_lj;      // LJ - agent-goal force cutoff
    
    int time_limit;
    int runs_number;
    bool run_simulation;
    float env_probability;
    
    bool initialize_from_file;
    char *scenario_filename;
    char *results_filename;
    
    int n_number;
    int k_number;
    int a_b_number;
    int *n_array;
    int *k_array;
    float *alpha_array;
    float *beta_array;
    
} Parameters;

typedef struct s_statistics
{
    int time_step;
    int reached_goal;
    float reach_ratio;
    int collisions;
    float collision_ratio;
    
} Statistics;

extern float agent_color[3];
extern float agent_color_coll[3];
extern float agent_color_conn[3];

extern float goal_color[3];
extern float obstacle_color[3];

extern Parameters params;
extern Statistics stats;

extern Agent **agents;
extern Obstacle **obstacles;
extern Goal *goal;

extern bool ( *agent_reached_goal )( Agent * );

extern bool running;

#endif /*DEFINITIONS_H_*/
