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

#include <pthread.h>
#include <stdlib.h>

#include <GL/glut.h>

#include "graphics.h"
#include "input.h"
#include "swarm.h"
#include "threading.h"

void process_normal_keys( unsigned char key, int x, int y )
{
    if ( key == 's' || key == 'S' )
    {
        running = !running;

        if ( running )
        {
            pthread_mutex_lock( &mutex_system );
            pthread_cond_broadcast( &cond_system );
            pthread_mutex_unlock( &mutex_system );
        }
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
        cur_sel_index = 0;        // AGENT
        glutPostRedisplay();
    }
    else if ( key == 'o' || key == 'O' )
    {
        cur_sel_index = 1;        // OBSTACLE
        glutPostRedisplay();
    }
    else if ( key == 'd' )
    {
        if ( save_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
        glutPostRedisplay();
    }
    else if ( key == 'D' )
    {
        if ( !params.initialize_from_file )
        {
            params.initialize_from_file = true;
            if ( save_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
            params.initialize_from_file = false;
        }
        else
        {
            if ( save_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
        }
        glutPostRedisplay();
    }
    else if ( key == 'l' )
    {
        if ( load_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
        glutPostRedisplay();
    }
    else if ( key == 'L' )
    {
        if ( !params.initialize_from_file )
        {
            params.initialize_from_file = true;
            if ( load_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
            params.initialize_from_file = false;
        }
        else
        {
            if ( load_scenario( "scenario.dat" ) != 0 ) { exit( EXIT_FAILURE ); }
        }
        glutPostRedisplay();
    }
    else if ( key == 'c' || key == 'C')
    {
        show_connectivity = show_connectivity ? false : true;
        glutPostRedisplay();
    }
    else if ( key == 'q' || key == 'Q' )
    {
        exit( EXIT_SUCCESS );
    }
}

void process_special_keys( int key, int x, int y )
{
    switch ( key )
    {
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

void process_mouse_buttons( int button, int state, int x, int y )
{
    if ( state == GLUT_DOWN )
    {
        if ( button == GLUT_LEFT_BUTTON )
        {
            x = x - stats_area_width;
            y = params.world_height - y;

            int i;

            for ( i = 0; i < params.obstacle_number; ++i )
            {
                int radius = obstacles[i]->radius;

                float x_o = obstacles[i]->position.x;
                float y_o = obstacles[i]->position.y;

                if ( ( x >= x_o - radius ) && ( x <= x_o + radius ) &&
                     ( y >= y_o - radius ) && ( y <= y_o + radius ) )
                {
                    inside_window = true;
                    selected_obstacle_id = i;
                    selection_active = true;
                    break;
                }
            }
        }
    }
    else
    {
        inside_window = false;
        selected_obstacle_id = -1;
        selection_active = false;
    }
}

void process_mouse_entry( int state )
{
    if ( state == GLUT_LEFT ) { inside_window = false; }
    else { inside_window = true; }
}

void process_mouse_active_motion( int x, int y )
{
    if ( selection_active && selected_obstacle_id != -1 && inside_window )
    {
        Obstacle *obs = obstacles[selected_obstacle_id];
        Vector2f *obs_pos = &( obs->position );

        obs_pos->x = x - stats_area_width;
        obs_pos->y = params.world_height - y;

        // prevent moving obstacle to the information and statistics area
        if ( obs_pos->x - obs->radius < 0.0f ) { obs_pos->x = obs->radius; }
        if ( obs_pos->y - obs->radius < 0.0f ) { obs_pos->y = obs->radius; }

        glutPostRedisplay();
    }
}
