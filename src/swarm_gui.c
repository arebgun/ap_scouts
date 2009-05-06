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

#include "GL/gl.h"
#include "GL/glut.h"

#include "graphics.h"
#include "input.h"
#include "swarm.h"
#include "swarm_gui.h"
#include "threading.h"

void run_gui( int time )
{
    if ( running )
    {
        if ( stats.time_step >= params.time_limit )
        {
            update_reach();
            running = false;
        }

        glutPostRedisplay();
    }

    glutTimerFunc( 0, run_gui, stats.time_step );
}

void print_usage( char *program_name )
{
    printf( "Robotic Swarm Simulator (C with GLUT/OpenGL) %s.\n", VERSION );
    printf( "Copyright (C) 2007, 2008 Antons Rebguns <anton at cs dot uwyo dot edu>\n\n" );
    printf( "Usage: %s [scenario_1, scenario_2, ...]\n\n", program_name );
    printf( "\tscenario_1, ... - one or more configuration files\n");
    printf( "\tNote: when using GUI mode only the first scenario is used.\n" );
}

int main( int argc, char **argv )
{
    if ( argc < 2 )
    {
        print_usage( argv[0] );
        return EXIT_FAILURE;
    }

    if ( load_scenario( argv[1] ) != 0 ) { return EXIT_FAILURE; }

    initialize_threading();

    // create threads and put on hold
    int extra = params.agent_number % MAX_THREADS;
    int num_per_cpu = ( params.agent_number - extra ) / MAX_THREADS;

    int i, j;

    for ( i = 0; i < MAX_THREADS; ++i )
    {
        thread_data[i].thread_id = i;

        if ( i == MAX_THREADS - 1 ) { thread_data[i].agent_number = num_per_cpu + extra; }
        else { thread_data[i].agent_number = num_per_cpu; }

        thread_data[i].agent_ids = calloc( thread_data[i].agent_number, sizeof(int) );

        for ( j = 0; j < thread_data[i].agent_number; ++j )
        {
            thread_data[i].agent_ids[j] = i * num_per_cpu + j;
        }

        printf( "Creating thread %d: begin\n", i );
        pthread_create( &threads[i], &attr, move_agents, (void *) &thread_data[i] );
        printf( "Creating thread %d: end\n", i );
    }

    pthread_attr_destroy( &attr );

    printf( "Thread creation complete.\n" );

    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB );
    glutInitWindowSize( params.world_width + stats_area_width, params.world_height + help_area_height );
    glutInitWindowPosition( 100, 100 );
    glutCreateWindow( "Robotic Swarm Simulation" );

    initialize_graphics();

    glutDisplayFunc( draw_all );

    glutKeyboardFunc( process_normal_keys );
    glutSpecialFunc( process_special_keys );

    glutMouseFunc( process_mouse_buttons );
    glutEntryFunc( process_mouse_entry );
    glutMotionFunc( process_mouse_active_motion );

    glutTimerFunc( 0, run_gui, stats.time_step );

    glutMainLoop();

    return EXIT_SUCCESS;
}
