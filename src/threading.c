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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "definitions.h"
#include "swarm.h"
#include "threading.h"

void barrier( void )
{
    pthread_mutex_lock( &mutex_barrier );

    ++at_barrier;

    if ( at_barrier == MAX_THREADS )
    {
        at_barrier = 0;
        pthread_cond_broadcast( &go );
    }
    else
    {
        pthread_cond_wait( &go, &mutex_barrier );
    }

    pthread_mutex_unlock( &mutex_barrier );
}

void initialize_threading( void )
{
    printf( "Begin threading system initialization\n" );

    // set global thread attributes
    pthread_attr_init( &attr );
    pthread_attr_setscope( &attr, PTHREAD_SCOPE_SYSTEM );
    pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );

    // initialize mutexes and condition variable
    pthread_mutex_init( &mutex, NULL );
    pthread_mutex_init( &mutex_barrier, NULL );
    pthread_cond_init( &go, NULL );

    pthread_barrier_init( &pt_barrier, NULL, MAX_THREADS );

    pthread_mutex_init( &mutex_system, NULL );
    pthread_cond_init( &cond_system, NULL );

    pthread_mutex_init( &mutex_finished, NULL );
    pthread_cond_init( &cond_finished, NULL );

    printf( "Threading system initialization successful\n" );
}

void create_update_threads( bool update_data_only )
{
    int i, j;

    // create threads and put on hold
    int extra = params.agent_number % MAX_THREADS;
    int num_per_cpu = ( params.agent_number - extra ) / MAX_THREADS;

    for ( i = 0; i < MAX_THREADS; ++i )
    {
        thread_data[i].thread_id = i;

        if ( i == MAX_THREADS - 1 ) { thread_data[i].agent_number = num_per_cpu + extra; }
        else { thread_data[i].agent_number = num_per_cpu; }

        if ( thread_data[i].agent_number != 0 )
        {
            if ( thread_data[i].agent_ids != NULL ) { free( thread_data[i].agent_ids ); }
            thread_data[i].agent_ids = calloc( thread_data[i].agent_number, sizeof(int) );
        }

        for ( j = 0; j < thread_data[i].agent_number; ++j )
        {
            thread_data[i].agent_ids[j] = i * num_per_cpu + j;
        }

        if ( !update_data_only ) { pthread_create( &threads[i], &attr, move_agents, (void *) &thread_data[i] ); }
    }

    pthread_attr_destroy(&attr);
}

pthread_t threads[MAX_THREADS];
pthread_attr_t attr;

pthread_mutex_t mutex_barrier;  // mutex for the barrier
pthread_cond_t go;              // condition variable for leaving
int at_barrier = 0;             // count of the number who have arrived

pthread_barrier_t pt_barrier;

pthread_mutex_t mutex;          // mutex for statistics updates
int active_threads = 0;         // update statistics at the end of movement

pthread_mutex_t mutex_system;   // mutex for the cond_system
pthread_cond_t cond_system;     // condition variable for starting/stopping simulator

pthread_mutex_t mutex_finished;  // mutex semaphore for the barrier
pthread_cond_t cond_finished;              // condition variable for leaving

ThreadData thread_data[MAX_THREADS];
