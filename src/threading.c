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
#include "queue.h"

void initialize_threading( void )
{
    printf( "Begin threading system initialization\n" );

    // set global thread attributes
    pthread_attr_init( &attr );
    pthread_attr_setscope( &attr, PTHREAD_SCOPE_SYSTEM );
    pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );

    pthread_barrier_init( &lock_step_barrier, NULL, MAX_THREADS );

    pthread_mutex_init( &mutex_system, NULL );
    pthread_cond_init( &cond_system, NULL );

    pthread_mutex_init( &mutex_finished, NULL );
    pthread_cond_init( &cond_finished, NULL );

    Q_Init( &thread_task_pool );

    printf( "Threading system initialization successful\n" );
}

void create_update_threads( bool update_data_only )
{
    int i;

    for ( i = 0; i < params.agent_number; ++i )
    {
        ThreadTask *t;
        t = calloc( 1, sizeof( ThreadTask ) );
        t->agent_id = i;
        Q_PushHead( &thread_task_pool, t );
    }

    for ( i = 0; i < MAX_THREADS; ++i )
    {
        thread_data[i].thread_id = i;
        thread_data[i].agent_ids = (int *) realloc( thread_data[i].agent_ids, params.agent_number * sizeof( int ) );
        thread_data[i].agent_number = 0;

        if ( !update_data_only ) { pthread_create( &threads[i], &attr, move_agents, (void *) &thread_data[i] ); }
    }

    pthread_attr_destroy( &attr );
}

pthread_t threads[MAX_THREADS];
pthread_attr_t attr;

pthread_barrier_t lock_step_barrier;   // lock step barrier

pthread_mutex_t mutex;                 // mutex for statistics updates
int active_threads = 0;                // update statistics at the end of movement

pthread_mutex_t mutex_system;          // mutex for the cond_system
pthread_cond_t cond_system;            // condition variable for starting/stopping simulator

pthread_mutex_t mutex_finished;        // mutex semaphore for signaling swarm_cli to continue
pthread_cond_t cond_finished;          // condition variable for signaling swarm_cli to continue

ThreadData thread_data[MAX_THREADS];
queue thread_task_pool;
