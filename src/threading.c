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

#include <stdio.h>
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
    printf( "Threading system initialization: begin\n" );

    // set global thread attributes
    pthread_attr_init( &attr );
    pthread_attr_setscope( &attr, PTHREAD_SCOPE_SYSTEM );
    pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );

    // initialize mutexes and condition variable
    pthread_mutex_init( &mutex, NULL );
    pthread_mutex_init( &mutex_barrier, NULL );
    pthread_cond_init( &go, NULL );

    pthread_mutex_init( &mutex_system, NULL );
    pthread_cond_init( &cond_system, NULL );

    printf( "Threading system initialization: successful\n" );
}

pthread_t threads[MAX_THREADS];
pthread_attr_t attr;

pthread_mutex_t mutex_barrier;  // mutex for the barrier
pthread_cond_t go;              // condition variable for leaving
int at_barrier = 0;             // count of the number who have arrived

pthread_mutex_t mutex;          // mutex for statistics updates
int active_threads = 0;         // update statistics at the end of movement

pthread_mutex_t mutex_system;   // mutex for the cond_system
pthread_cond_t cond_system;     // condition variable for starting/stopping simulator

ThreadData thread_data[MAX_THREADS];
