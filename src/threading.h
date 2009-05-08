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

#ifndef THREADING_H_
#define THREADING_H_

#include <pthread.h>
#include <stdbool.h>

#include "queue.h"

#define MAX_THREADS 2

typedef struct s_thread_data
{
    int thread_id;      // current thread id
    int *agent_ids;     // agent IDs to be processed by this thread
    int agent_number;   // total number of agents to be processed by this thread

} ThreadData;

typedef struct s_thread_task
{
    int agent_id;

} ThreadTask;

void initialize_threading( void );
void create_update_threads( bool update_data_only );

extern pthread_t threads[MAX_THREADS];
extern pthread_attr_t attr;

extern pthread_barrier_t lock_step_barrier;   // lock step barrier

extern pthread_mutex_t mutex;                 // mutex for statistics updates
extern int active_threads;                    // update statistics at the end of movement

extern pthread_mutex_t mutex_system;          // mutex for the cond_system
extern pthread_cond_t cond_system;            // condition variable for starting/stopping simulator

extern pthread_mutex_t mutex_finished;        // mutex semaphore for signaling swarm_cli to continue
extern pthread_cond_t cond_finished;          // condition variable for signaling swarm_cli to continue

extern ThreadData thread_data[MAX_THREADS];
extern queue thread_task_pool;

#endif /* THREADING_H_ */
