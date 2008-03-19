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

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <gsl/gsl_rng.h>

#include "swarm.h"
#include "swarm_cli.h"

void run_cli( int argc, char **argv )
{
    // skip program name and view mode arguments
    int env_number = argc - 1;  
    char *environments[env_number];
    
    memcpy( environments, &argv[1], env_number * sizeof( char * ) );
    
    int e, n;
    
    for ( e = 0; e < env_number; ++e )
    {
        if ( load_scenario( environments[e] ) == -1 ) { exit( EXIT_FAILURE ); }

        char *raw_filename = NULL;
        
        if ( asprintf( &raw_filename, "raw_%s", params.results_filename ) < 0 )
        {
            printf( "ERROR (%s:%d): allocating memory failed!", __FILE__, __LINE__ );
            exit( EXIT_FAILURE );
        }
        
        FILE *p_results;
        FILE *p_raw_results;
        
        p_results = fopen( params.results_filename, "w+" );
        p_raw_results = fopen( raw_filename, "w+" );

        if ( p_results == NULL )
        {
            printf( "ERROR (%s:%d): Unable to open file [%s]!", __FILE__, __LINE__, params.results_filename );
            exit( EXIT_FAILURE );
        }
        
        if ( p_raw_results == NULL )
        {
            printf( "ERROR (%s:%d): Unable to open file [%s]!", __FILE__, __LINE__, raw_filename );
            exit( EXIT_FAILURE );
        }
        
        fprintf( p_raw_results, "reconstructed_%s\n", params.results_filename );
        fprintf( p_raw_results, "%d ", params.runs_number );
        fprintf( p_raw_results, "%d ", params.n_number );
        fprintf( p_raw_results, "%d ", params.k_number );
        fprintf( p_raw_results, "%d\n", params.a_b_number );
        
        output_simulation_parameters( p_results );
        fflush( p_results );

        for ( n = 0; n < params.n_number; ++n )
        {
            change_agent_number( params.n_array[n] );
            
            fprintf( p_raw_results, "%d\n", params.n_array[n] );            
            
            int i, j;
            
            /*************************** Calculate ground truth - big_P_prime ************************************/
            printf( "\n\nCalculating P' (ground truth from simulation)\n" );
            
            double big_P_prime[params.n_array[n] + 1];
            double increment = 1.0 / params.runs_number;

            // initialize big_P_prime array to all 0's
            memset( big_P_prime, 0, sizeof( big_P_prime ) );
            
            double small_p = 0.0;
    
            gsl_rng_set( agent_rng, ( unsigned int ) time( NULL ) );
            gsl_rng_set( general_rng, ( unsigned int ) time( NULL ) );
            
            for ( i = 0; i < params.runs_number; ++i )
            {
                if ( params.run_simulation )
                {
                    restart_simulation();
                
                    int agent;
                
                    for ( agent = 0; agent < params.agent_number; ++agent )
                    {
                        deploy_agent( agents[agent] );
                    }
                
                    while ( stats.reached_goal != params.agent_number && stats.time_step < params.time_limit )
                    {
                        move_agents();
                    }
                    
                    update_reach();
                }
                else
                {
                    reset_statistics();

                    for ( j = 0; j < params.agent_number; ++j )
                    {
                        double random = ( double ) gsl_rng_get( general_rng ) / ( double ) gsl_rng_max( general_rng );
                        if ( random <= params.env_probability ) { ++stats.reached_goal; }
                    }
                
                    stats.reach_ratio = ( float ) stats.reached_goal / ( float ) params.agent_number;
                }
                
                for ( j = 0; j <= stats.reached_goal; ++j )
                {
                    big_P_prime[j] += increment;
                }
                
                small_p += stats.reach_ratio;
                
                if ( i % 10 == 0 ) { printf( "\ti = %d, \tcurrent p = %.2f, \taverage p = %.2f\n", i, stats.reach_ratio, small_p / ( i + 1 ) ); }
            }
            
            small_p /= params.runs_number;
            
            printf( "P' calculation finished, p = %.2f\n\n", small_p );
            
            fprintf( p_raw_results, "%g\n", small_p );
            
            for ( i = 0; i <= params.n_array[n]; ++i )
            {
                fprintf( p_raw_results, "%g\n", big_P_prime[i] );
            }
            /*****************************************************************************************************/
            
            int ab, k;
            
            for ( ab = 0; ab < params.a_b_number; ++ab )
            {
                fprintf( p_raw_results, "%g ", params.alpha_array[ab] );
                fprintf( p_raw_results, "%g\n", params.beta_array[ab] );
    
                for ( k = 0; k < params.k_number; ++k )
                {
                    change_agent_number( params.k_array[k] );
                    
                    fprintf( p_raw_results, "%d\n", params.k_array[k] );
                                        
                    gsl_rng_set( agent_rng, ( unsigned int ) time( NULL ) );
                    gsl_rng_set( general_rng, ( unsigned int ) time( NULL ) );
                    
                    for ( j = 0; j < params.runs_number; ++j )
                    {
                        if ( params.run_simulation )
                        {
                            restart_simulation();
                        
                            int agent;
                        
                            for ( agent = 0; agent < params.agent_number; ++agent )
                            {
                                deploy_agent( agents[agent] );
                            }
                        
                            while ( stats.reached_goal != params.agent_number && stats.time_step < params.time_limit )
                            {
                                move_agents();
                            }
                        
                            update_reach();
                        }
                        else
                        {
                            reset_statistics();

                            int u;

                            for ( u = 0; u < params.agent_number; ++u )
                            {
                                double random = ( double ) gsl_rng_get( general_rng ) / ( double ) gsl_rng_max( general_rng );
                                if ( random <= small_p ) { ++stats.reached_goal; }
                            }

                            stats.reach_ratio = ( float ) stats.reached_goal / ( float ) params.agent_number;
                        }
                        
                        fprintf( p_raw_results, "%g\n", stats.reach_ratio );

                        // print current status/progress to standard output
                        if ( j % 100 == 0 )
                        {
                            printf( "j = %d, n = %d, k = %d, a = %.2f, b = %.2f\n",
                                     j, params.n_array[n], params.k_array[k], params.alpha_array[ab], params.beta_array[ab] );
                        }
                    }

                    fprintf( p_results, "\n\n" );
                    fflush( p_raw_results );
                }
            }            
        }
        
        fclose( p_results );
        fclose( p_raw_results );
    }
}

void print_usage( char *program_name )
{
    printf( "Robotic Swarm Simulator (C with GLUT/OpenGL) v0.5.0.\n" );
    printf( "Copyright (C) 2007, 2008 Antons Rebguns <anton at cs dot uwyo dot edu>\n\n" );
    printf( "Usage: %s [scenario_1, scenario_2, ...]\n\n", program_name );
    printf( "\tscenario_1, ... - one or more configuration files\n");
    printf( "\tNote: when using GUI mode only the first scenraio is used.\n" );
}

int main( int argc, char **argv )
{
    if ( argc < 2 )
    {
        print_usage( argv[0] );
        return EXIT_FAILURE;
    }
    
    run_cli( argc, argv );
    
    return EXIT_SUCCESS;
}
