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

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "analysis.h"

/*************** TEMPORARY *************************/
double alpha;
double beta;
double ppHat;
int yy;
int kk;
int nn;
/***************************************************/

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
   
    for ( i = 0; i < n; ++i )
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
    
    for ( j = 0; j <= 5; ++j )
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
    
    double r = k * pHat + a;
    double s = k * ( 1 - pHat ) + b;
    
    double b1 = beta_function( r, s );
    double b2 = beta_function( y, n - y + 1 );
    double bb = b1 * b2;
    
    double C = pow( bb, -1 );

    if ( isinf( C ) == 1 ) { C = DBL_MAX; }
    else if ( isinf( C ) == -1 ) { C = DBL_MIN; }
    
    return C * pow( p, r - 1 ) * pow( 1 - p, s - 1 ) * gaussian_quadrature( 0.0, p, 100, incomplete_beta );
}

/****************************************************************************************/

/*
 * big_P_prime - "ground truth" obtained by running the simulation with
 *               agent_number number of agents runs_number number of times.
 * big_P       - approximation of big_P_prime obtained using Bernoulli trials.
 * big_P_hat   - approximation of big_P_prime obtained using proposed formula.
 */
void analyze( int argc, char **argv )
{
    // skip program name
    int env_number = argc - 1;    
    char *raw_filenames[env_number];
    
    memcpy( raw_filenames, &argv[1], env_number * sizeof( char * ) );
    
    // How accurate is our integral approximation
    int interval_number = 100;
    
    int runs_number = 0;
    int n_number = 0;
    int k_number = 0;
    int a_b_number = 0;
    
    float reach_ratio = 0.0f;
    
    int e, n;
    
    for ( e = 0; e < env_number; ++e )
    {
        char *raw_filename = raw_filenames[e];
        
        FILE *p_raw_results;        
        p_raw_results = fopen( raw_filename, "r" );
        
        if ( p_raw_results == NULL )
        {
            printf( "ERROR: Unable to open file [%s]!", raw_filename );
            exit( EXIT_FAILURE );
        }
        
        char *results_filename = NULL;
        
        fscanf( p_raw_results, "%as", &results_filename );

        FILE *p_results;
        p_results = fopen( results_filename, "w+" );

        if ( p_results == NULL )
        {
            printf( "ERROR: Unable to open file [%s]!", results_filename );
            exit( EXIT_FAILURE );
        }
        
        fscanf( p_raw_results, "%d", &runs_number );
        fscanf( p_raw_results, "%d", &n_number );
        fscanf( p_raw_results, "%d", &k_number );
        fscanf( p_raw_results, "%d", &a_b_number );
        
        int mean_diff_P[4] = { 0 };
        int mean_diff_P_hat[4] = { 0 };
        int means_overlap[2] = { 0 };
        
        int index = 0;
        int counter = 1;

        for ( n = 0; n < n_number; ++n )
        {
            fscanf( p_raw_results, "%d", &nn );            

            int i, j;
            
            /*************************** Calculate ground truth - big_P_prime ************************************/
            printf( "Calculating P' (ground truth from simulation)\n" );
            
            double big_P_prime[nn + 1];
            double small_p = 0.0;
    
            fscanf( p_raw_results, "%lf", &small_p );
            
            for ( i = 0; i <= nn; ++i )
            {
                fscanf( p_raw_results, "%lf", &( big_P_prime[i] ) );
            }
            
            printf( "P' calculation finished, p = %.2f\n\n", small_p );
            /*****************************************************************************************************/
            
            int ab, k, y, l;
            
            for ( ab = 0; ab < a_b_number; ++ab )
            {
                fscanf( p_raw_results, "%lf", &alpha );
                fscanf( p_raw_results, "%lf", &beta );
    
                for ( k = 0; k < k_number; ++k )
                {
                    fscanf( p_raw_results, "%d", &kk );
                    
                    fprintf( p_results, "#index %d, small_p = %f, n = %d, k = %d, alpha = %.2f, beta = %.2f\n", index, small_p, nn, kk, alpha, beta );
                    fprintf( p_results, "#n\t\t\tk\t\t\ty" );
                    fprintf( p_results, "\t\t\tbig_P_prime\t\t\tbig_P_mean\t\t\tbig_P_hat_mean\t\t\tbig_P_hat_plus_mean" );
                    fprintf( p_results, "\t\terror1\t\t\t\t\terror2\t\t\t\terror3" );
                    fprintf( p_results, "\t\t\t\tbig_P_std\t\t\tbig_P_hat_std\t\tbig_P_hat_plus_std" );
                    fprintf( p_results, "\tbig_P_var\t\t\tbig_P_hat_var\t\tbig_P_hat_plus_var" );
                    fprintf( p_results, "\t\tbig_P_bias\t\tbig_P_hat_bias\t\tbig_P_hat_plus_bias" );
                    fprintf( p_results, "\t\tbig_P_mse\t\tbig_P_hat_mse\t\tbig_P_hat_plus_mse\n" );
                    
                    double *big_P_array;
                    double *big_P_hat_array;
                    double *big_P_hat_plus_array;

                    big_P_array = ( double * ) calloc( ( nn + 1 ) * runs_number, sizeof( double ) );
                    big_P_hat_array = ( double * ) calloc( ( nn + 1 ) * runs_number, sizeof( double ) );
                    big_P_hat_plus_array = ( double * ) calloc( ( nn + 1 ) * runs_number, sizeof( double ) );

                    if ( big_P_array == NULL )
                    {
                        printf( "ERROR: allocating memory for big_P_array failed!" );
                        exit( EXIT_FAILURE );
                    }
                    
                    if ( big_P_hat_array == NULL )
                    {
                        printf( "ERROR: allocating memory for big_P_hat_array failed!" );
                        exit( EXIT_FAILURE );
                    }

                    if ( big_P_hat_plus_array == NULL )
                    {
                        printf( "ERROR: allocating memory for big_P_hat_plus_array failed!" );
                        exit( EXIT_FAILURE );
                    }
                    
                    double big_P_mean[nn + 1];
                    double big_P_hat_mean[nn + 1];
                    double big_P_hat_plus_mean[nn + 1];
                    
                    // initialize big_P_mean and big_P_hat_mean arrays to all 0's
                    memset( big_P_mean, 0, sizeof( big_P_mean ) );
                    memset( big_P_hat_mean, 0, sizeof( big_P_hat_mean ) );
                    memset( big_P_hat_plus_mean, 0, sizeof( big_P_hat_plus_mean ) );

                    for ( j = 0; j < runs_number; ++j )
                    {
                        fscanf( p_raw_results, "%f", &reach_ratio );
                        
                        for ( y = 1; y <= nn; ++y )
                        {
                            /***************** Calculate big_P ***************************************************************/
                            double big_P = 0.0;

                            for ( l = y; l <= nn; ++l )
                            {
                                // exp( gammaln( n + 1 ) ) == fac( n )
                                double n_choose_l =  exp( gammaln( nn + 1.0 ) - gammaln( l + 1.0 ) - gammaln( nn - l + 1.0 ) );
                                big_P += n_choose_l * pow( reach_ratio, l ) * pow( 1.0 - reach_ratio, nn - l );
                            }
                            
                            if ( big_P > 1.0 ) { big_P = 1.0; }
                            /*************************************************************************************************/
                            
                            /***************** Calculate big_P_hat ***********************************************************/
                            yy = y;
                            ppHat = reach_ratio;
                            
                            double big_P_hat = gaussian_quadrature( 0.0f, 1.0f, interval_number, f );
                            if ( big_P_hat > 1.0 ) { big_P_hat = 1.0; }
                            /*************************************************************************************************/
                            
                            /***************** Calculate big_P_hat_plus ******************************************************/
                            double big_P_hat_plus = gaussian_quadrature( 0.0f, 1.0f, interval_number, f );
                            if ( big_P_hat_plus > 1.0 ) { big_P_hat_plus = 1.0; }
                            /*************************************************************************************************/
                                                        
                            big_P_array[y * runs_number + j] = big_P;
                            big_P_hat_array[y * runs_number + j] = big_P_hat;
                            big_P_hat_plus_array[y * runs_number + j] = big_P_hat_plus;
                            
                            big_P_mean[y] += big_P / runs_number;
                            big_P_hat_mean[y] += big_P_hat / runs_number;
                            big_P_hat_plus_mean[y] += big_P_hat_plus / runs_number;
                        }
                        
                        if ( j % 100 == 0 ) { printf( "j = %d, n = %d, k = %d\n", j, nn, kk ); }
                    }
                    
                    for ( y = 1; y <= nn; ++y )
                    {
                        double big_P_sum = 0.0;
                        double big_P_hat_sum = 0.0;
                        double big_P_hat_plus_sum = 0.0;
                        
                        for ( j = 0; j < runs_number; ++j )
                        {
                            big_P_sum += pow( big_P_array[y * runs_number + j] - big_P_mean[y], 2.0 );
                            big_P_hat_sum += pow( big_P_hat_array[y * runs_number + j] - big_P_hat_mean[y], 2.0 );
                            big_P_hat_plus_sum += pow( big_P_hat_plus_array[y * runs_number + j] - big_P_hat_plus_mean[y], 2.0 );
                        }
                        
                        // calculate standard deviation
                        double big_P_std_dev = sqrt( ( 1.0 / runs_number ) * big_P_sum );
                        double big_P_hat_std_dev = sqrt( ( 1.0 / runs_number ) * big_P_hat_sum );
                        double big_P_hat_plus_std_dev = sqrt( ( 1.0 / runs_number ) * big_P_hat_plus_sum );

                        // calculate variance 
                        double big_P_var = pow( big_P_std_dev, 2.0 );
                        double big_P_hat_var = pow( big_P_hat_std_dev, 2.0 );
                        double big_P_hat_plus_var = pow( big_P_hat_plus_std_dev, 2.0 );
                        
                        /**************************************************** Calculate bias ***************************************************************/
                        double big_P_bias = 0.0;
                        double big_P_hat_bias = 0.0;
                        double big_P_hat_plus_bias = 0.0;
                        
                        int s;
                        
                        for ( s = 0; s <= kk; ++s )
                        {
                            double small_p_hat = ( double ) s / ( double ) kk;
                            
                            /***************** Calculate big_P *********************************************************************************************/
                            double big_P = 0.0;

                            for ( l = y; l <= nn; ++l )
                            {
                                // exp( gammaln( n + 1 ) ) == fac( n )
                                double n_choose_l =  exp( gammaln( nn + 1.0 ) - gammaln( l + 1.0 ) - gammaln( nn - l + 1.0 ) );
                                big_P += n_choose_l * pow( small_p_hat, l ) * pow( 1.0 - small_p_hat, nn - l );
                            }
                            
                            if ( big_P > 1.0 ) { big_P = 1.0; }
                            /*******************************************************************************************************************************/
                            
                            /***************** Calculate big_P_hat *****************************************************************************************/
                            yy = y;
                            ppHat = small_p_hat;
                            
                            double big_P_hat = gaussian_quadrature( 0.0f, 1.0f, interval_number, f );
                            if ( big_P_hat > 1.0 ) { big_P_hat = 1.0; }
                            /*******************************************************************************************************************************/
                            
                            // TODO: new formula for calculating P^+
                            /***************** Calculate big_P_hat_plus ************************************************************************************/
                            double big_P_hat_plus = gaussian_quadrature( 0.0f, 1.0f, interval_number, f );
                            if ( big_P_hat_plus > 1.0 ) { big_P_hat_plus = 1.0; }
                            /*******************************************************************************************************************************/
                                                        
                            double k_choose_s = exp( gammaln( kk + 1.0 ) - gammaln( s + 1.0 ) - gammaln( kk - s + 1.0 ) );
                            double bernoulli = k_choose_s * pow( small_p, s ) * pow( 1.0 - small_p, kk - s );
                            
                            big_P_bias += big_P * bernoulli;
                            big_P_hat_bias += big_P_hat * bernoulli;
                            big_P_hat_plus_bias += big_P_hat_plus * bernoulli;
                        }
                        
                        big_P_bias -= big_P_prime[y];
                        big_P_hat_bias -= big_P_prime[y];
                        big_P_hat_plus_bias -= big_P_prime[y];
                        
                        double big_P_mse = pow( big_P_bias, 2.0 ) + big_P_var;
                        double big_P_hat_mse = pow( big_P_hat_bias, 2.0 ) + big_P_hat_var;
                        double big_P_hat_plus_mse = pow( big_P_hat_plus_bias, 2.0 ) + big_P_hat_plus_var;
                        /***********************************************************************************************************************************/
                        
                        double error1 = big_P_mean[y] - big_P_prime[y];
                        double error2 = big_P_hat_mean[y] - big_P_prime[y];
                        double error3 = big_P_hat_plus_mean[y] - big_P_prime[y];

                        fprintf( p_results, "%d\t\t\t%d\t\t\t%d", nn, kk, y );
                        fprintf( p_results, "\t\t\t%f\t\t\t%f\t\t\t%f\t\t\t\t%f", big_P_prime[y], big_P_mean[y], big_P_hat_mean[y], big_P_hat_plus_mean[y] );
                        fprintf( p_results, "\t\t\t\t%f\t\t\t%f\t\t\t%f", error1, error2, error3 );
                        fprintf( p_results, "\t\t\t%f\t\t\t%f\t\t\t%f", big_P_std_dev, big_P_hat_std_dev, big_P_hat_plus_std_dev );
                        fprintf( p_results, "\t\t\t%f\t\t\t%f\t\t\t%f", big_P_var, big_P_hat_var, big_P_hat_plus_var );
                        fprintf( p_results, "\t\t%f\t\t%f\t\t%f", big_P_bias, big_P_hat_bias, big_P_hat_plus_bias );
                        fprintf( p_results, "\t\t%f\t\t%f\t\t%f\n", big_P_mse, big_P_hat_mse, big_P_hat_plus_mse );
                        
                        int diff_index = INT_MAX;
                        
                        if ( big_P_std_dev != 0.0 ) { diff_index = floor( fabs( error1 / big_P_std_dev ) ); }
                        
                        // diff_index of 0 = means are within 1 P std deviation
                        if ( diff_index < 3 ) { ++mean_diff_P[diff_index]; }
                        else { ++mean_diff_P[3]; }
                        
                        diff_index = INT_MAX;
                        
                        if ( big_P_hat_std_dev != 0.0 ) { diff_index = floor( fabs( error2 / big_P_hat_std_dev ) ); }
                        
                        // diff_index of 0 = means are within 1 P-hat std deviation
                        if ( diff_index < 3 ) { ++mean_diff_P_hat[diff_index]; }
                        else { ++mean_diff_P_hat[3]; }
                        
                        if ( ( big_P_mean[y] + big_P_std_dev >= big_P_hat_mean[y] + big_P_hat_std_dev &&
                               big_P_mean[y] - big_P_std_dev <= big_P_hat_mean[y] + big_P_hat_std_dev ) ||
                             ( big_P_mean[y] + big_P_std_dev <= big_P_hat_mean[y] + big_P_hat_std_dev &&
                               big_P_mean[y] - big_P_std_dev >= big_P_hat_mean[y] + big_P_hat_std_dev ) )
                        {
                            ++means_overlap[0];
                        }
                        else
                        {
                            ++means_overlap[1];
                        }
                        
                        ++counter;
                    }

                    fprintf( p_results, "\n\n" );
                    fflush( p_results );
                    fflush( p_raw_results );

                    ++index;
                    
                    if ( big_P_array != NULL ) { free( big_P_array ); }
                    if ( big_P_hat_array != NULL ) { free( big_P_hat_array ); }
                    if ( big_P_hat_plus_array != NULL ) { free( big_P_hat_plus_array ); }
                }
            }
            
            printf( "\n" );

            for ( j = 0; j < 3; ++j )
            {
                printf( "%.2f per cent of the time means are within %d P stdandard deviation(s)\n", ( ( double ) mean_diff_P[j] / ( double ) counter ), j + 1 );
                printf( "%.2f per cent of the time means are within %d P-hat stdandard deviation(s)\n", ( ( double ) mean_diff_P_hat[j] / ( double ) counter ), j + 1 );
            }
            
            printf( "%.2f per cent of the time means are within %d or more P stdandard deviation(s)\n", ( ( double ) mean_diff_P[3] / ( double ) counter ), 4 );
            printf( "%.2f per cent of the time means are within %d or more P-hat stdandard deviation(s)\n\n", ( ( double ) mean_diff_P_hat[3] / ( double ) counter ), 4 );
            
            printf( "%.2f per cent of the time means are overlapping\n", ( ( double ) means_overlap[0] / ( double ) counter ) );
            printf( "\n\n" );
        }
        
        int j;
        
        for ( j = 0; j < 3; ++j )
        {
            fprintf( p_results, "%.2f per cent of the time means are within %d P stdandard deviation(s)\n", ( ( double ) mean_diff_P[j] / ( double ) counter ), j + 1 );
        }
        
        fprintf( p_results, "%.2f per cent of the time means are within %d or more P stdandard deviation(s)\n", ( ( double ) mean_diff_P[3] / ( double ) counter ), 4 );
        
        for ( j = 0; j < 3; ++j )
        {
            fprintf( p_results, "%.2f per cent of the time means are within %d P-hat stdandard deviation(s)\n", ( ( double ) mean_diff_P_hat[j] / ( double ) counter ), j + 1 );
        }
        
        fprintf( p_results, "%.2f per cent of the time means are within %d or more P-hat stdandard deviation(s)\n", ( ( double ) mean_diff_P_hat[3] / ( double ) counter ), 4 );
        fprintf( p_results, "\n\n" );
        fprintf( p_results, "%.2f per cent of the time means are overlapping\n", ( ( double ) means_overlap[0] / ( double ) counter ) );
        
        fclose( p_results );
        fclose( p_raw_results );
    }
}

void print_usage( char *program_name )
{
    printf( "Robotic Swarm Simulator (C with GLUT/OpenGL) v0.5.0.\n" );
    printf( "Copyright (C) 2007, 2008 Antons Rebguns <anton at cs dot uwyo dot edu>\n\n" );
    printf( "Usage: %s [raw_filename_1, raw_filename_2, ...]\n\n", program_name );
    printf( "\traw_filename_1, ... - one or more raw data files\n");
}

int main( int argc, char **argv )
{
    if ( argc < 2 )
    {
        print_usage( argv[0] );
        return EXIT_FAILURE;
    }
    
    analyze( argc, argv );
    
    return EXIT_SUCCESS;
}
