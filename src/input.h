/*
 ============================================================================
 Name        : input.h
 Author      : Antons Rebguns
 Version     : 0.4.1
 Copyright   : Copyright(c) 2007, 2008
 Description : Robotic swarm simulator (OpenGL)
 ============================================================================
 */

#ifndef INPUT_H_
#define INPUT_H_

void process_normal_keys( unsigned char key, int x, int y );
void process_special_keys( int key, int x, int y );
void process_mouse_buttons( int button, int state, int x, int y );
void process_mouse_entry( int state );
void process_mouse_active_motion( int x, int y );

#endif /*INPUT_H_*/
