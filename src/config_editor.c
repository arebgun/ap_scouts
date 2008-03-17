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

#include <gtk-2.0/gtk/gtk.h>

static gboolean delete_event( GtkWidget*, GdkEvent*, gpointer );

int main( int argc, char *argv[] )
{
    gtk_init (&argc, &argv);

    GtkWidget *window_main;
    GtkWidget *window_scroll;
    
    GtkWidget *vbox_main;
    
    GtkWidget *expander_world;
    GtkWidget *expander_goal;
    GtkWidget *expander_agent;
    GtkWidget *expander_obstacle;
    GtkWidget *expander_physics;
    GtkWidget *expander_newton;
    GtkWidget *expander_lennard;
    GtkWidget *expander_batch;
    
    window_main = gtk_window_new( GTK_WINDOW_TOPLEVEL );
    
    gtk_window_set_title( GTK_WINDOW( window_main ), "Configuration Editor" );
    gtk_container_set_border_width( GTK_CONTAINER( window_main ), 5 );
    gtk_widget_set_size_request( window_main, 400, 200 );
    
    window_scroll = gtk_scrolled_window_new( NULL, NULL );
    gtk_scrolled_window_set_policy( GTK_SCROLLED_WINDOW( window_scroll), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC );
    gtk_container_set_border_width( GTK_CONTAINER( window_scroll ), 5 );
    
    vbox_main = gtk_vbox_new( TRUE, 0 );
    
    expander_world    = gtk_expander_new( "World Parameters" );
    expander_goal     = gtk_expander_new( "Goal Parameters" );
    expander_agent    = gtk_expander_new( "Agent Parameters" );
    expander_obstacle = gtk_expander_new( "Obstacle Parameters" );
    expander_physics  = gtk_expander_new( "General Physics Parameters" );
    expander_newton   = gtk_expander_new( "Newtonian Physics Parameters" );
    expander_lennard  = gtk_expander_new( "Lennard-Jones Physics Parameters" );
    expander_batch    = gtk_expander_new( "Batch Processing Parameters" );
    
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_world );
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_goal );
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_agent );
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_obstacle );
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_physics );
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_newton );
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_lennard );
    gtk_box_pack_start_defaults( GTK_BOX( vbox_main ), expander_batch );
    
    /* Connect the main window to the destroy and delete-event signals. */
    g_signal_connect( G_OBJECT( window_main ), "destroy",
                      G_CALLBACK( gtk_main_quit ), NULL );
    g_signal_connect( G_OBJECT( window_main ), "delete_event",
                      G_CALLBACK( delete_event ), NULL );
    
    /* Add the label as a child widget of the window. */
    gtk_scrolled_window_add_with_viewport( GTK_SCROLLED_WINDOW( window_scroll ), vbox_main );
    gtk_container_add( GTK_CONTAINER( window_main ), window_scroll );
    gtk_widget_show_all( window_main );
    gtk_main();
    
    return 0;
}

/*
 * Return FALSE to destroy the widget. By returning TRUE, you can cancel
 * a delete-event. This can be used to confirm quitting the application.
 */
static gboolean delete_event( GtkWidget *window, GdkEvent *event, gpointer data )
{
    return FALSE;
}
