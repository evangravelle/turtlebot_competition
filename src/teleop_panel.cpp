/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QSignalMapper>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

#include "drive_widget.h"
#include "teleop_panel.h"

#include <states.h>
#include <coconuts_common/ControlState.h>
#include <coconuts_common/ArmMovement.h>

namespace coconuts_rviz_plugins
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
//
// #include <states.h>
// #include <coconuts_common/ControlState.h>
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , behavior_state_(INIT)
  , behavior_sub_state_(DEFAULT_SUB_STATE)
{

  // Mapping object for passing button values
  QSignalMapper * control_mapper = new QSignalMapper(this);
  QSignalMapper * control_sub_mapper = new QSignalMapper(this);
  QSignalMapper * arm_mapper = new QSignalMapper(this);
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );
  currentStateLabel = new QLabel("DEFAULT");
  currentSubStateLabel = new QLabel("DEFAULT");

  // Control State Layout
  // 
  // Label
  QGridLayout* status_layout = new QGridLayout;
  status_layout->addWidget( new QLabel("Current State:"), 0, 0, 1, 1);
  status_layout->addWidget( currentStateLabel, 0, 1, 1, 1);
  status_layout->addWidget( new QLabel("SubState:"), 1, 0, 1, 1);
  status_layout->addWidget( currentSubStateLabel, 1, 1, 1, 1);
  
  QGridLayout* control_state_layout = new QGridLayout;
  control_state_layout->addWidget( new QLabel( "Control State:"), 0, 0, 1, 4 );

  // Top Row
  init_button_ = new QPushButton("Init", this);
  control_state_layout->addWidget( init_button_, 1, 0, 1, 1 ); 

  manual_button_ = new QPushButton("Manual Control", this);
  control_state_layout->addWidget( manual_button_, 1, 1, 1, 1 ); 

  start_button_ = new QPushButton("Start Derby", this);
  control_state_layout->addWidget( start_button_, 1, 2, 1, 1 ); 

  end_button_ = new QPushButton("End Derby", this);
  control_state_layout->addWidget( end_button_, 1, 3, 1, 1 ); 


  // 
  configuration_button_ = new QPushButton("Configuration", this);
  control_state_layout->addWidget( configuration_button_, 2, 1, 1, 2 ); 

  // Row 3 Config modes
  configuration_button_goal_color_ = new QPushButton("Goal Color", this);
  control_state_layout->addWidget( configuration_button_goal_color_, 3, 0, 1, 1 ); 
  configuration_button_ceiling_ = new QPushButton("Ceiling", this);
  control_state_layout->addWidget( configuration_button_ceiling_, 3, 1, 1, 1 ); 
  configuration_button_ball_color_ = new QPushButton("Ball Color", this);
  control_state_layout->addWidget( configuration_button_ball_color_, 3, 2, 1, 1 ); 
  

  find_goal_button_ = new QPushButton("Find goal", this);
  control_state_layout->addWidget( find_goal_button_, 4, 1, 1, 2  ); 

  // row 5 Find goal substates
  find_goal_button_search_ = new QPushButton("Search for Goal", this);
  control_state_layout->addWidget( find_goal_button_search_, 5, 0, 1, 1  ); 
  find_goal_button_found_ = new QPushButton("Goal Found", this);
  control_state_layout->addWidget( find_goal_button_found_, 5, 1, 1, 1  ); 

  
  move_to_goal_button_ = new QPushButton("Move To Goal", this);
  control_state_layout->addWidget( move_to_goal_button_, 6, 1, 1, 2 ); 

  // row 7 move to goal substates
  move_to_goal_button_moving_ = new QPushButton("Moving To Goal", this);
  control_state_layout->addWidget( move_to_goal_button_moving_, 7, 0, 1, 1 ); 
  move_to_goal_button_at_ = new QPushButton("At Goal", this);
  control_state_layout->addWidget( move_to_goal_button_at_, 7, 1, 1, 1 ); 
  move_to_goal_button_failed_ = new QPushButton("Moving To Goal Failed", this);
  control_state_layout->addWidget( move_to_goal_button_failed_, 7, 2, 1, 1 ); 

  find_ball_button_ = new QPushButton("Find Ball", this);
  control_state_layout->addWidget( find_ball_button_, 8, 1,1, 2 ); 
  
  // row 9 move to goal substates
  find_ball_button_search_ = new QPushButton("Search For Ball", this);
  control_state_layout->addWidget( find_ball_button_search_, 9, 0,1, 1 ); 
  find_ball_button_found_ = new QPushButton("Ball Found", this);
  control_state_layout->addWidget( find_ball_button_found_, 9, 1,1, 1 ); 

  move_to_ball_button_ = new QPushButton("Move to Ball", this);
  control_state_layout->addWidget( move_to_ball_button_, 10, 1, 1, 2 ); 

  // row 11 move to goal substates
  move_to_ball_button_moving_ = new QPushButton("Moving to Ball", this);
  control_state_layout->addWidget( move_to_ball_button_moving_, 11, 0, 1, 1 ); 
  move_to_ball_button_at_ = new QPushButton("At Ball", this);
  control_state_layout->addWidget( move_to_ball_button_at_, 11, 1, 1, 1 ); 
  move_to_ball_button_failed_ = new QPushButton("Moving to Ball Failed", this);
  control_state_layout->addWidget( move_to_ball_button_failed_, 11, 2, 1, 1 ); 
  move_to_ball_button_center_on_ball_ = new QPushButton("Center On Ball", this);
  control_state_layout->addWidget( move_to_ball_button_center_on_ball_, 11, 3, 1, 1 ); 
 
  pick_up_ball_button_ = new QPushButton("Pick Up Ball", this);
  control_state_layout->addWidget( pick_up_ball_button_, 12, 1,1, 2 ); 

  // row 13 pick up ball substates
  pick_up_ball_button_attempt_ = new QPushButton("Pick Up Ball Attempt", this);
  control_state_layout->addWidget( pick_up_ball_button_attempt_, 13, 0,1, 1 ); 
  pick_up_ball_button_check_ = new QPushButton("Check Ball", this);
  control_state_layout->addWidget( pick_up_ball_button_check_, 13, 1,1, 1 ); 
  pick_up_ball_button_got_ = new QPushButton("Got Ball", this);
  control_state_layout->addWidget( pick_up_ball_button_got_, 13, 2,1, 1 ); 
  pick_up_ball_button_failed_ = new QPushButton("Pick Up Ball Failed", this);
  control_state_layout->addWidget( pick_up_ball_button_failed_, 13, 3,1, 1 ); 
  
  drop_ball_button_ = new QPushButton("Drop Ball", this);
  control_state_layout->addWidget( drop_ball_button_, 14, 1, 1, 2 ); 

  drop_ball_button_dropped_ = new QPushButton("Droped Ball", this);
  control_state_layout->addWidget( drop_ball_button_dropped_, 15, 0, 1, 1 ); 
  drop_ball_button_failed_ = new QPushButton("Droped Ball Failed", this);
  control_state_layout->addWidget( drop_ball_button_failed_, 15, 1, 1, 1 ); 

  next_run_prep_button_ = new QPushButton("Next Run Prep", this);
  control_state_layout->addWidget( next_run_prep_button_, 16, 1, 1, 2);

  next_run_prep_button_turn_around_ = new QPushButton("Turn Around", this);
  control_state_layout->addWidget( next_run_prep_button_turn_around_, 17, 0, 1, 1);
  next_run_prep_button_ready_for_next_run_ = new QPushButton("Ready For Next Run", this);
  control_state_layout->addWidget( next_run_prep_button_ready_for_next_run_, 17, 1, 1, 1);

  // Arm Control Layout
  //
  // Label
  QGridLayout* arm_state_layout = new QGridLayout;
  arm_state_layout->addWidget( new QLabel( "Arm State:"), 0, 0, 1, 2 );
  
  arm_search_button_ = new QPushButton("Search", this);
  arm_state_layout->addWidget(arm_search_button_, 1, 0, 1, 2);

  arm_grab_open_button_ = new QPushButton("Grab Open", this);
  arm_state_layout->addWidget(arm_grab_open_button_, 2, 0, 1, 1);

  arm_grab_close_button_ = new QPushButton("Grab Close", this);
  arm_state_layout->addWidget(arm_grab_close_button_, 2, 1, 1, 1);
  
  arm_check_button_ = new QPushButton("Check", this);
  arm_state_layout->addWidget(arm_check_button_, 3, 0, 1, 1);

  arm_got_ball_search_button_ = new QPushButton("Got Ball Search", this);
  arm_state_layout->addWidget(arm_got_ball_search_button_, 3, 1, 1, 1);

  arm_drop_close_button_ = new QPushButton("Drop Close", this);
  arm_state_layout->addWidget(arm_drop_close_button_, 4, 0, 1, 1);

  arm_drop_open_button_ = new QPushButton("Drop Open", this);
  arm_state_layout->addWidget(arm_drop_open_button_, 4, 1, 1, 1);

  // Then create the control widget.
  drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( status_layout );
  layout->addLayout( control_state_layout );
  layout->addLayout( arm_state_layout );
  //layout->addWidget( drive_widget_ );
  //layout->addLayout( topic_layout );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TeleopPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  // SLOT connections cant be passed args likethis - TODO
  connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Control
  connect( configuration_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( configuration_button_goal_color_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( configuration_button_ceiling_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( configuration_button_ball_color_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
  connect( end_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( start_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( init_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( manual_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( find_goal_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( find_goal_button_search_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( find_goal_button_found_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
  connect( move_to_goal_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( move_to_goal_button_moving_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( move_to_goal_button_at_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( move_to_goal_button_failed_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
  connect( find_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( find_ball_button_search_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( find_ball_button_found_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
  connect( move_to_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( move_to_ball_button_moving_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( move_to_ball_button_at_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( move_to_ball_button_failed_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( move_to_ball_button_center_on_ball_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
  connect( pick_up_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( pick_up_ball_button_attempt_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( pick_up_ball_button_check_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( pick_up_ball_button_got_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( pick_up_ball_button_failed_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
  connect( drop_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( drop_ball_button_dropped_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( drop_ball_button_failed_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
  connect( next_run_prep_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
      connect( next_run_prep_button_turn_around_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));
      connect( next_run_prep_button_ready_for_next_run_, SIGNAL( released() ), control_sub_mapper, SLOT( map() ));

  // Arm
  connect( arm_search_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_grab_open_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_grab_close_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_check_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_drop_close_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_drop_open_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_got_ball_search_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  
  // Control
  control_mapper->setMapping(init_button_, INIT);
  control_mapper->setMapping(manual_button_, MANUAL);
  control_mapper->setMapping(start_button_, START);
  control_mapper->setMapping(end_button_, END);
  control_mapper->setMapping(configuration_button_, CONFIG);
      control_sub_mapper->setMapping(configuration_button_goal_color_, CONFIG_GOAL_COLOR);
      control_sub_mapper->setMapping(configuration_button_ceiling_, CONFIG_CEILING);
      control_sub_mapper->setMapping(configuration_button_ball_color_, CONFIG_BALL_COLOR );
  control_mapper->setMapping(find_goal_button_, FIND_GOAL);
      control_sub_mapper->setMapping(find_goal_button_search_, SEARCH_FOR_GOAL );
      control_sub_mapper->setMapping(find_goal_button_found_, GOAL_FOUND );
  control_mapper->setMapping(move_to_goal_button_, MOVE_TO_GOAL);
      control_sub_mapper->setMapping(move_to_goal_button_moving_, MOVING_TO_GOAL);
      control_sub_mapper->setMapping(move_to_goal_button_at_, AT_GOAL);
      control_sub_mapper->setMapping(move_to_goal_button_failed_, MOVE_TO_GOAL_FAILED);
  control_mapper->setMapping(find_ball_button_, FIND_BALL);
      control_sub_mapper->setMapping(find_ball_button_search_, SEARCH_FOR_BALL);
      control_sub_mapper->setMapping(find_ball_button_found_, BALL_FOUND);
  control_mapper->setMapping(move_to_ball_button_, MOVE_TO_BALL);
      control_sub_mapper->setMapping(move_to_ball_button_moving_, MOVING_TO_BALL);
      control_sub_mapper->setMapping(move_to_ball_button_at_, AT_BALL);
      control_sub_mapper->setMapping(move_to_ball_button_failed_, MOVE_TO_BALL_FAILED);
      control_sub_mapper->setMapping(move_to_ball_button_center_on_ball_, CENTER_ON_BALL);
  control_mapper->setMapping(pick_up_ball_button_, PICK_UP_BALL);
      control_sub_mapper->setMapping(pick_up_ball_button_attempt_, ATTEMPT_PICK_UP);
      control_sub_mapper->setMapping(pick_up_ball_button_check_, CHECK_BALL);
      control_sub_mapper->setMapping(pick_up_ball_button_got_, GOT_BALL);
      control_sub_mapper->setMapping(pick_up_ball_button_failed_, GOT_BALL_FAILED);
  control_mapper->setMapping(drop_ball_button_, DROP_BALL);
      control_sub_mapper->setMapping(drop_ball_button_dropped_, BALL_DROPPED);
      control_sub_mapper->setMapping(drop_ball_button_failed_, DROP_BALL_FAILED);
  control_mapper->setMapping(next_run_prep_button_, NEXT_RUN_PREP);
      control_sub_mapper->setMapping(next_run_prep_button_turn_around_, TURN_AROUND);
      control_sub_mapper->setMapping(next_run_prep_button_ready_for_next_run_, READY_FOR_NEXT_RUN);

  // Arm
  arm_mapper->setMapping(arm_search_button_, 0);
  arm_mapper->setMapping(arm_grab_open_button_, 1);
  arm_mapper->setMapping(arm_grab_close_button_, 2);
  arm_mapper->setMapping(arm_check_button_, 3);
  arm_mapper->setMapping(arm_drop_close_button_, 4);
  arm_mapper->setMapping(arm_drop_open_button_, 5);
  arm_mapper->setMapping(arm_got_ball_search_button_, 6);

  connect( control_mapper, SIGNAL(mapped(int)), this, SLOT(handleControlButton(int)));
  connect( arm_mapper, SIGNAL(mapped(int)), this, SLOT(handleArmButton(int)));
  connect( control_sub_mapper, SIGNAL(mapped(int)), this, SLOT(handleControlSubStateButton(int)));

  // Start the timer.
  output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  drive_widget_->setEnabled( false );

  control_state_publisher_ = nh_.advertise<coconuts_common::ControlState>("/control_state_override", 1);
  arm_control_publisher_ = nh_.advertise<coconuts_common::ArmMovement>("/motor_control", 1);
  control_state_subscriber_ = nh_.subscribe<coconuts_common::ControlState>("/control_state", 1, &coconuts_rviz_plugins::TeleopPanel::controlStateCallback, this);
}

// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
void TeleopPanel::setVel( float lin, float ang )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

void TeleopPanel::controlStateCallback(const coconuts_common::ControlState::ConstPtr& state_msg) {

   switch (state_msg->state) {

       case INIT: 
            currentStateLabel->setText("INIT");
           break;

       case MANUAL: 
            currentStateLabel->setText("MANUAL");
           break;

       case START: 
            currentStateLabel->setText("START");
           break;

       case END: 
            currentStateLabel->setText("END");
           break;

       case CONFIG: 
            currentStateLabel->setText("CONFIG");
           break;

       case FIND_GOAL: 
            currentStateLabel->setText("FIND_GOAL");
           break;

       case MOVE_TO_GOAL: 
            currentStateLabel->setText("MOVE_TO_GOAL");
           break;

       case FIND_BALL: 
            currentStateLabel->setText("FIND_BALL");
           break;

       case MOVE_TO_BALL: 
           currentStateLabel->setText("MOVE_TO_BALL");
           break;

       case PICK_UP_BALL: 
            currentStateLabel->setText("PICK_UP_BALL");
           break;

       case DROP_BALL: 
            currentStateLabel->setText("DROP_BALL");
           break;

       case NEXT_RUN_PREP: 
            currentStateLabel->setText("NEXT_RUN_PREP");
           break;

       default:
           currentStateLabel->setText(QString::number(state_msg->state));
           break;

   }

   switch(state_msg->sub_state) {
       case CONFIG_GOAL_COLOR:
           currentSubStateLabel->setText("CONFIG_GOAL_COLOR");
           break;

       case CONFIG_CEILING:
           currentSubStateLabel->setText("CONFIG_CEILING");
           break;

       case CONFIG_BALL_COLOR:
           currentSubStateLabel->setText("CONFIG_BALL_COLOR");
           break;

       case SEARCH_FOR_GOAL:
           currentSubStateLabel->setText("SEARCH_FOR_GOAL");
           break;

       case GOAL_FOUND:
           currentSubStateLabel->setText("GOAL_FOUND");
           break;

       case MOVING_TO_GOAL:
           currentSubStateLabel->setText("MOVING_TO_GOAL");
           break;

       case AT_GOAL:
           currentSubStateLabel->setText("AT_GOAL");
           break;

       case MOVE_TO_GOAL:
           currentSubStateLabel->setText("MOVE_TO_GOAL");
           break;

       case MOVE_TO_GOAL_FAILED:
           currentSubStateLabel->setText("MOVE_TO_GOAL_FAILED");
           break;

       case SEARCH_FOR_BALL:
           currentSubStateLabel->setText("SEARCH_FOR_BALL");
           break;

       case BALL_FOUND:
           currentSubStateLabel->setText("BALL_FOUND");
           break;

       case GREEN_BALL_FOUND:
           currentSubStateLabel->setText("GREEN_BALL_FOUND");
           break;

       case ORANGE_BALL_FOUND:
           currentSubStateLabel->setText("ORANGE_BALL_FOUND");
           break;

       case MOVING_TO_GREEN:
           currentSubStateLabel->setText("MOVING_TO_GREEN");
           break;

       case AT_GREEN:
           currentSubStateLabel->setText("AT_GREEN");
           break;

       case MOVE_TO_GREEN_FAILED:
           currentSubStateLabel->setText("MOVE_TO_GREEN_FAILED");
           break;

       case CENTER_ON_GREEN:
           currentSubStateLabel->setText("CENTER_ON_GREEN");
           break;

       case MOVING_TO_ORANGE:
           currentSubStateLabel->setText("MOVING_TO_ORANGE");
           break;

       case AT_ORANGE:
           currentSubStateLabel->setText("AT_ORANGE");
           break;

       case MOVE_TO_ORANGE_FAILED:
           currentSubStateLabel->setText("MOVE_TO_ORANGE_FAILED");
           break;

       case CENTER_ON_ORANGE:
           currentSubStateLabel->setText("CENTER_ON_ORANGE");
           break;

       case MOVE_TO_BALL_FAILED:
           currentSubStateLabel->setText("MOVE_TO_BALL_FAILED");
           break;

       case CENTER_ON_BALL:
           currentSubStateLabel->setText("CENTER_ON_BALL");
           break;

       case MOVING_TO_BALL:
           currentSubStateLabel->setText("MOVING_TO_BALL");
           break;

       case AT_BALL:
           currentSubStateLabel->setText("AT_BALL");
           break;

       case ATTEMPT_PICK_UP_GREEN:
           currentSubStateLabel->setText("ATTEMPT_PICK_UP_GREEN");
           break;

       case CHECK_GREEN:
           currentSubStateLabel->setText("CHECK_GREEN");
           break;

       case ATTEMPT_PICK_UP_ORANGE:
           currentSubStateLabel->setText("ATTEMPT_PICK_UP_ORANGE");
           break;

       case CHECK_ORANGE:
           currentSubStateLabel->setText("CHECK_ORANGE");
           break;

       case GOT_BALL:
           currentSubStateLabel->setText("GOT_BALL");
           break;

       case GOT_BALL_FAILED:
           currentSubStateLabel->setText("GOT_BALL_FAILED");
           break;

       case ATTEMPT_PICK_UP:
           currentSubStateLabel->setText("ATTEMPT_PICK_UP");
           break;

       case CHECK_BALL:
           currentSubStateLabel->setText("CHECK_BALL");
           break;

       case DROP_BALL:
           currentSubStateLabel->setText("DROP_BALL");
           break;

       case DROP_BALL_FAILED:
           currentSubStateLabel->setText("DROP_BALL_FAILED");
           break;

       case TURN_AROUND:
           currentSubStateLabel->setText("TURN_AROUND");
           break;

       case READY_FOR_NEXT_RUN:
           currentSubStateLabel->setText("READY_FOR_NEXT_RUN");
           break;

       case DEFAULT_SUB_STATE:
           currentSubStateLabel->setText("DEFAULT_SUB_STATE");
           break;

       default:
           currentSubStateLabel->setText(QString::number(state_msg->sub_state));
           break;
   }

   currentStateLabel->repaint();
   currentSubStateLabel->repaint();
   // This does not work
   //this->parent()->processEvents();
}

void TeleopPanel::handleControlButton(int control_state) {
  ROS_INFO("in handleControlButton");
  coconuts_common::ControlState control_state_msg;
  control_state_msg.state = control_state;
  control_state_msg.sub_state = DEFAULT_SUB_STATE;
  control_state_publisher_.publish(control_state_msg);
}

void TeleopPanel::handleControlSubStateButton(int control_sub_state) {
  ROS_INFO("in handleControlSubStateButton");
  coconuts_common::ControlState control_state;
  control_state.sub_state = control_sub_state;
  switch(control_sub_state) {
      case 21:
      case 22:
      case 23:
          control_state.state = CONFIG;
          break;

      case 41:
      case 42:
          control_state.state = FIND_GOAL;
          break;

      case 51:
      case 52:
      case 53:
          control_state.state = MOVE_TO_GOAL;
          break;

      case 61:
      case 62:
      case 63:
      case 64:
          control_state.state = FIND_BALL;
          break;

      case 71:
      case 72:
      case 73:
      case 74:
      case 75:
      case 76:
      case 77:
      case 78:
      case 79:
      case 170:
      case 171:
      case 172:
          control_state.state = MOVE_TO_BALL;
          break;

      case 81:
      case 82:
      case 83:
      case 84:
      case 85:
      case 86:
      case 87:
      case 88:
          control_state.state = PICK_UP_BALL;
          break;

      case 91:
      case 92:
          control_state.state = DROP_BALL;
          break;

      case 101:
      case 102:
          control_state.state = NEXT_RUN_PREP;
          break;


      default:
          control_state.state = INIT;
          control_state.sub_state = DEFAULT_SUB_STATE;
          break;
  }
  control_state_publisher_.publish(control_state);
}

void TeleopPanel::handleArmButton(int arm_state) {
  coconuts_common::ArmMovement arm_control;
  arm_control.type = "POSE";
  switch(arm_state) {
      case 0:
          arm_control.pose = "SEARCH";
          break;
      case 1:
          arm_control.pose = "GRAB_BALL_OPEN";
          break;
      case 2:
          arm_control.pose = "GRAB_BALL_CLOSE";
          break;
      case 3:
          arm_control.pose = "CHECK_BALL";
          break;
      case 4:
          arm_control.pose = "DROP_BALL_CLOSE";
          break;
      case 5:
          arm_control.pose = "DROP_BALL_OPEN";
          break;
      case 6:
          arm_control.pose = "GOT_BALL_SEARCH";
          break;
      default:
          arm_control.pose = "SEARCH";
          break;
  }
  arm_control_publisher_.publish(arm_control);
}

// Set the topic name we are publishing to.
void TeleopPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
    else
    {
      // The old ``velocity_publisher_`` is destroyed by this assignment,
      // and thus the old topic advertisement is removed.  The call to
      // nh_advertise() says we want to publish data on the new topic
      // name.
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
  drive_widget_->setEnabled( output_topic_ != "" );
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace coconuts_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(coconuts_rviz_plugins::TeleopPanel,rviz::Panel )
// END_TUTORIAL
