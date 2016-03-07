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
{

  // Mapping object for passing button values
  QSignalMapper * control_mapper = new QSignalMapper(this);
  QSignalMapper * arm_mapper = new QSignalMapper(this);
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  // Control State Layout
  // 
  // Label
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
  //
  

  find_goal_button_ = new QPushButton("Find goal", this);
  control_state_layout->addWidget( find_goal_button_, 4, 1, 1, 2  ); 

  // row 5 Find goal substates

  
  move_to_goal_button_ = new QPushButton("Move To Goal", this);
  control_state_layout->addWidget( move_to_goal_button_, 6, 1, 1, 2 ); 

  // row 7 move to goal substates

  find_ball_button_ = new QPushButton("Find Ball", this);
  control_state_layout->addWidget( find_ball_button_, 8, 1,1, 2 ); 
  
  // row 9 move to goal substates

  move_to_ball_button_ = new QPushButton("Move to Ball", this);
  control_state_layout->addWidget( move_to_ball_button_, 10, 1, 1, 2 ); 

  // row 11 move to goal substates
 
  pick_up_ball_button_ = new QPushButton("Pick Up Ball", this);
  control_state_layout->addWidget( pick_up_ball_button_, 12, 1,1, 2 ); 

  // row 11 move to goal substates
  
  drop_ball_button_ = new QPushButton("Drop Ball", this);
  control_state_layout->addWidget( drop_ball_button_, 14, 1, 1, 2 ); 

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
  arm_state_layout->addWidget(arm_check_button_, 3, 0, 1, 2);

  arm_drop_close_button_ = new QPushButton("Drop Close", this);
  arm_state_layout->addWidget(arm_drop_close_button_, 4, 0, 1, 1);

  arm_drop_open_button_ = new QPushButton("Drop Open", this);
  arm_state_layout->addWidget(arm_drop_open_button_, 4, 1, 1, 1);

  // Then create the control widget.
  drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
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
  connect( end_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( start_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( init_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( manual_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( find_goal_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( move_to_goal_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( find_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( move_to_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( pick_up_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));
  connect( drop_ball_button_, SIGNAL( released() ), control_mapper, SLOT( map() ));

  // Arm
  connect( arm_search_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_grab_open_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_grab_close_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_check_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_drop_close_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  connect( arm_drop_open_button_, SIGNAL( released() ), arm_mapper, SLOT( map() ));
  
  // Control
  control_mapper->setMapping(init_button_, INIT);
  control_mapper->setMapping(manual_button_, MANUAL);
  control_mapper->setMapping(start_button_, START);
  control_mapper->setMapping(end_button_, END);
  control_mapper->setMapping(configuration_button_, CONFIG);
  control_mapper->setMapping(find_goal_button_, FIND_GOAL);
  control_mapper->setMapping(move_to_goal_button_, MOVE_TO_GOAL);
  control_mapper->setMapping(find_ball_button_, FIND_BALL);
  control_mapper->setMapping(move_to_ball_button_, MOVE_TO_BALL);
  control_mapper->setMapping(pick_up_ball_button_, PICK_UP_BALL);
  control_mapper->setMapping(drop_ball_button_, DROP_BALL);

  // Arm
  arm_mapper->setMapping(arm_search_button_, 0);
  arm_mapper->setMapping(arm_grab_open_button_, 1);
  arm_mapper->setMapping(arm_grab_close_button_, 2);
  arm_mapper->setMapping(arm_check_button_, 3);
  arm_mapper->setMapping(arm_drop_close_button_, 4);
  arm_mapper->setMapping(arm_drop_open_button_, 5);

  connect( control_mapper, SIGNAL(mapped(int)), this, SLOT(handleControlButton(int)));
  connect( arm_mapper, SIGNAL(mapped(int)), this, SLOT(handleArmButton(int)));

  // Start the timer.
  output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  drive_widget_->setEnabled( false );

  control_state_publisher_ = nh_.advertise<coconuts_common::ControlState>("/control_state_override", 1);
  arm_control_publisher_ = nh_.advertise<coconuts_common::ArmMovement>("/motor_control", 1);
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


void TeleopPanel::handleControlButton(int control_state) {
    sendControlUpdate(control_state);
}

void TeleopPanel::handleArmButton(int arm_state) {
    sendArmUpdate(arm_state);
}

void TeleopPanel::sendControlUpdate(int behavior_state)
{
  // Do the stuff to do
  ROS_INFO("in sendControlUpdate");
  coconuts_common::ControlState control_state;
  control_state.state = behavior_state;
  control_state.sub_state = DEFAULT_SUB_STATE;
  control_state_publisher_.publish(control_state);
}

void TeleopPanel::sendArmUpdate(int arm_state)
{
  // Do the stuff to do
  ROS_INFO("in sendArmUpdate");
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
