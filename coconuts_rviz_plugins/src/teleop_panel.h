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
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QLabel>
#include <coconuts_common/ControlState.h>

class QLineEdit;
class QPushButton;

namespace coconuts_rviz_plugins
{

class DriveWidget;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class TeleopPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  TeleopPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // The control area, DriveWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.
  void setVel( float linear_velocity_, float angular_velocity_ );

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic( const QString& topic );

  // Here we declare some internal slots.
protected Q_SLOTS:
  // sendvel() publishes the current velocity values to a ROS
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.
  void sendVel();

  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateTopic();
  void controlStateCallback(const coconuts_common::ControlState::ConstPtr& state_msg);
  void handleControlButton(int control_state);
  void handleControlSubStateButton(int control_state);
  void handleArmButton(int arm_state);

  // Then we finish up with protected member variables.
protected:
  // The control-area widget which turns mouse events into command
  // velocities.
  DriveWidget* drive_widget_;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* output_topic_editor_;

  // Control State Buttons
  QPushButton* configuration_button_;
      QPushButton* configuration_button_goal_color_;
      QPushButton* configuration_button_ceiling_;
      QPushButton* configuration_button_ball_color_;
    
  QPushButton* start_button_;
  QPushButton* end_button_;
  QPushButton* init_button_;
  QPushButton* manual_button_;

  QPushButton* find_goal_button_;
      QPushButton* find_goal_button_search_;
      QPushButton* find_goal_button_found_;
    
  QPushButton* move_to_goal_button_;
      QPushButton* move_to_goal_button_moving_;
      QPushButton* move_to_goal_button_at_;
      QPushButton* move_to_goal_button_failed_;

  QPushButton* find_ball_button_;
      QPushButton* find_ball_button_search_;
      QPushButton* find_ball_button_found_;

  QPushButton* move_to_ball_button_;
      QPushButton* move_to_ball_button_moving_;
      QPushButton* move_to_ball_button_at_;
      QPushButton* move_to_ball_button_failed_;
      QPushButton* move_to_ball_button_center_on_ball_;

  QPushButton* pick_up_ball_button_;
      QPushButton* pick_up_ball_button_attempt_;
      QPushButton* pick_up_ball_button_check_;
      QPushButton* pick_up_ball_button_got_;
      QPushButton* pick_up_ball_button_failed_;

  QPushButton* drop_ball_button_;
      QPushButton* drop_ball_button_dropped_;
      QPushButton* drop_ball_button_failed_;

  QPushButton* next_run_prep_button_;
      QPushButton* next_run_prep_button_turn_around_;
      QPushButton* next_run_prep_button_ready_for_next_run_;

  // Arm Position Buttons
  //
  QPushButton* arm_search_button_;
  QPushButton* arm_grab_open_button_;
  QPushButton* arm_grab_close_button_;
  QPushButton* arm_check_button_;
  QPushButton* arm_drop_close_button_;
  QPushButton* arm_drop_open_button_;
  QPushButton* arm_got_ball_search_button_;

  // State values
  QLabel *currentStateLabel;
  QLabel *currentSubStateLabel;

  // The current name of the output topic.
  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Publisher velocity_publisher_;
  ros::Publisher control_state_publisher_;
  ros::Publisher arm_control_publisher_;

  // Ros Subscriptions 
  ros::Subscriber control_state_subscriber_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // The latest velocity values from the drive widget.
  float linear_velocity_;
  float angular_velocity_;
  
  // The Latest states from the control_state callback
  // Maybe NOT needed...
  int behavior_state_;
  int behavior_sub_state_;
  // END_TUTORIAL
};

} // end namespace coconuts_rviz_plugins

#endif // TELEOP_PANEL_H
