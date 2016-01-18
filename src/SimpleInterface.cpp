#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

using namespace coconuts

class SimpleInterface
{
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<ChangeStateModeAction> as_;
  std::string action_name_;

  ros::Publisher trim_pot_pub_;
  ros::Subscriber controller_sub;

public:
  SimpleInterface(std::string name) :
    as_(nh_, name, boost::bind(&SimpleInterface::stateChangedCallback, this, _1), false),
    action_name_(name),
    it_(nh_)

  {
    as_.start();

    ROS_INFO("SimpleInterface ready");

    trim_pot_pub_ = nh_.advertise<geometry_msgs::Twist>("/trim_pots", 1);
  }

  ~SimpleInterface(void)
  {
  }

  void stateAction()
  {
    ROS_INFO("stateAction(): %d", currentState_.state_mode_id);
    ros::Rate r(10);

    // echo state change via beep 
    if (playSongs && oldState_.state_mode_id != currentState_.state_mode_id) {
        ROS_INFO("Playing State Change Song %d", currentState_.state_mode_id);
	oldState_.state_mode_id = currentState_.state_mode_id;

        irobotcreate2::PlaySong play_song;
        play_song.song_number = currentState_.state_mode_id;
        try {
		play_song_pub_.publish(play_song);
        } catch (std::exception& e) {
           ROS_INFO("Failed to Play Song");
        }
	
    }

    switch(currentState_.state_mode_id) {
      case FOLLOW_OWNER:
        if (!ownerFlag) {
          move(0, 0.25); //turn and look for owner
        }
        else {
          move(0.25, 0); //move forward to owner
        }
        break;
      case FIND_BALL:
        if (!ballFlag) {
          move(0, 0.10); //turn and look for ball
        }
        else {
          move(0.15, 0); //move forward to ball
        }
        break;
      case FETCH_BALL:
        if (!ownerFlag) {
          move(0, 0.50);
        }
        else {
          move(0.25, 0);
        }
        break;

      //cases below used for testing
      case 10:
        ROS_INFO("CASE 10");
        ownerFlag = true;
        irBumperFlag = true;
        currentState_.state_mode_id = FOLLOW_OWNER;
        stateAction();
        break;
      case 11:
        ROS_INFO("CASE 11");
        ballFlag = true;
        irBumperFlag = true;
        currentState_.state_mode_id = FIND_BALL;
        stateAction();
        break;
      case 12:
        ROS_INFO("CASE 12");
        irBumperFlag = false;
        currentState_.state_mode_id = FETCH_BALL;
        stateAction();
        break;
      default:
        ROS_INFO("RESET");
        ownerFlag = false;
        ballFlag = false;
        irBumperFlag = true;
	currentState_.state_mode_id = -1; 
        break;
    }
  }

  void move(float distance, float angle)
  {
    geometry_msgs::Twist msg;
    msg.linear.x = distance;
    msg.angular.z = angle;

    vel_pub_.publish(msg);

    ros::Rate rate(1);
    rate.sleep();
  }

  void upperFrameCallback(const sensor_msgs::ImageConstPtr& msg) {
    bool ownerRecognized = false;
    sensor_msgs::Image::Ptr out_msg;
    cv::Mat element;
    cv::Mat img_smooth, frame_result, hsv, img_result, mask, masked_result, img_edge_gray_y;
    cv::Mat img_contour, drawing;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Rect> boundRect (4100);
    std::vector<std::vector<cv::Point> > contours_poly (4100);

    try {
      cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
      //CvMatArray *frameArray = new CvMatArray(frame);
      //CvSize size = cvGetSize(frameArray);
      //IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
      cv::cvtColor(frame, hsv, CV_BGR2HSV);

      //CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
      //cv::inRange(f, cv::Scalar(159, 135, 135), cv::Scalar(179, 255, 255), mask);
      //cv::inRange(hsv, cv::Scalar(0.11*256, 0.60*256, 0.20*256, 0),
      //            cv::Scalar(0.14*256, 1.00*256, 1.00*256, 0), mask);
      //The following looks for skin
      //cv::inRange(hsv, cv::Scalar(100, 0.1*256, 0.1*256, 0),
      //            cv::Scalar(130, 0.9*256, 0.8*256, 0), mask);
      cv::inRange(hsv, cv::Scalar(150, 0.0*256, 0.0*256, 0),
                  cv::Scalar(190, 1.0*256, 1.0*256, 0), mask);
      //cv::cvtColor(mask, img_result, CV_HSV2BGR);

      cv::findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0) );

      for( int i = 0; i< contours.size(); i++ )
      {
        //ROS_INFO("%d", i);
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      }

      drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );

      for( int i = 0; i< contours.size(); i++ ) {
        cv::Scalar color = cv::Scalar( 107,222, 46 );
        //cv::drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        if (boundRect[i].area() > 12000) {
          cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
          ownerRecognized = true;
        }
      }

      frame.copyTo(masked_result, mask);
      cv::add(drawing, masked_result, masked_result);

      out_msg = cv_bridge::CvImage(msg->header, msg->encoding, masked_result).toImageMsg();
      upper_image_pub_.publish(out_msg);
    } catch (cv::Exception &e) {
      ROS_INFO("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    if (ownerRecognized) {
      if (!ownerFlag) {
        ROS_INFO("OWNERFLAG TRUE");
        ownerFlag = true;

        if (currentState_.state_mode_id != FIND_BALL) {
          move(0, 0);
	  move(0, 0.35);
          stateAction();
        }
      }
    }
  }

  void lowerFrameCallback(const sensor_msgs::ImageConstPtr& msg) {
    bool ballRecognized = false;
    sensor_msgs::Image::Ptr out_msg;
    cv::Mat element;
    cv::Mat img_smooth, frame_result, hsv, img_result, mask, masked_result, img_edge_gray_y;
    cv::Mat img_contour, drawing;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Rect> boundRect (4100);
    std::vector<std::vector<cv::Point> > contours_poly (4100);

    try {
      cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
      //CvMatArray *frameArray = new CvMatArray(frame);
      //CvSize size = cvGetSize(frameArray);
      //IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
      cv::cvtColor(frame, hsv, CV_BGR2HSV);

      //CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
      //cv::inRange(f, cv::Scalar(159, 135, 135), cv::Scalar(179, 255, 255), mask);
      //cv::inRange(hsv, cv::Scalar(0.11*256, 0.60*256, 0.20*256, 0),
      //            cv::Scalar(0.14*256, 1.00*256, 1.00*256, 0), mask);
      cv::inRange(hsv, cv::Scalar(75, 0.4*256, 0.4*256, 0),
                  cv::Scalar(90, 0.9*256, 0.8*256, 0), mask);
      //cv::cvtColor(mask, img_result, CV_HSV2BGR);

      cv::findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0) );

      for( int i = 0; i< contours.size(); i++ )
      {
        //ROS_INFO("%d", i);
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      }

      drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );

      for( int i = 0; i< contours.size(); i++ ) {
        cv::Scalar color = cv::Scalar( 107,222, 46 );
        //cv::drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        if (boundRect[i].area() > 900) {
          cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
          ballRecognized = true;
        }
      }

      frame.copyTo(masked_result, mask);
      cv::add(drawing, masked_result, masked_result);

      out_msg = cv_bridge::CvImage(msg->header, msg->encoding, masked_result).toImageMsg();
      lower_image_pub_.publish(out_msg);
    } catch (cv::Exception &e) {
      ROS_INFO("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    if (ballRecognized) {
      if (!ballFlag) {
        ROS_INFO("BALLFLAG TRUE");
        ballFlag = true;

        if (currentState_.state_mode_id == FIND_BALL) {
          move(0, 0);
	  move(0, 0.20);
          stateAction();
        }
      }
    }
  }

  void irBumperCallback(const irobotcreate2::RoombaIR::ConstPtr& msg) {
    if (currentState_.state_mode_id != FETCH_BALL && irBumperFlag && msg->signal > 10 && msg->header.frame_id != "base_irbumper_right") {
        ROS_INFO("STATE: %d, IR_BUMPER SIGNAL: [%d]", currentState_.state_mode_id, msg->signal);
        move(0, 0);

      if (currentState_.state_mode_id == FIND_BALL) {
        ownerFlag = false;
        irBumperFlag = false;
        currentState_.state_mode_id = FETCH_BALL;
        stateAction();
      }
    }
  }

  void bumperCallback(const irobotcreate2::Bumper::ConstPtr& msg) {
    if (msg->left.state || msg->right.state) {
      if (currentState_.state_mode_id == FOLLOW_OWNER)
      {
        ballFlag = false;
        currentState_.state_mode_id = FIND_BALL;
        stateAction();
      }
      else if (currentState_.state_mode_id == FETCH_BALL)
      {
	move(0, 0.20);
        ownerFlag = false;
        currentState_.state_mode_id = FOLLOW_OWNER;
        stateAction();
      }
    }
  }

  void stateChangedCallback(const ChangeStateModeGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    } else {
      // Change configuration which will cause stateChangedCallback to behave differently
      //ROS_INFO("Configuration updated");
    }

    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep();

    if (success)
    {
      result_.state_mode_changed = 1;
      //ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);

      currentState_.state_mode_id = goal->state_mode_id;
    }

    stateAction();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SimpleInterface");
  SimpleInterface SimpleInterface(ros::this_node::getName());
  ros::spin();
  return 0;
}

