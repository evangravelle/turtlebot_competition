#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist
#from team6_assignment_5.msg import Song
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around, either the ARROW keys or:               
   u    i    o
   j    k    l
   m    ,    .

q/z     : increase/decrease max speeds by 10%
w/x     : increase/decrease only linear speed by 10%
e/c     : increase/decrease only angular speed by 10%
stop    : anything else 
s,v,b,n : play a song
Mode    : 1:Start 2:Stop 3:Reset 4:PowerDown 5:Safe 6:Full 0:Dock
Motor	: y followed by [Motor Number][space][Rotation]

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1),
                # arow keys
		'\x1b[A':(1,0), #up
		'\x1b[B':(-1,0), #down
		'\x1b[C':(0,-1), #right
		'\x1b[D':(0,1), #left
	       }

motorBindings = {
		'y':(0,0), # input motor
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

songBindings={
                's':0,
                'v':1,
                'b':2,
                'n':3,
             }

modeBindings={
                '1':'start',
                '2':'stop',
                '3':'reset',
                '4':'powerdown',
                '5':'safe',
                '6':'full',
                '0':'dock',
             }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
        # handle arrow keys (format -> ESC[n where n is ABCD)
        if key == '\x1b':
          key += sys.stdin.read(1);
          key += sys.stdin.read(1);
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def getEntry():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
        while(key != '\r'):
	  line += key
	  key = sys.stdin.read(1)

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return line

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist,queue_size=10)
	pub_motor = rospy.Publisher('motor_control', String,queue_size=10)
	#pub_play_song = rospy.Publisher('play_song', PlaySong,queue_size=10)
	pub_mode = rospy.Publisher('mode', String,queue_size=10)
	#pub_song = rospy.Publisher('song', Song,queue_size=10)

	rospy.init_node('roomba_keyboard')

	x = 0
	th = 0
	status = 0
	motor = 0
	degree = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][1]
				twist = Twist()
				twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				pub.publish(twist)
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				print vels(speed,turn)
				status = (status + 1) % 18
				twist = Twist()
				twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				pub.publish(twist)
                        elif key in songBindings.keys():
                                #play_song = PlaySong()
                                #play_song.song_number = songBindings[key]
                                print "Playing a song."
				status = (status + 1) % 18
                                #pub_play_song.publish(play_song)
                        elif key in modeBindings.keys():
                                mode = modeBindings[key]
                                print "Changing Mode to: ", mode
                                pub_mode.publish(mode)
				status = (status + 1) % 18
			elif key in motorBindings.keys():
				print "Input [Motor][space][Angle]:"	
				motor = 0
				degree = 30
				status = (status + 1) % 18
				motorMsg = "{} {}".format(motor,degree)
				pub_motor.publish(motorMsg)
			else:
				x = 0
				th = 0
				if (key == '\x03'):
					break

			if (status == 18):
				print msg
	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


