#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from ballfinder.msg import assn3
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


#This should be the latest version

class Robot:
	def __init__(self):
		self.pub = rospy.Publisher('/mobile_base/commands/velocity',Twist, queue_size =10)
		rospy.Subscriber('/ball_detector/ball_location',assn3,self.search)
		rospy.Subscriber('/odom', Odometry, self.handle_pose)
		rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
		self.bearing=-1
		self.x=0
		self.y=0
		self.theta=0
		self.goalFinalY=0
		self.goalFinalX=0
		self.goal_x=-1
		self.goal_y=-1
		self.goalTheta=0
		self.listener = tf.TransformListener()

		self.spinPID= PID(320,.005,.02,.00)
		self.distPID= PID(1,.15,.02,.02)

		self.interPID = PID(0, .15,.03,.01)
		self.bearPID = PID(0,1,.0,.00)

		self.goalPID = PID(0,.25,.03,.002)
		self.goalBearPID =  PID(0,1,.0,.002)
		self.state = "search"

	def search(self, msg):
		self.bearing=msg.bearing
		self.distance=msg.distance
		print(msg.distance)


	def bumped(self, msg):
		if(msg.state==1):
			self.state="back up"
			self.time = rospy.get_time()

	def get_vector(self, from_x, from_y, to_x, to_y):
		bearing = math.atan2(to_y - from_y, to_x - from_x)
		distance = math.sqrt((from_x - to_x)**2 + (from_y - to_y)**2)
		return (bearing, distance)


	def handle_pose(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(_, _, self.theta) = euler_from_quaternion(q)

	def standardize_angle(self, theta):
		if(theta <-math.pi):
			theta= theta+2*math.pi
		if(theta > math.pi):
			theta = theta-2*math.pi
		return theta



	def run(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			if self.state == "search":
				#print("seaching")
				twist.angular.z=.6
				twist.linear.x=0
				if(self.bearing !=-1 and self.distance!=-1):
					self.state="approach"
			elif self.state == "back up":
				twist.linear.x=-.3
				time =rospy.get_time()
				if(time - self.time >=2 ):
					self.state = "search"

			elif self.state == "approach":
				if(self.bearing ==-1 or self.distance==-1):
					self.state="search"
				print("approaching")
				twist.angular.z=self.spinPID.get_output(self.bearing)
				if (twist.angular.z>.3):
					twist.angular.z=.3
				if(twist.angular.z<-.3):
					twist.angular.z=-.3
				twist.linear.x= -self.distPID.get_output(self.distance)
				if(twist.linear.x>.5):
					twist.linear.x=.5
				if(twist.linear.x<-.5):
					twist.linear.x=-.5
				if(abs(self.spinPID.get_error())<=50 and abs(self.distPID.get_error())<=.3):
					print(self.spinPID.get_error())
					print("Calculating goals")
					twist.angular.z=0
					twist.linear.x=0
				#Here we do a lot of calculations to find goal valuess
					self.ballX= self.x + self.distance*math.cos(self.theta)
					self.ballY= self.y + self.distance*math.sin(self.theta)
					self.goalTheta = math.atan2((self.ballY - self.goal_y),(self.ballX - self.goal_x))
					self.goalTheta=self.standardize_angle(self.goalTheta)
					self.kX = self.ballX+1.5*math.cos(self.goalTheta)
					self.kY = self.ballY+1.5*math.sin(self.goalTheta)

					(bearing, distance) = self.get_vector(self.goal_x, self.goal_y, self.ballX,self.ballY)
					interTheta = (self.goalTheta+(math.pi/2))
					interTheta=self.standardize_angle(interTheta)
					#self.interY= self.ballY + math.sin(interTheta)
					#self.interX= self.ballX + math.cos(interTheta)
					#self.interY2= self.goal_y + math.sin(self.goalTheta-(math.pi/2))
					#self.interX2= self.goal_x + math.cos(self.goalTheta-(math.pi/2))
					(ang,_)= self.get_vector(self.ballX,self.ballY,self.goal_x,self.goal_y)
					ang= self.standardize_angle(ang)
					self.interX= self.kX+1.5*math.cos(ang +math.pi/4)
					self.interY =self.kY+1.5*math.sin(ang +math.pi/4)
					self.interX2= self.kX+math.cos(ang -math.pi/4)
					self.interY2=self.kY+math.sin(ang -math.pi/4)


					(_,dist2)=self.get_vector(self.x,self.y,self.interX2,self.interY2)
					(_,dist)=self.get_vector(self.x,self.y,self.interX,self.interY)
					if(dist2<dist):
						self.interX=self.interX2
						self.interY=self.interY2
					(_,goaldist) =self.get_vector(self.x,self.y,self.kX,self.kY)
					(_,balldist) =self.get_vector(self.x,self.y,self.ballX,self.ballY)
					#if(goaldist<balldist):
					#	self.state="approachKick"
					#else:
					self.state = "approachInter"
					#self.interX=self.goal_x + ((3.0*math.sqrt(2.0))/2.0)*math.cos(bearing+(math.pi/4.0))
					#self.interY=self.goal_y + ((3.0*math.sqrt(2.0))/2.0)*math.sin(bearing+(math.pi/4.0))
					#self.interX2=self.goal_x + ((3.0*math.sqrt(2.0))/2.0)*math.cos(bearing+(-math.pi/4.0))
					#self.interY2=self.goal_y + ((3.0*math.sqrt(2.0))/2.0)*math.sin(bearing+(-math.pi/4.0))
					#self.goalFinalX=self.x+(3.0*math.cos(self.theta))
					#self.goalFinalY=self.y+(3.0*math.sin(self.theta))


			elif self.state =="approachInter":
				print("Approaching Intermediate Position")
				(bearing, distance) = self.get_vector(self.x, self.y, self.interX,self.interY)
				bearingPRIME = self.theta-bearing
				bearingPRIME=self.standardize_angle(bearingPRIME)

				twist.angular.z= self.bearPID.get_output(bearingPRIME)
				twist.linear.x = -self.interPID.get_output(distance)
				if(twist.linear.x >.2):
					twist.linear.x=.2
				if(twist.linear.x<-.2):
					twist.linear.x=-.2


				print(self.interPID.get_error())
				if(abs(self.interPID.get_error()) <=.2):
					print("ayyy we got it")
					twist.linear.x=0
					twist.angular.z=0
					self.state = "approachKick"

			elif self.state =="approachKick":
				print("Approaching kick position")
				(bearing, distance) = self.get_vector(self.x, self.y, self.kX, self.kY)
				bearingPRIME = self.theta-bearing
				bearingPRIME=self.standardize_angle(bearingPRIME)
				twist.linear.x = -self.goalPID.get_output(distance)
				twist.angular.z = self.goalBearPID.get_output(bearingPRIME)
				print(self.goalFinalX)
				print(self.goalFinalX)
				if(twist.linear.x >.4):
					twist.linear.x=.4
				if(twist.linear.x <-.4):
					twist.linear.x=-.4
				print(self.goalPID.get_error())
				if(abs(self.goalPID.get_error()) <= .2): #and abs(self.goalBearPID.get_error()) <= 2):
					print("A winner is you!")
					self.state = "lineUp"

			elif self.state == "lineUp":
				print("Lining up")
				twist.linear.x=0
				if(self.bearing==-1):
					twist.angular.z=-.8
				if(self.bearing!=-1):
					twist.angular.z=self.spinPID.get_output(self.bearing)
					if (twist.angular.z>.3):
						twist.angular.z=.3
					if(twist.angular.z<-.3):
						twist.angular.z=-.3
					if(abs(self.spinPID.get_error())<=30):
						self.state="kick"
						self.time = rospy.get_time()

			elif self.state == "kick":
				print("kick")
				twist.linear.x=1
				twist.angular.z=0
				time=rospy.get_time()
				if(time-self.time>=3.5):
					self.state="search"

			self.pub.publish(twist)
			try:
				(trans, _) = self.listener.lookupTransform('odom', 'ar_marker_0', rospy.Time(0))
				(self.goal_x, self.goal_y, _) = trans
			except tf.LookupException:
				pass
			except tf.ConnectivityException:
				pass
			except tf.ExtrapolationException:
				pass
			#print("Goal x value")
			#print(self.goal_x)
			#print("Goal y value")
			#print(self.goal_y)
			rate.sleep()

class PID:
	def __init__(self, goal, kp,ki,kd):
		self.goal=goal
		self.kp=kp
		self.ki=ki
		self.kd=kd
		self.previous_error=0
		self.integral=0

	def get_output(self, measurement):
		error= self.goal-measurement
		self.integral=self.integral+error
		derivative = error-self.previous_error
		output= self.kp*error+self.ki*self.integral+self.kd*derivative
		self.previous_error=error
		return output

	def get_error(self):
		return self.previous_error

rospy.init_node('Puppy')
robot = Robot()
robot.run()
