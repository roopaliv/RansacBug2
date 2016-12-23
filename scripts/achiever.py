#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import tf
from std_msgs.msg import ColorRGBA
import smach
import smach_ros
import random

obstacle = False;
workableValues = {}
xBegining = -8.0; #constant  ##You cannot define constants in Python
yBeginning = -2.0; #constant
xPos =  -8.0;
yPos = -2.0;
xGoal = 4.5; #constant
yGoal = 9.0; #constant
xFollow = 0.0;
yFollow = 0.0;
xRansac1 = 0.0;
xRansac2 = 0.0;
yRansac1 = 0.0;
yRansac2 = 0.0;
state = "GOAL_SEEK"
orientation = 0.0;
distFromRansacLine = 8.0; # a value that doesnt affects the movement initially


class RansacIteration:
	xr1 = 0.0
	yr1 = 0.0
	xr2 = 0.0
	yr2 = 0.0
	inliers = 0
	inliersX = []
	inliersY = []

def callbackGroundPos(msg):
    global xPos
    global yPos
    xPos = 	msg.pose.pose.position.x;
    yPos = 	msg.pose.pose.position.y;
    global orientation
    orientation = msg.pose.pose.orientation.z;
def callbackLaser(msg):
	global workableValues
	workableValues = {}
	workableValues.clear()
	j = 0
	for detectedRange in msg.ranges:
		if(detectedRange !=3.0):
			angleWorkable = -1.57079637051 + (j * 0.00872664619237)
			workableValues[angleWorkable] = detectedRange
		j = j+1
	ransac()

def perceiver():
    rospy.init_node('achiever', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=2)
    rospy.Subscriber("base_scan", LaserScan, callbackLaser, queue_size=10)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callbackGroundPos, queue_size=10)
    while not rospy.is_shutdown():
        global state
        if (state == 'GOAL_SEEK'):
            ang_vel =1.0 * (math.atan2((yGoal - yPos),(xGoal - xPos)));
            lin_vel = 0.0
        elif (state == "WALL_FOLLOW"):
            global xRansac1;
            global xRansac2;
            global yRansac1;
            global yRansac2;
            #ang_vel = abs(1.0 * (math.atan2((yRansac2 - yRansac1),(xRansac2 - xRansac1))));
            #slope = (yRansac2 - yRansac1)/(xRansac2 - xRansac1)
            #const = 2
            #if(abs((const*slope) - ang_vel) >0.5): ##donot recalculate for smal differences in ransac iterations
            #    ang_vel = abs(const*slope) ## always turn right
            #    lin_vel = 0.0

            theta1 = (1.0 * (math.atan2((yRansac1 - yPos),(xRansac1- xPos))));
            theta2 = (1.0 * (math.atan2((yRansac2 - yPos),(xRansac2- xPos))));
            slope = ((yRansac2 - yRansac1)/(xRansac2- xRansac1))
            angleToFollow=theta2
            followed1 = False
            if(theta1>theta2):
                angleToFollow = theta1
                followed1 = True

            if(abs(theta1- ang_vel) >0.2): ##donot recalculate for smal differences in ransac iterations
                ang_vel = abs(1*angleToFollow) ## always turn right
                lin_vel = 0.0

        vel = geometry_msgs.msg.Twist()
        #lin_vel = 0.25 * (math.hypot(xFollow - xPos , yFollow - yPos));
        global orientation
        if (abs(orientation - ang_vel) < 0.09):
            ang_vel = 0.0
            lin_vel = 2.0
        if(abs(distFromRansacLine)<4.5):
            lin_vel=0.0
            if(followed1):
                angleToFollow = (1.0 * (math.atan2((yRansac2 - yRansac1),(xRansac2- xRansac1))))
            else:
                angleToFollow = (1.0 * (math.atan2((yRansac1 - yRansac2),(xRansac1- xRansac2))))
            #angleToFollow = (1.0 * ((yRansac1 - yRansac1)/(xRansac1- xRansac2)))

            if(abs(abs(0.3*angleToFollow)-ang_vel)>0.1):
                ang_vel = abs(0.3*angleToFollow)
                if(distFromRansacLine < 3.55):
                    ang_vel = -1 * abs(0.3*angleToFollow)
                    if(abs(abs(0.3*angleToFollow)-ang_vel)>0.1):
                        ang_vel = -1 * abs(0.3*angleToFollow)
            if(abs(orientation - ang_vel) < 0.1):
                lin_vel = 0.9
                ang_vel=0.0
        vel.linear.x = lin_vel
        vel.angular.z = ang_vel
        pub.publish(vel)
        rate.sleep()

def ransac():
    global workableValues
    global xRansac1;
    global xRansac2;
    global yRansac1;
    global yRansac2;
    #workableValues["1"] = "Sachine Tendulkar"
    ransacIterations = 50
    inlierThreshold = 0.2
    minDictionaryLength = 10
    while(len(workableValues) >= minDictionaryLength):
        ransacOutcomes = []
        maxInliers = 0
        for iteration in range(0,int(ransacIterations)):
            ransacOutcome = RansacIteration()
            ransacOutcome.inliersX = []
            ransacOutcome.inliersY = []
            randomPick1 = random.randrange(0, len(workableValues))
            randomPick2 = random.randrange(0, len(workableValues))
            while randomPick1 == randomPick2:
            	randomPick2 = random.randrange(0, len(workableValues))
            radius1 = workableValues.values()[randomPick1]
            radius2 = workableValues.values()[randomPick2]
            angleRad1 = workableValues.keys()[randomPick1]
            angleRad2 = workableValues.keys()[randomPick2]
            x1 = radius1 * (math.cos(angleRad1))
            y1 = radius1 * (math.sin(angleRad1))
            x2 = radius2 * (math.cos(angleRad2))
            y2 = radius2 * (math.sin(angleRad2))
            lenx = x2-x1
            leny = y2-y1
            lenSegment = float(lenx*lenx + leny*leny)
            NumberInliers = 0
            if lenSegment <> 0:
                for angleRad,Scannedrange in workableValues.iteritems():
                	x = Scannedrange * (math.cos(angleRad))
                	y = Scannedrange * (math.sin(angleRad))
                	distance =  ((x - x1) * lenx + (y - y1) * leny) / lenSegment
                	if(distance <= inlierThreshold):
                		ransacOutcome.inliersX.append(x)
                		ransacOutcome.inliersY.append(y)
                		NumberInliers = NumberInliers + 1
                		if(NumberInliers > maxInliers):
                			maxInliers = NumberInliers
				ransacOutcome.xr1 = x1
				ransacOutcome.xr2 = x2
				ransacOutcome.yr1 = y1
				ransacOutcome.yr2 = y2
				ransacOutcome.inliers = NumberInliers
				ransacOutcomes.append(ransacOutcome)

		for line in ransacOutcomes:
			if(line.inliers == maxInliers):
				dist =  math.hypot(line.xr2 - line.xr1, line.yr2 - line.yr1)
				farthestX1 = line.xr1
				farthestX2 = line.xr2
				farthestY1 = line.yr1
				farthestY2 = line.yr2
				ii=0
				for inlierXi in line.inliersX:
					inlierYi = line.inliersY[ii]
					ij = 0
					for inlierXj in line.inliersX:
						inlierYj = line.inliersY[ij]
						tempDist = 	math.hypot(inlierXj - inlierXi, inlierYj - inlierYi)
						if (tempDist > dist):
							dist = tempDist
							farthestX1 = inlierXi
							farthestX2 = inlierXj
							farthestY1 = inlierYi
							farthestY2 = inlierYj
						ij = ij + 1
					ii = ii + 1
                xRansac1 = farthestX1
                xRansac2 = farthestX2
                yRansac1 = farthestY1
                yRansac2 = farthestY2

                lenxr = xRansac2-xRansac1
                lenyr = yRansac2-yRansac1
                lenSegmentr = 	math.hypot(lenxr, lenyr)#float(lenxr*lenxr + lenyr*lenyr)
                global distFromRansacLine;
                distFromRansacLine =  ((xPos - x1) * lenxr + (yPos - y1) * lenyr) / lenSegmentr
                global state
                if(abs(distFromRansacLine)<5.0):
                    state = "WALL_FOLLOW"
                break
            workableValues.pop(angleRad1, None)
            workableValues.pop(angleRad2, None)

if __name__ == '__main__':
	    try:
		perceiver()
	    except rospy.ROSInterruptException:
		pass
