#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import random

workableValues = {}
obstacle = False;
class RansacIteration:
	xr1 = 0.0
	yr1 = 0.0
	xr2 = 0.0
	yr2 = 0.0
	inliers = 0
	inliersX = []
	inliersY = []

def callbackLaser(msg):
	global workableValues
	workableValues = {}
	workableValues.clear()
	j = 0
	for detectedRange in msg.ranges:
		if(detectedRange < 1.5):
			global obstacle
			obstacle = True;

		if(detectedRange !=3.0):
			angleWorkable = -1.57079637051 + (j * 0.00872664619237)
			workableValues[angleWorkable] = detectedRange
		j = j+1
	ransac()
def perceiver():
	rospy.init_node('perceiver', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=2)
	rospy.Subscriber("base_scan", LaserScan, callbackLaser, queue_size=1)
	while not rospy.is_shutdown():
		vel = geometry_msgs.msg.Twist()
		lin_vel =0.0
		ang_vel = 0.0
		global obstacle
		if(obstacle):
			ang_vel = 0.0#1.75
			lin_vel = 0.0
			obstacle = False
		vel.angular.z = ang_vel
		vel.linear.x = lin_vel
		#rospy.loginfo(vel)
		pub.publish(vel)

		rate.sleep()

def ransac():
	ransackMarker = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
	global workableValues
	#workableValues["1"] = "Sachine Tendulkar"
	ransacIterations = 10
	inlierThreshold = 0.05
	minDictionaryLength = 30
	markerLines = MarkerArray()
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
					distance =  math.hypot(lenx, leny)#((x - x1) * lenx + (y - y1) * leny) / lenSegment
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
			line_color = ColorRGBA()
			line_color.r = 1
			line_color.g = 1
			line_color.b = 1
			line_color.a = 1.0

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
				start_point = Point()
				start_point.x = farthestX1
				start_point.y = farthestY1
				start_point.z = 0.0
				end_point = Point()
				end_point.x = farthestX2
				end_point.y = farthestY2
				end_point.z = 0.0
				markerFound = Marker()
				#markerFound.id = 3
				markerFound.header.frame_id = 'base_laser_link'
				markerFound.type = Marker.LINE_STRIP
				markerFound.ns = 'DetectedLine'
				markerFound.action = 0
				markerFound.scale.x = 0.1
				markerFound.points.append(start_point)
				markerFound.points.append(end_point)
				markerFound.colors.append(line_color)
				markerFound.colors.append(line_color)
				markerLines.markers.append(markerFound)
				break
			workableValues.pop(angleRad1, None)
			workableValues.pop(angleRad2, None)
			#del workableValues[angleRad2]
		ransackMarker.publish(markerLines);
if __name__ == '__main__':
	    try:
		perceiver()
	    except rospy.ROSInterruptException:
		pass
