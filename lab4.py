
#!usr/bin/env python
import rosbag
import numpy as np
import math
from tf.transformations import *
import rospy
import rviz
from visualization_msgs.msg import Marker
import rospy
import matplotlib.pyplot as plt
import matplotlib.patches as patches

bag = rosbag.Bag("grid.bag")
tags = [(125,525),(125,325),(125,125),(425,125),(425,325),(425,525)]
prob_array = np.zeros((35,35,5))
loc_array = np.empty((35,35,5),dtype=object)
grid_array_x = []
grid_array_y = []
tags_grid_array = []
sumt = 0.0

rvizpub = rospy.Publisher("marker",Marker,queue_size = 10)

file = open("trajectory.txt","w")
fig,ax = plt.subplots()


def draw_marker_rviz(x,y,angle):
	global rvizpub
	marker = Marker()
	marker.type = marker.CUBE
	marker.action = marker.ADD
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.orientation.w = angle
	marker.color.a = 1.0
	marker.ns = "tag_marker"
	rvizpub.publish(marker)

def get_euler_angles(quat):
	quat_array = np.array([quat.x,quat.y,quat.z,quat.w])
	tr = euler_from_quaternion(quat_array)
	ad_ang = np.degrees(np.abs(tr[2]))
	return adjust_angle(ad_ang)


def prob_motion():
	global prob_array
	prob_array[11,27,2] = 1 


def get_grid_number((x,y)):
	grid_x = x // 20 
	grid_y = y // 20 
	return (grid_x,grid_y)
	

def get_angle_grid(theta):
	grid_theta = theta // 72 

def tags_grid():
	global tags_grid_array
	for tag in tags:
		tags_grid_array.append(get_grid_number(tag))

	print tags_grid_array

def adjust_angle(angle):

	if angle < 0:
		angle = (angle + 360) % 360

	if angle > 360:
		angle = (angle) % 360

	return angle


def get_angle_distance(x,y,theta1,x1,y1,theta2):
	trans = np.sqrt((x-x1)**2 + (y-y1)**2)
	ang_diff = np.arctan2(y1-y , x1-x)
	angle = np.degrees(ang_diff)
	
	rotation1 = adjust_angle(theta1 - angle)
	rotation2 = adjust_angle(angle - theta2)
	return rotation1,trans,rotation2

def get_angle_distance_for_tag(x,y,theta1,x1,y1):
	trans = np.sqrt((x-x1)**2 + (y-y1)**2)
	
	ang_diff = np.arctan2(y-y1 , x-x1)
	angle = np.degrees(ang_diff)
	rotation1 = adjust_angle(angle - theta1)
	return rotation1,trans

def get_prob_guassian(mean,variance,value):
	numer = 1.0 /  np.sqrt(2 * np.pi * (variance ** 2))
	denum = np.power(np.e , -0.5*(((value-mean) / variance)**2))
	guass = numer * denum
	return guass

def get_grid_center(posx,posy,pos_angle):
	x_cord = (posx * 20)  +  10
	y_cord = (posy * 20) + 10
	theta  = (pos_angle * 72 ) + 36
	
	return x_cord,y_cord,theta

def get_grid_center_forTag(posx,posy):
	x_cord = (posx * 20)  +  10
	y_cord = (posy * 20) + 10
	return x_cord,y_cord

def calculate_prob(msg):
	global prob_array
	sump =0.0
	quat1 = msg.rotation1
	quat2 = msg.rotation2
	trans_dist = float(msg.translation) * 100
	rot1 = get_euler_angles(quat1)
	rot2 = get_euler_angles(quat2)
	new_prob_array = np.copy(prob_array)
	
   
	for grid_x in range(0,35):
		for grid_y in range(0,35):
			for grid_angle in range(0,5):
			
				global sumt
				sumt = 0.0
				x1,y1,theta1 = loc_array[grid_x,grid_y,grid_angle]
				for i in range(0,35):
					for j in range(0,35):
						for k in range(0,5):
							if prob_array[i,j,k] < 0.00001:
								continue

							x,y,theta = loc_array[i,j,k]
							rotation1,trans,rotation2 = get_angle_distance(x,y,theta,x1,y1,theta1)

							prob_rot = get_prob_guassian(rot1,36,rotation1) 
							prob_tran = get_prob_guassian(trans_dist,10,trans)
							prob_rot2 = get_prob_guassian(rot2,36,rotation2)
							
							
							prob =   prob_rot * prob_tran * prob_rot2 * prob_array[i,j,k]
							
							sump += prob
							sumt += prob
							
							
				new_prob_array[grid_x,grid_y,grid_angle] = sumt
				sumt = 0.0
				

	count = 0
	
	prob_array = new_prob_array / sump
	f,t,g = np.unravel_index(np.argmax(prob_array,axis = None),prob_array.shape)

	#print("Max probabilities at:", f, t ,g , "value:" ,  prob_array[f,t,g])
	#stringw = "MaximumProbability at after motion:["+str(f+1)+","+ str(t+1)+","+str(g)+"] "+str(prob_array[f,t,g])+"\n"
	#file.write(stringw)
	return True

def prob_obs(obv_msg):
	global prob_array
	tag_no = obv_msg.tagNum
	grid_x,grid_y = tags_grid_array[tag_no]
	quat1 = obv_msg.bearing
	percept_angle = get_euler_angles(quat1)
	percept_range = obv_msg.range * 100
	x2,y2 = get_grid_center_forTag(grid_x,grid_y)
	new_prob_array = np.copy(prob_array)
	sump = 0.0

	for i in range(0,35):
		for j in range(0,35):
			for k in range(0,5):
			
				x,y,theta = loc_array[i,j,k] 
				rotation1,trans = get_angle_distance_for_tag(x2,y2,theta,x,y)
				prob_rot = get_prob_guassian(percept_angle,36,rotation1)  
				prob_tran = get_prob_guassian(percept_range,10,trans)

				prob =   prob_rot * prob_tran * prob_array[i,j,k]
				
				sump += prob
				new_prob_array[i,j,k] = prob
	
	prob_array = new_prob_array / sump
	f,t,g = np.unravel_index(np.argmax(prob_array,axis = None),prob_array.shape)
	print("Max probabilities at:", f, t ,g , "value:" , prob_array[f,t,g])
	
	stringw = "Maximum Probability at after Observation:["+str(f+1)+","+ str(t+1)+","+ str(g)+"] "+str(prob_array[f,t,g])+"\n"
	file.write(stringw)
	x,y,theta = get_grid_center(f,t,g)

	#plt.scatter(x,y,marker='^',facecolor = 'none',edgecolor = 'red')
	grid_array_x.append(x)
	grid_array_y.append(y)
	#plt.plot(x,y,'.r-')
	return True
						

def prob_calc():
	
	global loc_array
	global truth
	tags_grid()
	for tag in tags_grid_array:
		x2,y2 = get_grid_center_forTag(tag[0],tag[1])
		plt.scatter(x2,y2,marker='o',facecolor = 'none',edgecolor = 'green')
	
	for i in range(0,35):
		for j in range(0,35):
			for k in range(0,5):
				
				x,y,z = get_grid_center(i,j,k)
				loc_array[i,j,k] = (x,y,z)
	
	for topic,msg,t in bag.read_messages(topics = ["Observations","Movements"]):
	
		if topic == "Movements":
			calculate_prob(msg)

		else:
			prob_obs(msg)
				
	for i in range(len(grid_array_x)):
		plt.plot(grid_array_x[:i],grid_array_y[:i],'.r-')


	plt.savefig("trajectory_plot1.png",dpi=300)	

prob_motion()
prob_calc()

'''if __name__ == '__main__':
	try:
		prob_calc()
	except rospy.ROSInterruptException:
		pass'''




		
