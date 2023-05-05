import rospy
import numpy as np
import time
import math
import shutil
import copy
import re

from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *




class PublishTrack():
    def __init__(self):
        self.server = InteractiveMarkerServer("track_info_interative")
        self.menu_handler = MenuHandler()
        self.menu_list = [ "Anchor1","Anchor2","Vel_Smooth","Pos_Smooth"]
        for menu in self.menu_list:
            self.menu_handler.insert( menu , callback=self.processFeedback)
        

        self.anchor1 = np.nan
        self.anchor2 = np.nan
        self.vel_smooth_pivot = np.nan
        self.pos_smooth_pivot = np.nan

        # file_name = "./result/center_traj_with_boundary.txt"
        file_name = "./result/opt_traj.txt"
        copy_file_name = "./result/opt_traj_new.txt"

        shutil.copy2(file_name,copy_file_name)
        self.read_file(copy_file_name)
        
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.server.applyChanges()

    def CreateMarkerControl(self, interaction_marker, interaction_mode, name, w,x,y,z):
        tarck_marker_control = InteractiveMarkerControl()
        tarck_marker_control.always_visible = True
        tarck_marker_control.name = name
        tarck_marker_control.orientation.w = w
        tarck_marker_control.orientation.x = x
        tarck_marker_control.orientation.y = y
        tarck_marker_control.orientation.z = z
        tarck_marker_control.interaction_mode = interaction_mode
        interaction_marker.controls.append(tarck_marker_control) 

    def smooth_path(self,smooth_quantity):
        anchor1_marker_position = self.server.get(self.anchor1).pose.position
        anchor2_marker_position = self.server.get(self.anchor2).pose.position
        if smooth_quantity == "pos":
            pivot_marker_position = self.server.get(self.pos_smooth_pivot).pose.position
        elif smooth_quantity == "vel":
            pivot_marker_position = self.server.get(self.vel_smooth_pivot).pose.position

        pivot_marker_position = np.array([pivot_marker_position.x, pivot_marker_position.y, pivot_marker_position.z])
        anchor1_marker_position = np.array([anchor1_marker_position.x, anchor1_marker_position.y,anchor1_marker_position.z])
        anchor2_marker_position = np.array([anchor2_marker_position.x, anchor2_marker_position.y,anchor2_marker_position.z])
        # Compute the step size for interpolating between anchor1 and pivot
        if smooth_quantity == "pos":
            first_number_tmp = int(re.search(r'\d+', self.pos_smooth_pivot).group()), int(re.search(r'\d+', self.anchor1).group())
        elif smooth_quantity == "vel":
            first_number_tmp = int(re.search(r'\d+', self.vel_smooth_pivot).group()), int(re.search(r'\d+', self.anchor1).group())
        first_number = min(first_number_tmp[0],first_number_tmp[1])
        last_number = max(first_number_tmp[0],first_number_tmp[1])
        # Compute the direction and distance between the anchor1 and pivot markers
        direction = pivot_marker_position-anchor1_marker_position  
        if first_number_tmp[0] < first_number_tmp[1]:
            direction = -1* direction
        distance = np.linalg.norm(direction)       
        step_size = distance / (last_number - first_number)
        # Iterate over the range of marker numbers to interpolate
        for i in range(first_number, last_number+1):
            # Compute the position of the current marker using linear interpolation
            offset = (i - first_number) * step_size
            position = tuple(anchor1_marker_position + offset * direction / distance)
            if first_number_tmp[0] < first_number_tmp[1]:
                position = tuple(pivot_marker_position + offset * direction / distance)
            # Update the position of the current marker
            marker_name = "wp{:02d}".format(i)
            ori_pose = self.server.get(marker_name).pose
            new_pose = Pose()
            if smooth_quantity =="pos":
                new_pose.position.x = position[0]
                new_pose.position.y = position[1]
                new_pose.position.z = ori_pose.position.z
            elif smooth_quantity == "vel":
                new_pose.position.x = ori_pose.position.x
                new_pose.position.y = ori_pose.position.y
                new_pose.position.z = position[2]
            new_pose.orientation = ori_pose.orientation
            self.server.setPose(marker_name, new_pose)       # Compute the direction and distance between the anchor1 and pivot markers       # Compute the step size for interpolating between anchor1 and pivot
        
        if smooth_quantity == "pos":
            first_number_tmp = int(re.search(r'\d+', self.pos_smooth_pivot).group()), int(re.search(r'\d+', self.anchor2).group())
        elif smooth_quantity == "vel":
            first_number_tmp = int(re.search(r'\d+', self.vel_smooth_pivot).group()), int(re.search(r'\d+', self.anchor2).group())
        first_number = min(first_number_tmp[0],first_number_tmp[1])
        last_number = max(first_number_tmp[0],first_number_tmp[1])
        direction = anchor2_marker_position - pivot_marker_position
        if first_number_tmp[0] > first_number_tmp[1]:
            direction = -1* direction
        distance = np.linalg.norm(direction)
        step_size = distance / (last_number - first_number)
        # Iterate over the range of marker numbers to interpolate
        for i in range(first_number, last_number+1):
            # Compute the position of the current marker using linear interpolation
            offset = (i - first_number) * step_size
            position = tuple(pivot_marker_position + offset * direction / distance)
            if first_number_tmp[0] > first_number_tmp[1]:
                position = tuple(anchor2_marker_position + offset * direction / distance)
            # Update the position of the current marker
            marker_name = "wp{:02d}".format(i)
            ori_pose = self.server.get(marker_name).pose
            new_pose = Pose()
            if smooth_quantity =="pos":
                new_pose.position.x = position[0]
                new_pose.position.y = position[1]
                new_pose.position.z = ori_pose.position.z
            elif smooth_quantity == "vel":
                new_pose.position.x = ori_pose.position.x
                new_pose.position.y = ori_pose.position.y
                new_pose.position.z = position[2]
            new_pose.orientation = ori_pose.orientation
            self.server.setPose(marker_name, new_pose)

        self.server.applyChanges()
        return
    
    def processFeedback(self,feedback):
        p = feedback.pose.position

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            selected_menu = self.menu_list[feedback.menu_entry_id-1]
            print(feedback.marker_name + ": menu item " + selected_menu + " clicked" )
            if selected_menu == "Anchor1":
                self.anchor1 = feedback.marker_name
            elif selected_menu == "Anchor2":
                self.anchor2 = feedback.marker_name
            elif selected_menu == "Vel_Smooth":
                self.vel_smooth_pivot = feedback.marker_name
                self.smooth_path("vel")
                print("smooth vel finished")
            elif selected_menu == "Pos_Smooth":
                self.pos_smooth_pivot = feedback.marker_name
                self.smooth_path("pos")
                print("smooth pos finished")
            else:
                print("something wrong..")
                exit()
        else: 
            print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z) )

        return

    def read_file(self, filename):
        track = np.loadtxt(filename, delimiter=",", dtype = float)
        
        id = 0
        for i in range(len(track)):
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.name = "wp"+str(id)
            int_marker.header.stamp = rospy.Time.now()
            int_marker.scale = 0.5
            int_marker.pose.position.x = track[i,0] #position x
            int_marker.pose.position.y = track[i,1] #posiiton y
            int_marker.pose.position.z = track[i,2] #velocity
            int_marker.pose.orientation.x = track[i,3] #curvature
            int_marker.pose.orientation.y = track[i,4] #left_width
            int_marker.pose.orientation.z = track[i,5] #right_width

            # # visualize marker
            tarck_marker_control = InteractiveMarkerControl()
            tarck_marker_control.always_visible = True
            track_marker = Marker()
            track_marker.id = id
            track_marker.ns = "track"
            track_marker.header.stamp = rospy.Time.now()
            track_marker.type = Marker.SPHERE
            track_marker.color.r, track_marker.color.g, track_marker.color.b = (0, 255, 0)
            track_marker.color.a = 1
            track_marker.scale.x = 0.2
            track_marker.scale.y = 0.2
            track_marker.scale.z = 0.2
            id += 1
            int_marker.controls.append(tarck_marker_control) 
            tarck_marker_control.markers.append(track_marker)

            # # move_x
            self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_x",1,1,0,0)
            # move_y
            self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_y",1,0,0,1)
            # move_z
            self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_y",1,0,1,0)

            # menu
            self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MENU,"menu",0,0,0,1)

            self.server.insert(int_marker, self.processFeedback)
            self.menu_handler.apply( self.server, int_marker.name )
            self.server.applyChanges()
            
            
def main():
    rospy.init_node("track_info_node")
    PublishTrack()
    
if __name__ == '__main__':
    main()