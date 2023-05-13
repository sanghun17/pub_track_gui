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


wp_sampling = 10
class PublishTrack():
    def __init__(self):
        self.server = InteractiveMarkerServer("track_info_interative")
        self.menu_handler = MenuHandler()
        self.menu_list = [ "Anchor1","Anchor2","Vel_Smooth","Pos_Smooth","Vel_Multiply"]
        for menu in self.menu_list:
            self.menu_handler.insert( menu , callback=self.processFeedback)
        self.cst_pub = rospy.Publisher("track_info", MarkerArray, queue_size = 1)
        self.interative_marker_sub = rospy.Subscriber("/track_info_interative/update_full",InteractiveMarkerInit,self.InterativeMarkerCallback)
        

        self.anchor1 = np.nan
        self.anchor2 = np.nan
        self.vel_smooth_pivot = np.nan
        self.pos_smooth_pivot = np.nan
        self.vel_multiply_pivot = np.nan
        self.max_id = np.nan
        # file_name = "./result/center_traj_with_boundary.txt"
        file_name = "./result/opt_traj_with_boundary.txt"
        self.copy_file_name = "./result/opt_traj_with_boundary_new.txt"
        self.track =  np.loadtxt(self.copy_file_name, delimiter=",", dtype = float)
        shutil.copy2(file_name,self.copy_file_name)
        self.publish_interative_marker(self.copy_file_name)
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.server.applyChanges()

    def InterativeMarkerCallback(self,msg):
        marker_array = MarkerArray()
        # track =  np.loadtxt(self.copy_file_name, delimiter=",", dtype = float)
        for int_marker in msg.markers:
            marker = Marker()
            marker.header = int_marker.header
            marker.header.frame_id = "map"
            marker.id = int_marker.controls[0].markers[0].id
            marker.pose = int_marker.pose
            marker.pose.orientation.x = self.track[marker.id,3] #curvature
            marker.pose.orientation.y = self.track[marker.id,4] #left_width
            marker.pose.orientation.z = self.track[marker.id,5] #right_width
            marker.pose.orientation.w = self.track[marker.id,6] # psi
            marker.ns= int_marker.controls[0].markers[0].ns
            marker.type = int_marker.controls[0].markers[0].type
            marker.action = int_marker.controls[0].markers[0].action
            marker.scale = int_marker.controls[0].markers[0].scale
            marker.color = int_marker.controls[0].markers[0].color
            marker.color.a = 1
            marker_array.markers.append(marker)
        
        self.cst_pub.publish(marker_array)
        
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

    def path_contain_zero_wp(self,m1_id, m2_id):
        diff = abs(m1_id - m2_id)
        if diff > 0.5*self.max_id:
            return True
        else:
            return False
        
    def smooth_path_xy(self, marker1_name, marker2_name):
        m1_p = self.server.get(marker1_name).pose.position
        m2_p = self.server.get(marker2_name).pose.position
        m1_p = np.array([m1_p.x, m1_p.y])
        m2_p = np.array([m2_p.x, m2_p.y])
        m1_id = int(marker1_name[2:])
        m2_id = int(marker2_name[2:])
        direction = m2_p - m1_p
        distance = np.linalg.norm(direction)
        unit_vector = direction / distance
        
        if self.path_contain_zero_wp(m1_id, m2_id):
            step_size = distance / (self.max_id - abs(m1_id - m2_id))
            if m1_id < m2_id:
                tmp1 = range(m1_id, -1,-1)
                tmp2 = range(self.max_id,m2_id-1,-1)
                smooth_range = list(tmp1)+list(tmp2)
            else:
                tmp1 = range(m2_id,-1,-1)
                tmp2 = range(self.max_id,m1_id-1,-1)
                smooth_range = list(tmp1)+list(tmp2)
        else:
            step_size = distance/ abs(m1_id - m2_id)
            if m1_id < m2_id:
                smooth_range = range(m1_id, m2_id+1,1)
            else:
                smooth_range = range(m1_id, m2_id-1,-1)

        for i in smooth_range:
            if self.path_contain_zero_wp(i,m1_id):
                offset = (self.max_id - abs(i-m1_id)) * step_size
            else:
                offset = abs(i-m1_id) * step_size

            position = tuple(m1_p + offset*unit_vector)
            marker_name = "wp"+str(i)
            ori_pose = self.server.get(marker_name).pose
            new_pose = Pose()
            new_pose.position.x = position[0]
            new_pose.position.y = position[1]
            new_pose.position.z = ori_pose.position.z
            new_pose.orientation = ori_pose.orientation
            self.server.setPose(marker_name, new_pose)

        self.menu_handler.reApply( self.server )
        self.server.applyChanges()
        return
    
    def smooth_path_z(self, marker1_name, marker2_name):
        m1_p = self.server.get(marker1_name).pose.position
        m2_p = self.server.get(marker2_name).pose.position
        m1_p = np.array([m1_p.z])
        m2_p = np.array([m2_p.z])
        m1_id = int(marker1_name[2:])
        m2_id = int(marker2_name[2:])
        direction = m2_p - m1_p
        distance = np.linalg.norm(direction)
        unit_vector = direction / distance
        
        if self.path_contain_zero_wp(m1_id, m2_id):
            step_size = distance / (self.max_id - abs(m1_id - m2_id))
            if m1_id < m2_id:
                tmp1 = range(m1_id, -1,-1)
                tmp2 = range(self.max_id,m2_id-1,-1)
                smooth_range = list(tmp1)+list(tmp2)
            else:
                tmp1 = range(m2_id,-1,-1)
                tmp2 = range(self.max_id,m1_id-1,-1)
                smooth_range = list(tmp1)+list(tmp2)
        else:
            step_size = distance/ abs(m1_id - m2_id)
            if m1_id < m2_id:
                smooth_range = range(m1_id, m2_id+1,1)
            else:
                smooth_range = range(m1_id, m2_id-1,-1)

        for i in smooth_range:
            if self.path_contain_zero_wp(i,m1_id):
                offset = (self.max_id - abs(i-m1_id)) * step_size
            else:
                offset = abs(i-m1_id) * step_size

            position = tuple(m1_p + offset*unit_vector)
            marker_name = "wp"+str(i)
            ori_pose = self.server.get(marker_name).pose
            new_pose = Pose()
            new_pose.position.x = ori_pose.position.x
            new_pose.position.y = ori_pose.position.y
            new_pose.position.z = position[0]
            new_pose.orientation = ori_pose.orientation
            self.server.setPose(marker_name, new_pose)

        self.menu_handler.reApply( self.server )
        self.server.applyChanges()
        return

    def multiply_path_z(self, marker1_name, marker2_name):
        m1_p = self.server.get(marker1_name).pose.position
        m2_p = self.server.get(marker2_name).pose.position
        m1_p = np.array([m1_p.z])
        m2_p = np.array([m2_p.z])
        m1_id = int(marker1_name[2:])
        m2_id = int(marker2_name[2:])
        pivot_id = int(self.vel_multiply_pivot[2:])
        marker_name = self.vel_multiply_pivot
        pivot_p = self.server.get(marker_name).pose
        if self.path_contain_zero_wp(m1_id, m2_id):
            if m1_id < m2_id:
                tmp1 = range(m1_id, -1,-1)
                tmp2 = range(self.max_id,m2_id-1,-1)
                smooth_range = list(tmp1)+list(tmp2)
            else:
                tmp1 = range(m2_id,-1,-1)
                tmp2 = range(self.max_id,m1_id-1,-1)
                smooth_range = list(tmp1)+list(tmp2)
        else:
            if m1_id < m2_id:
                smooth_range = range(m1_id, m2_id+1,1)
            else:
                smooth_range = range(m1_id, m2_id-1,-1)
        
        
        rate = pivot_p.position.z / self.track[pivot_id,2]
        for i in smooth_range:
            marker_name = "wp"+str(i)
            ori_pose = self.server.get(marker_name).pose
            new_pose = Pose()
            new_pose.position.x = ori_pose.position.x
            new_pose.position.y = ori_pose.position.y
            if i==pivot_id or i==m1_id or i == m2_id:
                new_pose.position.z = ori_pose.position.z
            else:
                new_pose.position.z =  self.track[i,2]*rate
            new_pose.orientation = ori_pose.orientation
            self.server.setPose(marker_name, new_pose)

        self.menu_handler.reApply( self.server )
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
                self.smooth_path_z(self.anchor1,self.vel_smooth_pivot)
                self.smooth_path_z(self.anchor2,self.vel_smooth_pivot)
                print("smooth vel finished")
            elif selected_menu == "Pos_Smooth":
                self.pos_smooth_pivot = feedback.marker_name
                self.smooth_path_xy(self.anchor1,self.pos_smooth_pivot)
                self.smooth_path_xy(self.anchor2,self.pos_smooth_pivot)
                print("smooth pos finished")
            elif selected_menu == "Vel_Multiply":
                self.vel_multiply_pivot = feedback.marker_name
                self.multiply_path_z(self.anchor1, self.anchor2)
                print("multiply vel finished")
            else:
                print("something wrong..")
                exit()
        else: 
            print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z) )

        return

    def publish_interative_marker(self, filename):
        # track =  np.loadtxt(filename, delimiter=",", dtype = float)
        id = 0
        for i in range(len(self.track)):
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.name ="wp"+str(id)
            if i%wp_sampling ==0:
                int_marker.description = str(id)
            int_marker.header.stamp = rospy.Time.now()
            int_marker.scale = 0.5
            int_marker.pose.position.x = self.track[i,0] #position x
            int_marker.pose.position.y = self.track[i,1] #posiiton y
            int_marker.pose.position.z = self.track[i,2] #velocity
            yaw = self.track[i,6]
            qx = 0.0
            qy = 0.0
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
            # int_marker.pose.orientation.x = track[i,3] #curvature
            # int_marker.pose.orientation.y = track[i,4] #left_width
            # int_marker.pose.orientation.z = track[i,5] #right_width
            # int_marker.pose.orientation.w = track[i,6] # psi
            int_marker.pose.orientation.x = qx
            int_marker.pose.orientation.y = qy
            int_marker.pose.orientation.z = qz
            int_marker.pose.orientation.w = qw

            # # visualize marker
            tarck_marker_control = InteractiveMarkerControl()
            tarck_marker_control.always_visible = True
            track_marker = Marker()
            track_marker.id = id
            # track_marker.ns = "track"
            track_marker.header.stamp = rospy.Time.now()
            track_marker.type = Marker.SPHERE
            track_marker.ns = "track"
            track_marker.color.r, track_marker.color.g, track_marker.color.b = (0, 255, 0)
            if i%wp_sampling ==0:
                track_marker.color.a = 1
            else:
                 track_marker.color.a = 0
            track_marker.scale.x = 0.2
            track_marker.scale.y = 0.2
            track_marker.scale.z = 0.2
            self.max_id = id
            id += 1
            tarck_marker_control.markers.append(track_marker)
            int_marker.controls.append(tarck_marker_control)
            if i%wp_sampling ==0:
                # move_x
                self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_x",1,1,0,0)
                # move_y
                self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_y",1,0,0,1)
                # move_z
                self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_z",1,0,1,0)
                # menu
                self.CreateMarkerControl(int_marker,InteractiveMarkerControl.BUTTON,"menu",0,0,0,1)

            self.server.insert(int_marker, self.processFeedback)
            self.menu_handler.apply( self.server, int_marker.name )
        self.server.applyChanges()
            
            
def main():
    rospy.init_node("track_info_node")
    PublishTrack()
    
if __name__ == '__main__':
    main()