
# Copyright 2024, Christophe VIEL
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted # provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

import imutils  # pour rotation image

import gi
import imutils
import numpy as np
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst

from message2.msg import Detection
from std_msgs.msg import Float64, Bool, Float32MultiArray
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu

from scipy.spatial.transform import Rotation 

class Affichage(Node):

    def __init__(self,name,id):


        super().__init__('Affichage_node_'+name)

        self.fps = 15

        # pour reception des informations

        """
        self.declare_parameter('ROV_name', '')
        self.ROV_name = self.get_parameter('ROV_name').get_parameter_value().string_value
        self.declare_parameter('ID', 1.0)
        ID = self.get_parameter('ID').get_parameter_value().double_value
        self.ID = str(ID)
        self.ID0 = ID 
        self.declare_parameter('nb_ROV', 1.0)
        self.nb_ROV = self.get_parameter('nb_ROV').get_parameter_value().double_value
        self.declare_parameter('biais_viseur', 0.0)
        self.biais_box = self.get_parameter('biais_viseur').get_parameter_value().double_value
        """

  
        self.declare_parameter('ROV_name_'+str(int(id)), '')
        self.ROV_name = self.get_parameter('ROV_name_'+str(int(id))).get_parameter_value().string_value
        self.ID = str(id)
        self.ID0 = id
        self.declare_parameter('nb_ROV', 1.0)
        self.nb_ROV = self.get_parameter('nb_ROV').get_parameter_value().double_value
        self.declare_parameter('biais_viseur_'+str(int(id)), 0.0)
        self.biais_box = self.get_parameter('biais_viseur_'+str(int(id))).get_parameter_value().double_value
  
        self.nb_ROV = int(self.nb_ROV)
        """
        self.ROV_name = ROV_name
        self.ID = str(ID)
        self.ID0 = ID
        self.nb_ROV = nb_ROV
        self.biais_box = biais_viseur
        """


        if not(self.ROV_name == ''):
            self.ns = "/"+ self.ROV_name 
        else:
            self.ns = ""


        self.cam_active = 1
        self.target_cam = 0
        
        self.joy_active = 1.0
        self.piloted = False
        
        self.tracking  = False
        
        # Detection
        self.bbox_detected = [0,0,0,0]
        self.detected = False
        self.detected_x = 0.0
        self.detected_y = 0.0
        self.detected_w = 0.0
        self.detected_h = 0.0

        ROOT_DIR = os.path.expanduser('~')
        self.filename =  ROOT_DIR + "/BlueROV_multi_ros2/" 
        #self.filename =  "/home/bluerovpc/BlueROV_multi_ros2/" # TODO : voir si on peut améliorer...

        if self.filename == 'fail':
            self.affichage_boussole = False
        else:
            self.affichage_boussole = True
            self.filename = self.filename + 'boussole4.png'
            self.boussole = cv2.imread(self.filename,0)
           
 

        self.declare_parameter('display_length_video', 800.0)
        self.display_length_video = self.get_parameter('display_length_video').get_parameter_value().double_value
        #self.display_length_video = int(rospy.get_param('~display_length_video',"800"))   # largeur de la vidéo à afficher (independant de la taille de l'image du stream)

        self.bridge = CvBridge()
        self.frame1 = 0 
        self.init_frame1 = 0
        self.frame1_size = (int(0), int(0))
        self.frame2 = 0 
        self.init_frame2 = 0
        self.frame2_size = (int(0), int(0))

        self.frame_size = (int(0), int(0))

        self.heading = 0
        self.altitude = 0
        self.distance = 0
        self.target_heading = 0
        self.target_depth = 0
        self.target_distance = 0
        self.Phi, self.Theta, self.Psy = 0.0, 0.0, 0.0
        
        # listener
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.queue_listener = 10
        self.listener()
        #######

        timer_period = 1/self.fps # seconds
        #self.timer = self.create_timer(timer_period, self.run)
        

    def draw_bbox(self, img, bbox, color = (0, 0, 255)):
        if bbox is not None:
            p1 = (int(bbox[0]), int(bbox[1]))  # start point
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])) # end point
            cv2.rectangle(img, p1, p2, color, 2, 1)



    def add_cap(self,display_img,taille1):         
                    coef = 10

                    boussole = imutils.rotate(self.boussole, angle=self.heading)
                    #taille = (int(self.display_length_video/coef), int(self.frame_size[0]/self.frame_size[1]*self.display_length_video/coef) )
                    taille = (int(self.display_length_video/coef), int(self.display_length_video/coef) )
                    boussole_resize = cv2.resize(boussole, (int(taille[1]), int(taille[0])) )
                    nx = 0 # taille1[1]-taille[0]
                    ny = int(taille1[0]-taille[1])
                    
                    boussole_resize[boussole_resize> 10] = 255
                    boussole_resize[boussole_resize< 255] = 0

                    display_img[0+nx:taille[0]+nx, 0+ny:taille[1]+ny,0] = boussole_resize
                    display_img[0+nx:taille[0]+nx, 0+ny:taille[1]+ny,1] = boussole_resize
                    display_img[0+nx:taille[0]+nx, 0+ny:taille[1]+ny,2] = boussole_resize



    def listener(self):

        #rospy.Subscriber(self.ns+"/mavros/global_position/compass_hdg", Float64, self.callback_compass)  
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/compass_hdg',
            self.callback_compass,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/mavros/global_position/rel_alt", Float64, self.callback_alt)
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/rel_alt',
            self.callback_alt,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning


       #rospy.Subscriber(self.ns+"/distance", Float64, self.callback_distance)
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/distance',
            self.callback_distance,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/target_heading", Float64, self.callback_target_heading)
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/target_heading',
            self.callback_target_heading,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/target_depth", Float64, self.callback_target_alt)
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/target_depth',
            self.callback_target_alt,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/target_distance", Float64, self.callback_target_distance)
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/target_distance',
            self.callback_target_distance,
            self.queue_listener)
        self.subscription  # prevent unused variable warning
        
        #rospy.Subscriber(self.ns+"/tracking_buoy", Bool, self.callback_start_tracking)
        self.subscription = self.create_subscription(
            Bool,
            self.ns+"/tracking_buoy",
            self.callback_start_tracking,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/cam_active", Float64, self.callback_cam_active)
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/cam_active',
            self.callback_cam_active,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/target_cam", Float64, self.callback_target_cam)
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/target_cam',
            self.callback_target_cam,
            self.queue_listener)
        self.subscription  # prevent unused variable warning
        
        #rospy.Subscriber(self.ns+"/mavros/imu/data", Imu, self.callback_imu)
        self.subscription = self.create_subscription(
            Imu,
            self.ns+"/mavros/imu/data",
            self.callback_imu,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+'/cam_1/compressed', CompressedImage, self.callback_cam1)
        self.subscription = self.create_subscription(
            CompressedImage,
            self.ns+"/cam_1/compressed",
            self.callback_cam1,
            self.queue_listener)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+'/cam_2/compressed', CompressedImage, self.callback_cam2)
        self.subscription = self.create_subscription(
            CompressedImage,
            self.ns+"/cam_2/compressed",
            self.callback_cam2,
            self.queue_listener)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/cam_detection", Detection, self.callback_detection)
        self.subscription = self.create_subscription(
            Detection,
            self.ns+"/cam_detection",
            self.callback_detection,
            self.queue_listener)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/cam_detection_2", Detection, self.callback_detection_2)
        self.subscription = self.create_subscription(
            Detection,
            self.ns+"/cam_detection_2",
            self.callback_detection_2,
            self.queue_listener)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        #rospy.Subscriber(self.ns+"/box_detected", Float32MultiArray, self.callback_box_detected)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.ns+"/box_detected",
            self.callback_box_detected,
            self.queue_listener)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning
        
        #rospy.Subscriber("/num_joy_control", Float64, self.callback_joy_switch)
        self.subscription = self.create_subscription(
            Float64,
            "/num_joy_control",
            self.callback_joy_switch,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

    def callback_joy_switch(self, data):
        self.joy_active = data.data
        if (self.joy_active == self.ID0):
            self.piloted = True
        else:
            self.piloted = False


    def callback_imu(self, msg):
        self.angular_velocity_z = msg.angular_velocity.z  # vitesse de lacet
        self.angular_velocity_y = msg.angular_velocity.y  # (vitesse de tangage ?)
        self.angular_velocity_x = msg.angular_velocity.x  # (vitesse de roulis ?)
        W = msg.orientation.w
        X = msg.orientation.x
        Y = msg.orientation.y
        Z = msg.orientation.z
        orientq=(W, X, Y, Z)
        ### Conversion quaternions in rotation matrix
        Phi_rad, Theta_rad, Psy_rad = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet     
        self.Phi, self.Theta, self.Psy = Phi_rad , Theta_rad, Psy_rad     
            
        
    def callback_start_tracking(self,data):
        self.tracking = data.data
        

    def callback_compass(self,data):
        self.heading = data.data

    
    def callback_alt(self,data):
        self.altitude = -data.data

    def callback_distance(self,data):
        self.distance = data.data
        
    def callback_target_heading(self,data):
        self.target_heading = data.data
        
    def callback_target_alt(self,data):
        self.target_depth = data.data
        
    def callback_target_distance(self,data):
        self.target_distance = data.data
        
    def callback_cam_active(self,data):
        self.cam_active = data.data                       	

    def callback_target_cam(self,data):
        self.target_cam = data.data        

    def callback_cam1(self,data):

        
        """
        try:
           # Convert your ROS Image message to OpenCV2
           self.frame1 = self.bridge.imgmsg_to_cv2(data, "bgr8")    
        except CvBridgeError as e:
           print(e)
           
        self.init_frame1 = 1
        self.frame1_size = (int(data.width), int(data.height))
        
        
        """
        try:
           # Convert your ROS CompressedImage message to OpenCV2
           self.frame1 = self.bridge.compressed_imgmsg_to_cv2(data)    
        except CvBridgeError as e:
           print(e)
           
        self.init_frame1 = 1
        h,w,d = self.frame1.shape
        self.frame1_size = (w, h)
        #self.frame1_size = (int(data.width), int(data.height))
        #"""
        
        
        
       
    def callback_cam2(self,data):
    
            
        """
        self.frame2 = self.bridge.imgmsg_to_cv2(data, "bgr8") 
        self.init_frame2 = 1
        self.frame2_size = (int(data.width), int(data.height))
        """
        self.frame2 = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8") 
        self.init_frame2 = 1
        h,w,d = self.frame2.shape
        self.frame2_size = (w, h)
        #self.frame2_size = (int(data.width), int(data.height))
        #"""
       
       
    def callback_detection(self, msg):
    
        if self.cam_active == 1: 
            self.detected = msg.detected
            self.timer_max = msg.time
            if self.detected:
                self.detected_x = msg.x
                self.detected_y = msg.y
                self.detected_w = msg.w
                self.detected_h = msg.h
                self.distance = -np.log10(max(0.001, self.detected_h))


    def callback_detection_2(self, msg):
    
        if self.cam_active == 2: 
            self.detected = msg.detected
            self.timer_max = msg.time
            if self.detected:
                self.detected_x = msg.x
                self.detected_y = msg.y
                self.detected_w = msg.w
                self.detected_h = msg.h
                self.distance = -np.log10(max(0.001, self.detected_h))

                self.bbox_detected[0] = msg.x - msg.w
                self.bbox_detected[2] = msg.x + msg.w
                self.bbox_detected[1] = msg.y - msg.h
                self.bbox_detected[3] = msg.y + msg.h
                
    def callback_box_detected(self, msg):

        self.bbox_detected[0] = msg.data[0]
        self.bbox_detected[1] = msg.data[1]
        self.bbox_detected[2] = msg.data[2]
        self.bbox_detected[3] = msg.data[3]


                
####################################"



    def run(self):

        
        # additionnal information
        

        if (self.cam_active == 1)&(self.init_frame1 == 1):
        
            self.display_img = self.frame1
            
            self.frame_size = self.frame1_size
            
        elif (self.cam_active == 2)&(self.init_frame2 == 1):
        
            self.display_img = self.frame2
            
            self.frame_size = self.frame2_size
        
        
        
        ########## Affichage supplémentaire
        biais_box = 0.0 # self.biais_box

        
        if (self.init_frame1 == 1)|(self.init_frame1 == 2): # self.cam_active == self.num_cam:  # si on est bien la camera active
                        
            length_box = 100  # taille du cote de la boite en pixel
            box_center = ( int(self.frame_size[0]/2-length_box/2), int(self.frame_size[1]/2-length_box/2), int(length_box), int(length_box) ) #box_center = [start point x, star point y, length x, kength y]
            box_center_pince = ( int(self.frame_size[0]/2-length_box/2-biais_box), int(self.frame_size[1]/2-length_box/2), int(length_box), int(length_box) ) #box_center = [start point x, star point y, length x, kength y]            
            

            # croix
            cross_center1 = ( int(self.frame_size[0]/2-length_box/2), int(self.frame_size[1]/2), int(length_box), 1 ) #box_center = [start point x, star point y, length x, kength y]
            cross_center2 = ( int(self.frame_size[0]/2), int(self.frame_size[1]/2)-length_box/2, 1, int(length_box) ) #box_center = [start point x, star point y, length x, kength y]

        

            if self.tracking == True :

                self.draw_bbox(self.display_img, self.bbox_detected) # red box
                self.draw_bbox(self.display_img, cross_center1,[200,200,200])
                self.draw_bbox(self.display_img, cross_center2,[200,200,200]) 
                
                if (self.target_cam == 1):# &(self.cam_active ==1):
                    self.draw_bbox(self.display_img, box_center_pince,[200,200,200])  

            else:
            
                self.draw_bbox(self.display_img, box_center,[200,200,200]) # grey box
                if (self.target_cam == 1): # &(self.cam_active ==1):
                    line_center = ( 0, int(self.frame_size[1]/2), int(2*length_box), 1 ) #box_center = [start point x, star point y, length x, kength y]
                    cross_center3 = ( int(self.frame_size[0]/2-length_box/2-biais_box), int(self.frame_size[1]/2), int(length_box), 1 ) #box_center = [start point x, star point y, length x, kength y]
                    cross_center4 = ( int(self.frame_size[0]/2-biais_box), int(self.frame_size[1]/2)-length_box/2, 1, int(length_box) ) #box_center = [start point x, star point y, length x, kength y]

                    self.draw_bbox(self.display_img, cross_center3,[200,200,200])
                    self.draw_bbox(self.display_img, cross_center4,[200,200,200])                 
                    self.draw_bbox(self.display_img, line_center,[200,200,200]) 

            
        
        
        #########################################"
        # redimensionnement
            
        if (self.cam_active == 1)&(self.init_frame1 == 1):
            
            
            self.display_img = self.frame1
            self.frame_size = self.frame1_size
            
            # affichage ligne d'horizon
            length_line = 100
            start_point = (int(self.frame_size[0]/2 + length_line*np.cos(self.Phi)), int(self.frame_size[1]/2 - length_line*np.sin(self.Phi) ) )
            end_point = (int(self.frame_size[0]/2 - length_line*np.cos(self.Phi) ), int(self.frame_size[1]/2 + length_line*np.sin(self.Phi)))
            cv2.line(self.display_img, start_point, end_point, [200,200,200], 3) 
            
            
            ######
            # affichage ligne d'altitude (en test)
            length_line = 50
            alt = -(self.Theta*2/3.14)*self.frame_size[1]/2 # angle/90 pour monter jusqu'à la bordure quand angle = 90°
            start_point = (int(self.frame_size[0]/2 + length_line), int(self.frame_size[1]/2 + alt ) )
            end_point = (int(self.frame_size[0]/2 - length_line ), int(self.frame_size[1]/2 + alt))
            cv2.line(self.display_img, start_point, end_point, [200,200,200], 3) 
            ########
            

            # redimensionner image pour l'affichage
            self.display_img = cv2.resize(self.display_img, (int(self.display_length_video), int(self.frame_size[1]/self.frame_size[0]*self.display_length_video)  ))
            
            
            taille1 = (self.display_length_video, int(self.frame_size[1]/self.frame_size[0]*self.display_length_video) )



            # ajout de la boussole
            if self.affichage_boussole: 
                self.add_cap(self.display_img,taille1)


            
            
            if (self.init_frame2 == 1):
                
                
            
                self.display_img2 = self.frame2
                self.frame_size = self.frame2_size
                # redimensionner image pour l'affichage
                coef = 5
                taille = (int(self.display_length_video/coef), int(self.frame_size[0]/self.frame_size[1]*self.display_length_video/coef) )
                
                self.display_img2 = cv2.resize(self.display_img2, (taille[1], taille[0]) )

                nx = int(taille1[1]-taille[0])
                ny = int(taille1[0]-taille[1])
                self.display_img[0+nx:taille[0]+nx, 0+ny:taille[1]+ny,:] = self.display_img2
                
                
                
        elif (self.cam_active == 2)&(self.init_frame2 == 1):
        
            self.display_img = self.frame2
            self.frame_size = self.frame2_size
            
            # redimensionner image pour l'affichage
            self.display_img = cv2.resize(self.display_img, (int(self.display_length_video), int(self.frame_size[1]/self.frame_size[0]*self.display_length_video)  ))
                
                
            taille1 = (int(self.display_length_video), int(self.frame_size[1]/self.frame_size[0]*self.display_length_video) )
                
                
            # ajout de la boussole
            if self.affichage_boussole: 
                self.add_cap(self.display_img,taille1)
                
            if (self.init_frame1 == 1):
                
                
                self.display_img2 = self.frame1
                self.frame_size = self.frame1_size
                # redimensionner image pour l'affichage
                coef = 5
                taille = (int(self.display_length_video/coef), int(self.frame_size[0]/self.frame_size[1]*self.display_length_video/coef) )
                
                self.display_img2 = cv2.resize(self.display_img2, (taille[1], taille[0]) )

                nx = taille1[1]-taille[0]
                ny = taille1[0]-taille[1]
                self.display_img[0+nx:taille[0]+nx, 0+ny:taille[1]+ny,:] = self.display_img2
                
                #self.display_img[10:110, 10:110,:] =  np.zeros(100,100,3 ) 
                
        

        
            
            
        if ((self.init_frame1 == 1)|(self.init_frame1 == 2))&(self.joy_active == self.ID0):    
            cv2.imshow("Camera ROV active", self.display_img)
            cv2.waitKey(1)  & 0xFF
            
            cv2.moveWindow("Camera ROV active", 50, 1) # note: le +50 pour la largeur des barres outils de linux
            



def main(args=None):


    rclpy.init(args=args)



    node_affichage_1 = Affichage("rov_1",1)
    node_affichage_2 = Affichage("rov_2",2)
    node_affichage_3 = Affichage("rov_3",3)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_affichage_1)
    executor.add_node(node_affichage_2)
    executor.add_node(node_affichage_3)

    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    #rate = node_affichage_1.create_rate(10) # TODO: a modifier/choisir


    List_affichage = []
    List_affichage.append(node_affichage_1)
    List_affichage.append(node_affichage_2)
    nb_ROV = node_affichage_1.nb_ROV

    # on ajoute le 3ieme dans la liste que si il existe. Au pire on a lancer un node avec un run très court car il ne recevra jamais d'image
    if nb_ROV > 2:
        List_affichage.append(node_affichage_3)



    try:
        while rclpy.ok():
            
            #print('multi_affichage, go!')

            ###############################
            # (normalement, l'affichage principal est géré par le "run")
            node_affichage_1.run()
            node_affichage_2.run()
            if nb_ROV > 2:
                node_affichage_3.run()

            # Affichage secondaire
            if (node_affichage_1.frame_size[0]>0.0)&(node_affichage_1.init_frame1 == 1):
                coef = 5
                taille = (int(node_affichage_1.display_length_video/coef), int(node_affichage_1.frame_size[0]/node_affichage_1.frame_size[1]*node_affichage_1.display_length_video/coef) )

                taille_2 = (int(taille[0]*nb_ROV) , int(taille[1]))  
                
                sum_img = cv2.resize(node_affichage_1.frame1, (taille_2[1], taille_2[0]) ) # np.zeros(taille_2[0], taille_2[1], np.uint8) # image vierge
                for i in range(0,int(nb_ROV)):
                
                    affichage_i = List_affichage[i]
                    #affichage_i.run()  # TODO <- je pense que c'est inutile, à voir
                    
                    if affichage_i.init_frame1 == 1:
                        
                        display_img2 = affichage_i.display_img # frame1 # on recupere l'image
                        display_img2 = cv2.resize(display_img2, (taille[1], taille[0]) ) # redimensionne l image
                        ny = int(taille[0]*i)
                        sum_img[0+ny:taille[0]+ny, :,:] = display_img2 # position de l'image
                
                cv2.imshow("ROVs camera", sum_img)    
                cv2.moveWindow("ROVs camera", int(node_affichage_1.display_length_video+20)+50, 1) # note: le +50 pour la largeur des barres outils de linux
                
            #######################            
            #rate.sleep()
            time.sleep(1/node_affichage_1.fps)

    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()




if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
