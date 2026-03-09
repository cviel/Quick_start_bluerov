

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


# Note : by default, angles provid by sensor are in deg. Variable with a "rad" are expressed in radian

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile


import datetime
import time
import subprocess, os, signal, psutil


#import parameters as param
from scipy.spatial.transform import Rotation 

import numpy as np
import math

from mavros import command as mavros_command
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String, Bool, Float32MultiArray

from std_srvs.srv import Trigger #,  TriggerResponse


import importlib

import time

#################################################################

def light_control(self,light_modif_value):


    adresse_ip0 = self.adresse_ip

    pwm_light0 = self.pwm_light+light_modif_value
    if (pwm_light0 > 1900):
        pwm_light0 = 1000.0
    if (pwm_light0 < 1000):
        pwm_light0 = 1900.0

    self.pwm_light = pwm_light0

    
    
    # on ecrit la valeur que l'on veut dans le fichier test.txt   TODO: retirer ce passage pour version publique
    msg_lum = str(pwm_light0) 
    password = 'companion' 
    file3 = '/home/pi/pwm_light.txt'
    cmd = 'sshpass -p '+password+' ssh '+ adresse_ip0 +' "echo '+ msg_lum +' > '+file3+'"'
    os.system(cmd) 

    # for classic bluerov
    if (self.nb_channels > 8): # if the MAVROS version can control the light
        self.commands[8] = self.pwm_light


def clip(val, min_val, max_val):
    if val <= min_val:
        return min_val
    elif val >= max_val:
        return max_val
    return val


class ROV(Node):

    def __init__(self):
    
        super().__init__('Control_ROV')


        self.dt = 1/10
        
        self.declare_parameter('ROV_name', '')
        ROV_name = self.get_parameter('ROV_name').get_parameter_value().string_value
        self.declare_parameter('ID', 1.0)
        ID = self.get_parameter('ID').get_parameter_value().double_value
        self.declare_parameter('ip_adress_light', "192.168.2")
        ip_adress_light = self.get_parameter('ip_adress_light').get_parameter_value().string_value


        # identification
        self.ROV_name = ROV_name #rospy.get_param('~ROV_name',"inky")
        self.ID = str(ID)
        self.ID0 = ID
        self.IDnum = str(self.ID0+1)

        if not(self.ROV_name == ''):
            self.ns = "/"+ self.ROV_name # TODO
        else:
            self.ns = ""

        
        # pour lumiere (configuration maison)
        self.adresse_ip_short = ip_adress_light
        self.adresse_ip = 'pi@' + self.adresse_ip_short  +'.2'
        self.pwm_light = 1000.0
            

        # Robot modes
        self.armed = False
        self.program_B = False
        self.name_program_B = "(pas de programme)"

        # Robot parameter
        self.depth = 0.0 # Depth
        self.heading = 0.0 # Heading

        # IMU
        self.Phi_rad = 0.0
        self.Theta_rad = 0.0
        self.Psy_rad = 0.0
        self.angular_velocity_y = 0.0  # vitesse tangage
        self.angular_velocity_x = 0.0  # vitesse rouli
        

        # Buttons and Axes
        self.frame_id = 0
        self.frame_id_last = 0
        self.joy_time = 0.0
        self.joy_time_old = 0.0
        self.letter_to_indice = {"A": 0, "B": 1, "X": 2, "Y": 3, "LH": 4,
                                 "RH": 5, "Back": 6, "Start": 7, "?": 8, "L3": 9, "R3": 10}
        self.button_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # A, B, X, Y, LH , RH , back, start, ?, L3, R3
        self.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        

        # enregistrement
        self.start_record = False
        self.record = False
        self.record_data = False
        self.choix_record = 0.0 # 1: record all / 2: record datas without cams / 0 : sans choix encore 
        self.nb_choix_record = 2.0 
        self.record_step = 0.0 # etape de selection du mode d'enregistrement
          


        # ROS
        #### listener
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.queue_listener = 10
        self.listener()  # Subscribes to the messages

        ######################
        # liste des publishers
        self.command_pub = self.create_publisher(OverrideRCIn,self.ns+'/mavros/rc/override', self.queue_listener)
        self.program_B_name_pub = self.create_publisher(String,self.ns+"/program_B_name", self.queue_listener) 

        self.choice_record_pub = self.create_publisher(Float32MultiArray,self.ns+"/choice_record", self.queue_listener) 
        self.enregistrement_pub = self.create_publisher(Bool, self.ns+'/enregistrement_en_cours',  self.queue_listener)

        #####################

        self.nb_channels = 18 # rospy.get_param(rospy.get_name() + "/nb_channels", 18) # TODO: chercher pk ca ne marche pas...
        self.commands = [1500] * self.nb_channels
        if self.nb_channels > 8:     # if the MAVROS version can control the channels 8, i.e. the light
            self.commands[8] = 1000  # light pwm : 1000 = off, 2000 = max luminosity

        
        # MAVROS
        self.client_arm = self.create_client(CommandBool, self.ns+'/mavros/cmd/arming')
        self.send_request_arm(False)


        timer_period = self.dt # seconds
        self.timer = self.create_timer(timer_period, self.run)


    def button(self, letter):
        return self.button_values[self.letter_to_indice[letter]]


    def send_commands(self):

        msg = OverrideRCIn()
        for i in range(self.nb_channels):
            self.commands[i] = clip(self.commands[i], 1000, 2000)
            msg.channels[i] = self.commands[i] 
        #msg.channels = self.commands
        self.command_pub.publish(msg)

        msg = String()
        msg.data = self.name_program_B
        self.program_B_name_pub.publish(msg)
        
        msg1 = [self.record_step,self.choix_record]
        msg = Float32MultiArray(data = msg1)
        self.choice_record_pub.publish(msg)

        # booléen si on enregistre
        msg = Bool(data = self.start_record)
        self.enregistrement_pub.publish(msg)



    def callback_heading(self, msg):
        self.heading = msg.data

    def callback_heading_reference(self, data):
        self.heading_reference = data.data
        self.heading_global_target = self.heading_reference + self.heading_offset

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
        self.Phi_rad, self.Theta_rad, self.Psy_rad = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet 
        
                
    def callback_joy(self, msg):

        new_time = msg.header.stamp.sec +  msg.header.stamp.nanosec*10**(-9)

        if new_time - self.joy_time_old > 0.1:
            self.button_values = msg.buttons
            self.axes = msg.axes
            
            self.joy_time_old = self.joy_time
            self.joy_time = new_time

            self.frame_id = self.frame_id + 1

    def callback_depth(self, msg):
        self.depth = -msg.data

        
    def callback_ping1d(self, data):
        self.distance_ping = data.data[0]
        self.confidence_ping = data.data[1]
        self.ping = data.data[2]
                     
           
    def listener(self):

        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/compass_hdg',
            self.callback_heading,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.callback_joy,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/rel_alt',
            self.callback_depth,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning
      
        self.subscription = self.create_subscription(
            Imu,
            self.ns+"/mavros/imu/data",
            self.callback_imu,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning


        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.ns+'/msg_ping1d',
            self.callback_ping1d,
            self.queue_listener)
        self.subscription  # prevent unused variable warning



    def send_request_arm(self, arming):

        while not self.client_arm.wait_for_service(timeout_sec=0.5):
                    self.get_logger().info('service not available, waiting again...')
        req = CommandBool.Request()
        req.value = bool(arming)
        self.client_arm.call_async(req)


###############################
    def run(self):


        if (self.frame_id_last != self.frame_id):  # Check if new commands are given with joystick  : + on regarde si on obéit ou non à la télécommande
            self.frame_id_last = self.frame_id


            for i in range(8):
                self.commands[i] = 1500 # on remets les commandes a zero

            if (self.button("Start") == 1)|(self.button("Back") != 0):  # Arm and disarm
                
                if (self.button("Start") == 1):
                    self.armed = True 
                if self.button("Back") != 0:
                    self.armed = False # not self.armed
                response = self.send_request_arm(self.armed) # armement/desarmement

                self.commands = [1500] * self.nb_channels
                if self.nb_channels > 8:
                    self.commands[8] = 1000

            # camera inclination 
            if (self.axes[7] != 0):  # fleche haut/bas
                self.commands[7] = int(100 * self.axes[7] + 1500)
            else:
                self.commands[7] = 1500

            # light control 
            if self.button("?") != 0:  # button Back : control light intensity
                light_modif_value = 100
                light_control(self,light_modif_value)


            # activation d'un programme (au coder plus bas) avec le bouton "B"
            if (self.button('B') != 0):
                if (self.program_B == True):
                    self.program_B = False

                elif (self.program_B == False):
                    self.program_B = True

            ##### Lecture des input de la manette

            # Example : move forward/backward
            if self.axes[4] != 0:  # joy right up/down
                self.commands[4] = int(200 * self.axes[4] + 1500)
                self.commands_front = self.commands[4]

            else:
                self.commands[4] = 1500
                self.commands_front = self.commands[4]

            # (...)





        # Reste du code
        ##############################################
        
        # (...)

        ######## Programme a activer/desactive en appuyant sur le bouton "B" ############
        if self.program_B:

            self.name_program_B = "(votre nom de programme pour afficher dans l'IHM)"
            # ---> ce nom apparaitra en vert dans l'IHM quand "self.program_B = True" et en gris quand "self.program_B = Flase"

            # (...)

        ####################################################################
        # selection de l'enregistrement           
        #"""
        if (self.axes[6] != 0)&((self.record_step == 1)|(self.record_step == 2)):
                
            if self.axes[6] > 0.5:
                self.choix_record = 1
                print('record data+cam selected')
                print('Press "Y" again')
                self.record_step = 2 # etape suivante
            elif self.axes[6] < -0.5:
                self.choix_record = 2   
                print('record data without cam selected')
                print('Press "Y" again')
                self.record_step = 2 # etape suivante


        if (self.button("Y") != 0):  # button Y : record
            

            if self.record_step == 0: # si premiere demande

                print('Record : select one choice')
                print(' <- : record data+cam /-> : record data without cam') 
                self.record_step += 1 # demande du choix

            if self.record_step == -1: 
                self.record_step = 0

            # BBYY
            if (self.record_step > 1): # si choix fait OU si enregistrement deja en cours
                

                if (self.choix_record == 1)&(self.record_step == 2): # 0 : par defaut en cas de probleme
                    self.record = not self.record
                    
                    self.record_step = self.record_step+1

                    print('choix 1 fait')
                    time.sleep(0.5)


                elif (self.choix_record == 2)&(self.record_step == 2):
                    self.record_data = not self.record_data

                    self.record_step = self.record_step+1

                    print('choix 2 fait')

                    time.sleep(0.5)


                # si on a demande d'arreter, reset variable    
                elif (self.record_step > 3):

                    self.record_step = -1 # <- pk -1 ? car bug a la ... ou il relance immediatement l'enregistrement. Ca corrige en le faisant passer directement a 0 plutot qu a 1
                    self.choix_record = 0
                    print('Stop recording.')
                    time.sleep(0.5)

                else:
                    self.record_step += 1 # demande du choix

            
            print("self.record_step = ", self.record_step)

        self.function_rosbag()

        ##################################################
        self.send_commands()  # publisher general, pas que commande
        self.commands_old = self.commands


#### Pour enregistrement



    def function_rosbag(self):


        if (self.record_step > 2)&(self.start_record == False):
            
            if self.record_data == True:
                add_title = '_without_cam'
            else :
                add_title = ''
            
            # create directory to stock data
            ROOT_DIR = os.path.expanduser('~')
            self.filename0 =  ROOT_DIR + "/quick_start_bluerov_ros2/rosbag/" 

            current_date_and_time = datetime.datetime.now()
            current_date_and_time_string = str(current_date_and_time)[0:10] + '-' +  str(current_date_and_time)[11:13]+'-'+str(current_date_and_time)[14:16]+'-'+ str(current_date_and_time)[17:19]                 #[0:19] : keep only seconds
            path =  self.filename0 + self.ROV_name + add_title +'_' + current_date_and_time_string
            os.mkdir(path)
        
            # start rosbag
            print("Start recording...")
            if self.record_data == True:
                command = 'ros2 bag record -a -x "(.*)/cam_1/compressed|/(.*)/cam_2/compressed"' # enregistre tout sauf les cameras (pour gain de memoire)
                self.record = False # (qu'un seul enregistrement a la fois)
            else:
                command = "ros2 bag record -a"  # enregistre tout
                self.record_data = False  # (qu'un seul enregistrement a la fois)

            
            dir_save_bagfile = path
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)
            
            self.path = path # dossier principal d'enregistrement
            self.start_record = True

        elif (self.choix_record == 0)&(self.start_record == True):

            self.start_record = False
            # stop enregistrement rosbag
            self.terminate_process_and_children(self.p)   


########## pour rosbag, terminer l'enregistrement
# https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/


    def terminate_process_and_children(self,p):
              
        process = psutil.Process(p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        p.wait()  # we wait for children to terminate
        p.terminate()





def main(args=None):


    rclpy.init(args=args)


    node_rov = ROV()
    
    rclpy.spin(node_rov)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_rov.destroy_node()
    rclpy.shutdown()

          

if __name__ == '__main__':
    main()
    
    
    
