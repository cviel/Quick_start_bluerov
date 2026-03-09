

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64  

from sensor_msgs.msg import Joy

import time


class ROV_switch(Node):

    def __init__(self):
    
        super().__init__('ROV_switch')


        #self.ID_max = "1" #  rospy.get_param('~nb_ROV',"1")
        self.declare_parameter('nb_ROV', 1.0)
        self.ID_max = self.get_parameter('nb_ROV').get_parameter_value().double_value


        self.msg = 1
        self.button = [0,0,0,0,0,0,0,0,0,0,0]  # A, B,X, Y , LH , RH , back, start, ?, L3, R3
        self.Jaxes = [0,0,0,0,0,0,0,0]   # (gauche gauche(1)/droite(-1), gauche haut(1)/bas(-1), ? , droite gauche(1)/droite(-1), droite haut(1)/bas(-1), ?, flèches gauche(1)/droite(-1), flèches haut(1)/bas(-1))
        #channel = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.frame_id = 0.0
        self.frame_id_old = 0.0
        self.joy_time = 0.0
        self.joy_time_old = 0.0


        # publisher
        self.publisher_switch = self.create_publisher( Float64, '/num_joy_control', 10)

        self.listener()

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.run)




    def callback_joy(self, msg):

        new_time = msg.header.stamp.sec +  msg.header.stamp.nanosec*10**(-9)

        if new_time - self.joy_time_old > 0.3:
            self.button = msg.buttons
            self.Jaxes = msg.axes
                    
            self.joy_time_old = self.joy_time
            self.joy_time = new_time

            #self.frame_id = msg.header.seq
            # TODO : Corriger ca...
            self.frame_id = self.frame_id + 1 # paliatif grossier... TODO

            
    def listener(self):

        #rospy.Subscriber("/joy", Joy, self.callback_joy)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.callback_joy,
            10)
        self.subscription  # prevent unused variable warning

    
    def run(self):
        
        test_input = 0
        if (self.frame_id_old != self.frame_id): 
        
            self.frame_id_old = self.frame_id
            
            if (self.button[10] == 1):  # si on enfonce le joystick droit, on augmente de +1
                self.msg = self.msg + 1.0
                if self.msg > self.ID_max: # si on dépasse le nombre de group, on revient à 1
                    self.msg = 1.0
                test_input = 1
            if (self.button[9] == 1):  # si on enfonce le joystick droit, on augmente de -1
                self.msg = self.msg - 1.0
                if self.msg < 1: # si on dépasse le nombre de group, on revient à 1
                    self.msg = self.ID_max
                test_input = 2

        msg0 = Float64(data = float(self.msg))
        self.publisher_switch.publish(msg0)
        if (test_input)>0 :
            if test_input == 1:
                print("change ROV : + 1")
            else:
                print("change ROV : - 1")
            time.sleep(0.5) # si il y a eu une input, on attend une seconde afin d'eviter d'enregistrer plusieurs pression de bouton a la suite
            

        # remettre à zero les inputs des boutons
        self.button = [0,0,0,0,0,0,0,0,0,0,0]
        self.Jaxes = [0,0,0,0,0,0,0,0]


            
            
def main(args=None):


    rclpy.init(args=args)


    node_ROV_switch = ROV_switch()
    
    rclpy.spin(node_ROV_switch)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_ROV_switch.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
    
    
    


