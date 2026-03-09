import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from sensor_msgs.msg import Imu, MagneticField, BatteryState
from std_msgs.msg import Float64, String
from mavros_msgs.msg import State, RCIn, OverrideRCIn, ActuatorControl, ManualControl, ManualControl, ParamEvent, VfrHud
from geometry_msgs.msg import Vector3
from mavros_msgs.srv import CommandBool
from rclpy.qos import QoSProfile

import threading
import time
import math
import numpy as np

from scipy.spatial.transform import Rotation as R



def clip(val, min_val, max_val):
    if val <= min_val:
        return min_val
    elif val >= max_val:
        return max_val
    return val




class MiniMavros(Node):

    MILLIG_TO_MS2 = 9.80665 / 1000.0  # comme dans imu.cpp :contentReference[oaicite:0]{index=0}  
    MILLIMS2_TO_MS2 = 1.0e-3  # conversion des milli-m/s² :contentReference[oaicite:1]{index=1}  
    MILLIRADS_TO_RAD = 1.0e-3  # millirad/s → rad/s :contentReference[oaicite:2]{index=2}  
    GAUSS_TO_TESLA = 1e-4  # Gauss → Tesla :contentReference[oaicite:3]{index=3}  


    def __init__(self):
        super().__init__("mini_mavros")


        ######################################
        # Ouverture connexion MAVLink BlueROV

        self.declare_parameter('adresse_udp', "udp:192.168.2.1:14550")
        #self.declare_parameter('adresse_udp', "udp:0.0.0.0:14550")
        self.adresse_udp = self.get_parameter('adresse_udp').get_parameter_value().string_value
        self.declare_parameter('system_id', '1')
        self.system_id = int(self.get_parameter('system_id').get_parameter_value().string_value)
        self.declare_parameter('tgt_system', '1')
        self.target_system = int(self.get_parameter('tgt_system').get_parameter_value().string_value)

        """
        # pour castor:
        self.system_id = 11
        self.target_system = 21 # SYSID_THISMAV dans ton BlueROV
        """
        self.target_component = 1  # DOIS TOUJOURS ETRE A 1

        #initializing mavlink master
        self.master = mavutil.mavlink_connection(
            #"udp:0.0.0.0:14550",     
            #"udp:192.168.2.1:14550", 
            #'tcp:192.168.21.2:14550',  
            self.adresse_udp,     
            source_system=self.system_id, # 11
            source_component=self.target_component) # 1
        self.master.wait_heartbeat()
     

        # Passage en mode MANUAL
        self.master.set_mode_manual() 

        self.get_logger().info("Connexion MAVLink établie")
        print("target_system : ", self.master.target_system, " / target_component : ", self.master.target_component)
        print("self.target_component = ", self.target_component)
        print("self.adresse_udp = ", self.adresse_udp)

        #######################################
        print("Desarmement par securite...")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )


        self.master.motors_disarmed_wait()
        print("ROV UNARMED")
        self.armed = False

        # envoie d'une commande pour mettre les moteurs à l arret 
        rc = [1500] * 18   # 8 canaux
        rc[8] = 1000 # la lumière

        self.command_old = rc

        self.master.mav.rc_channels_override_send(
            self.target_system,
            self.target_component,
            *rc
        )

        self.rc = rc
        ########################################
        # variable

        # Covariances (3x3) : initialisation comme dans `setup_covariance()` de imu.cpp :contentReference[oaicite:4]{index=4}
        self.cov_ang = np.zeros((3,3))
        self.cov_acc = np.zeros((3,3))
        self.cov_ori = np.zeros((3,3))
        self._setup_covariance(self.cov_ang, 0.02)   # std ~ 0.02 rad/s
        self._setup_covariance(self.cov_acc, 0.1)    # std ~ 0.1 m/s²

        self.imu_pub = self.create_publisher(Imu, 'mavros/imu/data', 10)
        self.imu_raw_pub = self.create_publisher(Imu, 'mavros/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'mavros/imu/mag', 10)

        ############
        # Publishers
        self.pub_state = self.create_publisher(State, "mavros/state", 10)
        self.pub_battery = self.create_publisher(BatteryState, "mavros/battery", 10)
        self.pub_rel_alt = self.create_publisher(Float64, "mavros/global_position/rel_alt", 10)
        self.pub_heading = self.create_publisher(Float64, "mavros/global_position/compass_hdg", 10)
        self.pub_rc_in = self.create_publisher(RCIn, "mavros/rc/in", 10)
        self.pub_sys_status = self.create_publisher(String, "mavros/sys_status", 10)
        self.pub_ext_state = self.create_publisher(String, "mavros/extended_state", 10)

        self.pub_statustext = self.create_publisher(String, "mavros/statustext/recv", 10)
        self.pub_home_position = self.create_publisher(String, "mavros/home_position/home", 10)  # simplifié
        self.pub_diff_pressure = self.create_publisher(Float64, "mavros/imu/diff_pressure", 10)
        self.pub_temperature_baro = self.create_publisher(Float64, "mavros/imu/temperature_baro", 10)
        self.pub_vfr_hud = self.create_publisher(VfrHud, "mavros/vfr_hud", 10)


        ######## service ###############
        self.srv = self.create_service(CommandBool, 'mavros/cmd/arming', self.Arm_ROV)


        ###### publisher ############
        self.queue_listener = 10
        self.command_pub = self.create_publisher(OverrideRCIn,'mavros/rc/override', self.queue_listener)
        msg = OverrideRCIn()

        msg.channels = [int(1500)] * 18
        self.command_pub.publish(msg)  # on envoie 1 message, juste pour creer le topic. ce ne sera pas ce code qui les enverra, mais plutôt les ecoutera

        ########################

        #### listener
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.queue_listener = 10
        self.listener()  # Subscribes to the messages

        #############################

        # Thread MAVLink -> ROS2
        self.thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.thread.start()

        timer_period = 0.1/2  # seconds
        self.timer = self.create_timer(timer_period, self.run)

        self.temps = time.time()


    def listener(self):

        self.subscription = self.create_subscription(
            OverrideRCIn,
            'mavros/rc/override',
            self.callback_override,
            self.qos_profile)
        self.subscription  # prevent unused variable warning

    def callback_override(self, msg):
        self.rc = msg.channels


    def run(self):  # (pour le test)

        #print("mini MAVROS pour bluerov: on")
        #print("self.armed = ", self.armed)

        #print("rc = ", self.rc)
        self.rc_override(self.rc)
        
        

        ### quelques fonctions de test
        """
        command = [1500] * 8
        command[1] = 1500

        #if not(self.command_old == command):
        self.rc_override(command)
        #"""

        """
        if self.armed == False:
            #self.Disarm()
            self.Arm()
        #"""
        """
        if self.armed == True:
            self.Disarm()
        #"""
        
        """
        if time.time()> self.temps + 2:
            if self.armed == False:

                self.Arm()
            elif self.armed == True:
                self.Disarm()
            self.temps = time.time()
        """

    def Arm_ROV(self, request, response): # service pour les autres codes pour l'armement/desarmement du ROV

        time_out = time.time()
        response.success = False

        while (time.time() < time_out+1)&(response.success == False): # on tente plusieurs fois avant d'abandonner
            if (request.value == True):
                self.Arm()
            elif (request.value == False):
                self.Disarm()
            if request.value == self.armed:
                response.success = True
            else:
                response.success = False
            
        response.result = self.armed
        print("success : ", response.success, "ROV armed :", self.armed)
        return response
    

    def Arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        #print("Arming...")
        ##self.master.motors_armed_wait()
        #print("ROV ARMED")
        ##self.armed = True

    def Disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        #print("Disarming...")
        ##self.master.motors_armed_wait()
        #print("ROV DISARMED")
        ##self.armed = False
    
    def rc_override(self, channels):

        rc = [1500] * 18  # [1500] * 18   # 18 canaux
        rc[8] = 1000

        for i in range(len(channels)):
            rc[i] = clip(channels[i], 1000, 2000)
            #rc[i] = channels[i]

        self.master.mav.rc_channels_override_send(
            self.target_system , # self.master.target_system,
            self.target_component, # self.master.target_component,
            *rc
        )
        self.command_old = channels

    ############# pour le calcul des IMUs ###############################################
    def _setup_covariance(self, cov, stdev):
        var = stdev * stdev
        for i in range(3):
            cov[i, i] = var

    def poll(self):
        msg = self.master.recv_match(blocking=False)
        if msg is None:
            return

    def handle_scaled_imu(self, msg):
        # Conversion comme dans imu.cpp handle_scaled_imu :contentReference[oaicite:5]{index=5}
        # Accélération: xacc, yacc, zacc en milli-g → m/s²
        accel = np.array([msg.xacc, msg.yacc, msg.zacc]) * self.MILLIG_TO_MS2
        # Gyro: xgyro, ygyro, zgyro en milli-rad/s → rad/s
        gyro = np.array([msg.xgyro, msg.ygyro, msg.zgyro]) * self.MILLIRADS_TO_RAD
        # Champ magnétique: xmag, ymag, zmag en “milli-gauss” (?) converti en Tesla
        mag = np.array([msg.xmag, msg.ymag, msg.zmag]) * self.GAUSS_TO_TESLA

        # Calcul orientation approximative (roll, pitch), yaw = 0 (comme fallback)
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        yaw = 0.0
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        # Header
        t = self.get_clock().now().to_msg()

        # Publisher champ magnétique
        mag_msg = MagneticField()
        mag_msg.header.stamp = t
        mag_msg.header.frame_id = "base_link"
        mag_msg.magnetic_field.x = mag[0]
        mag_msg.magnetic_field.y = mag[1]
        mag_msg.magnetic_field.z = mag[2]
        self.mag_pub.publish(mag_msg)

        # Message IMU
        imu_msg = Imu()
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # Covariances
        imu_msg.orientation_covariance = [float(self.cov_ori[i,j]) for i in range(3) for j in range(3)]
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.angular_velocity_covariance = [float(self.cov_ang[i,j]) for i in range(3) for j in range(3)]
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance = [float(self.cov_acc[i,j]) for i in range(3) for j in range(3)]

        self.imu_pub.publish(imu_msg)

        # Message IMU “raw” 
        raw = Imu()
        raw.header = imu_msg.header
        raw.orientation = imu_msg.orientation
        raw.orientation_covariance = imu_msg.orientation_covariance
        raw.angular_velocity = imu_msg.angular_velocity
        raw.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        raw.linear_acceleration = imu_msg.linear_acceleration
        raw.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        self.imu_raw_pub.publish(raw)

    def handle_highres_imu(self, msg):
        # HIGHRES_IMU :
        accel = np.array([msg.xacc, msg.yacc, msg.zacc])  # m/s²
        gyro = np.array([msg.xgyro, msg.ygyro, msg.zgyro])  # rad/s
        mag = np.array([msg.xmag, msg.ymag, msg.zmag]) * self.GAUSS_TO_TESLA

        # Orientation fallback comme scaled imu
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        yaw = 0.0
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        t = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.angular_velocity_covariance = [float(self.cov_ang[i,j]) for i in range(3) for j in range(3)]

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance = [float(self.cov_acc[i,j]) for i in range(3) for j in range(3)]

        imu_msg.orientation_covariance = [float(self.cov_ori[i,j]) for i in range(3) for j in range(3)]
        self.imu_pub.publish(imu_msg)

    def handle_attitude(self, msg):
        # ATTITUDE : contient roll, pitch, yaw (rad/s), transform de NED vers ENU, etc. :contentReference[oaicite:8]{index=8}
        # msg.roll, msg.pitch, msg.yaw : en rad
        # msg.rollspeed, pitchspeed, yawspeed : rad/s
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        # Conversion des angles pour passer de NED → ENU, puis vers base_link (frame) selon imu.cpp :contentReference[oaicite:9]{index=9}
        # Ici on effectue un “transform_orientation_ned_enu” implicite : NED (FCU) → ENU
        # Puis un “transform_orientation_aircraft_baselink” = identité si on considère base_link comme frame d’IMU
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        t = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        imu_msg.orientation_covariance = [1.0, 0.0, 0.0,
                                          0.0, 1.0, 0.0,
                                          0.0, 0.0, 1.0]

        imu_msg.angular_velocity.x = msg.rollspeed
        imu_msg.angular_velocity.y = msg.pitchspeed
        imu_msg.angular_velocity.z = msg.yawspeed
        imu_msg.angular_velocity_covariance = [float(self.cov_ang[i,j]) for i in range(3) for j in range(3)]

        imu_msg.linear_acceleration_covariance = [float(self.cov_acc[i,j]) for i in range(3) for j in range(3)]
        # On ne publie pas les accélérations depuis ATTITUDE
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        self.imu_pub.publish(imu_msg)


    ############################################################
    # ----------------------------------------------------------------------
    # LECTURE MAVLINK EN CONTINU
    # ----------------------------------------------------------------------
    def mavlink_loop(self):
        while rclpy.ok():



            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue


            if msg.get_srcSystem() != self.target_system or msg.get_srcComponent() not in [1, 250]:
                continue


            msg_type = msg.get_type()


            # ================
            # STATE
            # ================
            if msg_type == "HEARTBEAT":
                rosmsg = State()

                rosmsg.header.stamp = self.get_clock().now().to_msg()

                # Armed/désarmé
                rosmsg.armed = (msg.base_mode &
                                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self.armed = rosmsg.armed 
                # Mode (custom)
                rosmsg.mode = str(msg.custom_mode)

                # system_status = uint8 → OK
                rosmsg.system_status = int(msg.system_status)

                # Champs MAVROS que nous simulons :
                rosmsg.connected = True
                rosmsg.guided = (msg.base_mode &
                                mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0
                rosmsg.manual_input = True

                # Selon ROS2, pas de mode_id → ne pas l’envoyer

                self.pub_state.publish(rosmsg)

            # ================
            # BATTERY STATUS
            # ================
            elif msg_type == "BATTERY_STATUS":
                rosmsg = BatteryState()
                rosmsg.voltage = msg.voltages[0] / 1000.0
                rosmsg.percentage = msg.battery_remaining / 100.0
                self.pub_battery.publish(rosmsg)

            # ================
            # IMU RAW
            # ================

            elif msg_type == 'SCALED_IMU':
                self.handle_scaled_imu(msg)
            elif msg_type == 'HIGHRES_IMU':
                self.handle_highres_imu(msg)
            elif msg_type == 'ATTITUDE':
                self.handle_attitude(msg)


            # ================
            # RC INPUT
            # ================
            elif msg_type == "RC_CHANNELS":
                rosmsg = RCIn()
                rosmsg.channels = [
                        msg.chan1_raw,
                        msg.chan2_raw,
                        msg.chan3_raw,
                        msg.chan4_raw,
                        msg.chan5_raw,
                        msg.chan6_raw,
                        msg.chan7_raw,
                        msg.chan8_raw
                    ]
                self.pub_rc_in.publish(rosmsg)

            # ================
            # ALTITUDE
            # ================
            elif msg_type == "GLOBAL_POSITION_INT":
                alt = msg.relative_alt / 1000.0
                self.pub_rel_alt.publish(Float64(data=alt))
                self.pub_heading.publish(Float64(data=msg.hdg / 100.0))

            # ================
            # SYSTEM STATUS
            # ================
            elif msg_type == "SYS_STATUS":

                self.pub_sys_status.publish(String(data=str(msg)))

            # ================
            # EXTENDED STATE
            # ================
            elif msg_type == "EXTENDED_SYS_STATE":
                self.pub_ext_state.publish(String(data=str(msg)))

            elif msg_type == "STATUSTEXT":
                st = String()
                st.data = msg.text
                self.pub_statustext.publish(st)

            elif msg_type == "HOME_POSITION":
                hp = String()
                hp.data = str(msg)  # pour simplifier, tu peux créer un message spécifique si besoin
                self.pub_home_position.publish(hp)

            elif msg_type == "SCALED_PRESSURE":
                diff_pressure = Float64(data=float(msg.press_diff))
                static_pressure = Float64(data=float(msg.press_abs))
                temperature_baro = Float64(data=float(msg.temperature))  # CAST EN FLOAT !

                self.pub_diff_pressure.publish(diff_pressure)
                self.pub_temperature_baro.publish(temperature_baro)

            elif msg_type == "VFR_HUD":
                rosmsg = VfrHud()
                rosmsg.airspeed = float(msg.airspeed)        # m/s
                rosmsg.groundspeed = float(msg.groundspeed)  # m/s
                rosmsg.heading = int(msg.heading)          # deg
                rosmsg.throttle = float(msg.throttle)        # 0..100
                rosmsg.altitude = float(msg.alt)             # m
                rosmsg.climb = float(msg.climb)              # m/s
                self.pub_vfr_hud.publish(rosmsg)



# ----------------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = MiniMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
