#!/usr/bin/env python

import rospy
import math
import os
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from tf.transformations import euler_from_quaternion

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        # Parâmetros
        self.waypoint_radius = rospy.get_param('~waypoint_radius', 2.0)
        self.waypoints_file = rospy.get_param('~waypoints_file', 'waypoints.txt')
        self.control_rate = rospy.get_param('~control_rate', 20)
        self.max_velocity = rospy.get_param('~max_velocity', 2.0)

        # Estado
        self.current_position = None
        self.reference_position = None
        self.current_heading = None
        self.current_waypoint_index = 0
        self.waypoints = []

        # Publicadores
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.debug_pub = rospy.Publisher('/waypoint_follower/debug', Float64, queue_size=10)
        self.debug_string_pub = rospy.Publisher('/waypoint_follower/status', String, queue_size=10)

        # Subscritores
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)

        self.load_waypoints()

        rospy.loginfo("Navegação por waypoints iniciada")
        rospy.loginfo(f"Raio de tolerância: {self.waypoint_radius}m")
        rospy.loginfo(f"Velocidade máxima: {self.max_velocity}m/s")

        self.run()

    def position_callback(self, msg):
        position = (msg.latitude, msg.longitude, 0.0)

        if self.reference_position is None:
            self.reference_position = position
            rospy.loginfo(f"Posição de referência definida: {position}")

        self.current_position = position

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.current_heading = yaw

    def load_waypoints(self):
        """Carregar waypoints do arquivo"""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        waypoints_path = os.path.join(script_dir, self.waypoints_file)
        rospy.loginfo(f"Carregando waypoints de: {waypoints_path}")
        
        try:
            with open(waypoints_path, 'r') as file:
                for line in file:
                    if line.strip() and not line.strip().startswith('#'):
                        parts = line.strip().split(',')
                        if len(parts) >= 2:
                            first_val = float(parts[0])
                            second_val = float(parts[1])
                            
                            if first_val < 0 and (second_val > 30 and second_val < 60):
                                lat, lon = second_val, first_val
                                rospy.logwarn(f"Ordem invertida corrigida: {first_val},{second_val} -> {lat},{lon}")
                            else:
                                lat, lon = first_val, second_val
                                
                            self.waypoints.append((lat, lon, 0.0))
                            rospy.loginfo(f"Waypoint carregado -> Lat: {lat:.7f}, Lon: {lon:.7f}")
        
            if not self.waypoints:
                rospy.logerr("Nenhum waypoint válido encontrado!")
        except Exception as e:
            rospy.logerr(f"Erro ao carregar waypoints: {e}")

    def simple_relative_position(self, ref_pos, target_pos):
        ref_lat, ref_lon, _ = ref_pos
        target_lat, target_lon, _ = target_pos

        METERS_PER_LAT = 111320.0
        METERS_PER_LON = 111320.0 * math.cos(math.radians(ref_lat))

        lat_diff = (target_lat - ref_lat) * METERS_PER_LAT
        lon_diff = (target_lon - ref_lon) * METERS_PER_LON

        return lon_diff, lat_diff, 0.0  # ENU

    def send_velocity_command(self, vx, vy, yaw_rate=0.0):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = yaw_rate

        self.velocity_pub.publish(msg)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.debug_string_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown() and self.current_position is None:
            rospy.loginfo_throttle(1.0, "Aguardando posição GPS...")
            rate.sleep()

        rospy.loginfo("GPS disponível. Iniciando navegação.")
        last_status_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("Todos os waypoints foram alcançados.")
                self.send_velocity_command(0.0, 0.0)
                self.publish_status("Missão completa")
                break

            target = self.waypoints[self.current_waypoint_index]
            east, north, _ = self.simple_relative_position(self.current_position, target)
            distance = math.hypot(east, north)

            if (rospy.Time.now() - last_status_time).to_sec() >= 1.0:
                status = f"WP{self.current_waypoint_index+1}: E={east:.1f}m N={north:.1f}m dist={distance:.1f}m"
                self.publish_status(status)
                rospy.loginfo(status)
                last_status_time = rospy.Time.now()

            if distance < self.waypoint_radius:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index + 1} alcançado.")
                self.current_waypoint_index += 1
                for _ in range(self.control_rate):  # Pausa de 1 segundo
                    self.send_velocity_command(0.0, 0.0)
                    rate.sleep()
                continue

            vx = (north / distance) * min(self.max_velocity, distance * 0.5)
            vy = (east / distance) * min(self.max_velocity, distance * 0.5)

            # Cálculo simples de yaw rate
            desired_yaw = math.atan2(east, north)
            yaw_rate = 0.0
            if self.current_heading is not None:
                yaw_error = desired_yaw - self.current_heading
                while yaw_error > math.pi:
                    yaw_error -= 2 * math.pi
                while yaw_error < -math.pi:
                    yaw_error += 2 * math.pi
                yaw_rate = max(-1.0, min(1.0, yaw_error * 0.5))

            self.send_velocity_command(vx, vy, yaw_rate)
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointFollower()
    except rospy.ROSInterruptException:
        pass

