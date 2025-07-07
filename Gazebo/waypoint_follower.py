#!/usr/bin/env python
import os
import rospy
from math import atan2, sqrt, pi, cos, sin
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class Movement:
    """
    Controla o robô para seguir uma sequência de waypoints definidos num ficheiro.
    """
    def __init__(self):
        # Estado do robô
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.initialized = False
        self.origin = None

        # Inicializa o nó ROS
        rospy.init_node("husky_relative_waypoints")

        # Publisher e Subscriber
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.newOdom)

        # Mensagem de comando e taxa
        self.twist = Twist()
        self.r = rospy.Rate(10)

        # Caminho para o ficheiro de waypoints (pode vir por parâmetro)
        default_wp = "/home/afonso/catkin_ws/src/scripts/src/waypoints.txt"
        self.waypoints_file = rospy.get_param("~waypoints_file", default_wp)

        # Carrega a lista de waypoints relativos
        self.relative_waypoints = self.load_waypoints(self.waypoints_file)

        # Aguarda 1s para garantir receção de odometria
        rospy.sleep(1)

        # Inicia o loop de controlo
        self.move()

    def load_waypoints(self, file_path):
        """
        Lê um ficheiro texto com linhas 'x y' e devolve lista de tuplos (float, float).
        Linhas vazias ou começadas por '#' são ignoradas.
        """
        waypoints = []
        if not os.path.isfile(file_path):
            rospy.logerr(f"[Waypoints] Ficheiro não encontrado: {file_path}")
            return waypoints

        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) < 2:
                    rospy.logwarn(f"[Waypoints] Linha inválida (menos de 2 valores): '{line}'")
                    continue
                try:
                    x, y = float(parts[0]), float(parts[1])
                    waypoints.append((x, y))
                except ValueError:
                    rospy.logwarn(f"[Waypoints] Impossível converter: '{line}'")
        rospy.loginfo(f"[Waypoints] {len(waypoints)} waypoints carregados de '{file_path}'")
        return waypoints

    def newOdom(self, msg):
        """
        Callback para /odom: atualiza posição e orientação.
        Guarda a posição inicial na primeira mensagem.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.initialized:
            self.origin = (self.x, self.y, self.theta)
            self.initialized = True
            rospy.loginfo(f"[Odom] Origem capturada: x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}")

    def normalize_angle(self, angle):
        """Normaliza ângulo para [-pi, pi]."""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def transform_relative_to_global(self, rel_x, rel_y):
        """
        Transforma coordenadas relativas (ao ponto de partida)
        para o referencial de odometria global.
        """
        ox, oy, otheta = self.origin
        gx = ox + rel_x * cos(otheta) - rel_y * sin(otheta)
        gy = oy + rel_x * sin(otheta) + rel_y * cos(otheta)
        return gx, gy

    def move(self):
        """
        Loop principal: percorre cada waypoint e aplica controlo proporcional
        para orientar e mover o robô até lá.
        """
        # Espera pela primeira leitura de odometria
        while not self.initialized and not rospy.is_shutdown():
            self.r.sleep()

        if not self.relative_waypoints:
            rospy.logerr("[Move] Sem waypoints para navegar. Encerrando.")
            return

        rospy.loginfo("[Move] Iniciando navegação pelos waypoints...")

        for idx, (rel_x, rel_y) in enumerate(self.relative_waypoints, start=1):
            goal_x, goal_y = self.transform_relative_to_global(rel_x, rel_y)
            rospy.loginfo(f"[Move] Waypoint {idx}/{len(self.relative_waypoints)}: "
                          f"Rel({rel_x:.2f},{rel_y:.2f}) → Glob({goal_x:.2f},{goal_y:.2f})")

            distance = sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)
            # Enquanto não chegar perto do waypoint
            while distance >= 0.3 and not rospy.is_shutdown():
                dx = goal_x - self.x
                dy = goal_y - self.y
                distance = sqrt(dx*dx + dy*dy)
                target_angle = atan2(dy, dx)
                angle_diff = self.normalize_angle(target_angle - self.theta)

                # Se estiver muito desenquadrado, roda primeiro
                if abs(angle_diff) > 0.1:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.8 * angle_diff
                else:
                    # Avança proporcional à distância (até 0.4 m/s)
                    self.twist.linear.x = min(0.4, distance)
                    self.twist.angular.z = 0.0

                self.publisher.publish(self.twist)
                self.r.sleep()

            rospy.loginfo(f"[Move] Waypoint {idx} atingido!")

        # Pára o robô
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)
        rospy.loginfo("[Move] Todos os waypoints foram alcançados.")

if __name__ == "__main__":
    try:
        Movement()
    except rospy.ROSInterruptException:
        pass

