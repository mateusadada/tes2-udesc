#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, sqrt, sin, cos, pi

class TurtleShapeController:
    def __init__(self):
        rospy.init_node('turtle_shape_controller')

        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.subs = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Kp
        self.Kp_linear = 1.5
        self.Kp_angular = 4.0

        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.1

        self.pose = None
        self.targets = []

        rospy.loginfo("Controlador de formas inicializado.")


    def pose_callback(self, msg):
        self.pose = msg

        if not self.targets:
            return

        target_x, target_y = self.targets[0]

        error_x = target_x - msg.x
        error_y = target_y - msg.y
        distance = sqrt(error_x**2 + error_y**2)

        # reached the vertex, go to the next one
        if distance < self.distance_tolerance:
            rospy.loginfo(f"Vertice alcancado: x:{target_x:.2f}, y:{target_y:.2f}")
            self.targets.pop(0)
            self.cmd_vel_pub.publish(Twist())
            return

        target_theta_z = atan2(error_y, error_x)
        error_theta_z = target_theta_z - msg.theta
        error_theta_z = atan2(sin(error_theta_z), cos(error_theta_z))

        cmd = Twist()
        cmd.angular.z = self.Kp_angular * error_theta_z

        # only moves forward when facing the vertex, keeps the corners sharp
        if abs(error_theta_z) < self.angle_tolerance:
            cmd.linear.x = min(self.Kp_linear * distance, 2.0)

        self.cmd_vel_pub.publish(cmd)


    def build_shape(self, shape):
        x = self.pose.x
        y = self.pose.y
        side = 3.0

        # triangle
        if shape == '1':
            height = side * sqrt(3) / 2
            return [
                (x + side, y),
                (x + side / 2, y + height),
                (x, y),
            ]

        # square
        if shape == '2':
            return [
                (x + side, y),
                (x + side, y + side),
                (x, y + side),
                (x, y),
            ]

        # star: 5 points on a circle, jumping one point each time
        if shape == '3':
            radius = 2.0
            center_x = x
            center_y = y - radius
            points = []

            for i in [2, 4, 1, 3, 0]:
                angle = pi / 2 + i * 2 * pi / 5
                point_x = center_x + radius * cos(angle)
                point_y = center_y + radius * sin(angle)
                points.append((point_x, point_y))

            return points

        return []


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        # wait for the first pose
        while self.pose is None and not rospy.is_shutdown():
            rate.sleep()

        while not rospy.is_shutdown():
            print('\n1 - Triangulo\n2 - Quadrado\n3 - Estrela\n0 - Sair')
            shape = input('Escolha uma forma: ')

            if shape == '0':
                break

            targets = self.build_shape(shape)
            if not targets:
                print('Opcao invalida.')
                continue

            self.targets = targets
            while self.targets and not rospy.is_shutdown():
                rate.sleep()

            print('Forma concluida!')

if __name__ == '__main__':
    try:
        controller = TurtleShapeController()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
