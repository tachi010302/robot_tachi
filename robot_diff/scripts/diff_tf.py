#!/usr/bin/env python3

import rospy
import tf
from math import sin, cos
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from tf.broadcaster import TransformBroadcaster

class CombinedTf:
    def __init__(self):
        # Inicialización de nodos de ROS
        rospy.init_node('combined_tf')
        self.nodename = rospy.get_name()
        rospy.loginfo("Nodo %s Iniciado" % self.nodename)

        # Parámetros para la odometría
        self.rate = rospy.get_param('~rate', 10.0)  # frecuencia de publicación de la odometría
        self.ticks_meter = float(rospy.get_param('ticks_meter', 6150))  # Ticks por metro
        self.base_width = float(rospy.get_param('~base_width', 0.25))  # Ancho de la base

        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')  # ID del marco base
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')  # ID del marco de odometría

        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # Datos internos para la odometría

        self.enc_left = None
        self.enc_right = None
        self.left = 0
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0
        self.then = rospy.Time.now()

        # Suscriptores

        rospy.Subscriber("lwheel", Int16, self.lwheelCallback)
        rospy.Subscriber("rwheel", Int16, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)

        # Publicadores de transformaciones

        self.odomBroadcaster = TransformBroadcaster()
        self.tfBroadcaster = tf.TransformBroadcaster()  # Para la cámara

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()  # Actualizar odometría
            self.publish_camera_transform()  # Publicar transformación de la cámara
            r.sleep()

    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            # Calcular odometría
            if self.enc_left is None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right

            # Distancia recorrida y cambio de ángulo
            d = (d_left + d_right) / 2
            th = (d_right - d_left) / self.base_width

            # Velocidades

            self.dx = d / elapsed
            self.dr = th / elapsed

            if d != 0:
                x = cos(th) * d
                y = -sin(th) * d
                self.x += cos(self.th) * x - sin(self.th) * y
                self.y += sin(self.th) * x + cos(self.th) * y
            if th != 0:
                self.th += th

            # Publicar odometría
            
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

    def publish_camera_transform(self):
        # Publicar la transformación de la cámara
        self.tfBroadcaster.sendTransform(
            (0.16, 0.0, 0.135),  # Posición de la cámara
            (0.0, 0.0, 0.0, 1.0),  # Orientación (sin rotación)
            rospy.Time.now(),
            "camera_link",
            "base_link"
        )

    def lwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult += 1
        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult -= 1
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult += 1
        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult -= 1
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

if __name__ == '__main__':
    try:
        combinedTf = CombinedTf()
        combinedTf.spin()
    except rospy.ROSInterruptException:
        pass
