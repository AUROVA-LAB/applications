#!/usr/bin/env python

import rospy
import os
import csv
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf.transformations as tf_trans

def euler_to_quaternion(yaw):
    """
    Convierte un ángulo Euler (yaw) en un cuaternión.
    """
    quat = tf_trans.quaternion_from_euler(0, 0, yaw)
    return Quaternion(*quat)

def read_pose2d_files(folder_path, N):
    """
    Lee los archivos i_pose2d.csv y devuelve una lista de poses.
    """
    poses = []
    for i in range(N):
        file_name = f"{i}_pose2d.csv"
        file_path = os.path.join(folder_path, file_name)
        if not os.path.isfile(file_path):
            rospy.logwarn(f"El archivo {file_name} no existe en la ruta especificada.")
            continue
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) != 4:
                    rospy.logwarn(f"El archivo {file_name} tiene un formato incorrecto.")
                    continue
                try:
                    x = float(row[1])
                    y = float(row[2])
                    theta = float(row[3])
                    poses.append((x, y, theta))
                except ValueError:
                    rospy.logwarn(f"Los datos en {file_name} no pueden ser convertidos a flotantes.")
    return poses

def publish_poses(poses):
    """
    Publica las poses como mensajes de Odometry en ROS.
    """
    pub = rospy.Publisher('/ground_truth', Odometry, queue_size=10)
    rospy.init_node('pose2d_publisher', anonymous=True)
    rate = rospy.Rate(50)  # Publica a 1 Hz

    odom_msg = Odometry()
    odom_msg.header.frame_id = "map"
    odom_msg.child_frame_id = ""

    for idx, (x, y, theta) in enumerate(poses):
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0  # Asumimos movimiento en 2D
        odom_msg.pose.pose.orientation = euler_to_quaternion(theta)

        pub.publish(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Ruta de la carpeta que contiene los archivos
        if len(sys.argv)!=2:
            print("Usage: python publish_ground_truth.py FOLDER")
            exit()
        folder_path = sys.argv[1]
        # Número de archivos
        N = len(os.listdir(folder_path))  # Reemplaza con el número real de archivos

        poses = read_pose2d_files(folder_path, N)
        if poses:
            publish_poses(poses)
        else:
            rospy.logwarn("No se encontraron poses para publicar.")
    except rospy.ROSInterruptException:
        pass
