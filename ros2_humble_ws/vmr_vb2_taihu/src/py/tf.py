#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf_transformations import quaternion_matrix, translation_matrix, concatenate_matrices

class TFMatrixNode(Node):
    def __init__(self):
        super().__init__('tf_matrix_node')
        
        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建一个定时器，定期获取变换矩阵
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        try:
            # 获取从base_link到camera_link的变换
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',  # 目标坐标系
                'camera_link',  # 源坐标系
                rclpy.time.Time()  # 获取最新可用的变换
            )
            
            # 提取平移和旋转信息
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 构建变换矩阵
            # 方法1: 使用tf_transformations库
            T = self.transform_to_matrix(translation, rotation)
            
            # 打印变换矩阵
            self.get_logger().info("Transformation matrix from camera_link to base_link:")
            for i in range(4):
                self.get_logger().info("[%.4f, %.4f, %.4f, %.4f]" % 
                                      (T[i, 0], T[i, 1], T[i, 2], T[i, 3]))
            
            # 提取旋转矩阵和平移向量
            rotation_matrix = T[:3, :3]
            translation_vector = T[:3, 3]
            
            self.get_logger().info("Rotation matrix:")
            for i in range(3):
                self.get_logger().info("[%.4f, %.4f, %.4f]" % 
                                      (rotation_matrix[i, 0], rotation_matrix[i, 1], rotation_matrix[i, 2]))
            
            self.get_logger().info("Translation vector: [%.4f, %.4f, %.4f]" % 
                                  (translation_vector[0], translation_vector[1], translation_vector[2]))
            
        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform: {ex}")

    def transform_to_matrix(self, translation, rotation):
        """
        将平移和四元数转换为4x4齐次变换矩阵
        """
        # 创建平移矩阵
        trans_matrix = translation_matrix([translation.x, translation.y, translation.z])
        
        # 创建旋转矩阵
        rot_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        
        # 组合平移和旋转矩阵
        return concatenate_matrices(trans_matrix, rot_matrix)

def main(args=None):
    rclpy.init(args=args)
    node = TFMatrixNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()