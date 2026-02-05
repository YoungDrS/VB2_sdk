#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import threading
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class TFReader(Node):
    def __init__(self, target_frame='end_effector', source_frame='base_link'):
        super().__init__('tf_reader')
        
        # Use callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frame names
        self.target_frame = target_frame  # 目标关节/末端执行器
        self.source_frame = source_frame  # 基座标系
        
        # Thread locks for data safety
        self._lock = threading.Lock()
        
        # Transform data
        self.transform = None
        self.transform_ts = None
        self.position = None  # [x, y, z]
        self.orientation_rpy = None  # [rx, ry, rz] in radians
        
        # Executor and spin thread
        self._executor = None
        self._spin_thread = None
        self._running = False
        self.timer = None
        self.quaternion = None  # [x, y, z, w]
        
        self.get_logger().info(f'TF Reader initialized - monitoring {target_frame} relative to {source_frame}')
        
    def start_recv(self):
        """Start receiving TF data in background thread"""
        if self._running:
            self.get_logger().warn('TF reader already running')
            return
        
        self._running = True
        
        # Create timer with callback group
        self.timer = self.create_timer(
            0.1,  # 10Hz update rate
            self.timer_callback,
            callback_group=self.callback_group
        )
        
        # Start executor in background thread
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        
        self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self._spin_thread.start()
        
        self.get_logger().info('TF receiving started in background thread')
    
    def _spin_executor(self):
        """Internal method to spin the executor in background thread"""
        try:
            while self._running and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f'Error in spin thread: {e}')
    
    def lookup_matrix(self, parent, child):
        """查询 parent->child 的 4x4 变换矩阵"""
        tf: TransformStamped = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def timer_callback(self):
        """定时器回调，更新TF数据"""
        try:
            # 直接查询 source_frame -> target_frame 的变换
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 提取时间戳
            ts = int(transform.header.stamp.sec * 1000 + 
                    transform.header.stamp.nanosec / 1000000)
            
            # 提取平移
            t = transform.transform.translation
            position = np.array([t.x, t.y, t.z])
            
            # 提取旋转并转换为RPY
            q = transform.transform.rotation
            Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            orientation_rpy = R.from_matrix(Rm).as_euler('xyz', degrees=False)
            
            # 构建4x4变换矩阵
            T = np.eye(4)
            T[:3, :3] = Rm
            T[:3, 3] = position
            
            # 线程安全地更新数据
            with self._lock:
                self.transform = T
                self.transform_ts = ts
                self.position = position
                self.orientation_rpy = orientation_rpy
                self.quaternion = np.array([q.x, q.y, q.z, q.w])

        except Exception as e:
            # TF查询失败时不打印错误（避免刷屏）
            pass

    def get_transform(self):
        """获取完整的变换信息"""
        return self.transform, self.transform_ts

    def get_position(self):
        """获取位置信息 [x, y, z]"""
        return self.position, self.transform_ts

    def get_orientation_rpy(self):
        """获取姿态信息 [rx, ry, rz] (弧度)"""
        with self._lock:
            return self.orientation_rpy, self.transform_ts
    
    def get_orientation_rpy_degrees(self):
        """获取姿态信息 [rx, ry, rz] (角度)"""
        with self._lock:
            if self.orientation_rpy is not None:
                rpy_degrees = [math.degrees(angle) for angle in self.orientation_rpy]
                return rpy_degrees, self.transform_ts
            return None, self.transform_ts
    
    def get_pose_info(self):
        """获取完整的位姿信息"""
        with self._lock:
            if self.position is not None and self.orientation_rpy is not None and self.transform_ts is not None and self.quaternion is not None:
                pose_info = {
                    'position': self.position.copy(),
                    'orientation_rpy_rad': self.orientation_rpy.copy(),
                    'orientation_rpy_deg': [math.degrees(angle) for angle in self.orientation_rpy],
                    'timestamp': self.transform_ts,
                    'quaternion': self.quaternion.copy() if self.quaternion is not None else None
                }
                return pose_info
            return None

    def is_data_available(self):
        """检查是否有可用的TF数据"""
        with self._lock:
            return self.transform is not None

    def set_frames(self, target_frame, source_frame):
        """更新要监听的坐标系"""
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.get_logger().info(f'Updated frames - monitoring {target_frame} relative to {source_frame}')

    def list_available_frames(self):
        """列出当前可用的TF坐标系"""
        try:
            # 等待一下让TF buffer填充
            time.sleep(1.0)
            frame_string = self.tf_buffer.all_frames_as_string()
            self.get_logger().info(f'Available frames:\n{frame_string}')
            return frame_string
        except Exception as e:
            self.get_logger().error(f'Could not get available frames: {e}')
            return None
    
    def stop_recv(self):
        """Stop receiving TF data"""
        if not self._running:
            return
        
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
        
        self.get_logger().info('TF receiving stopped')
    
    def is_running(self):
        """Check if TF reader is running"""
        return self._running


def main(args=None):
    rclpy.init(args=args)
    
    # 创建TF读取器，使用base_link作为基坐标系
    tf_reader = TFReader(target_frame='right_hand_index_tool_link', source_frame='right_index_tip_link')
    tf_reader.start_recv()
    
    # 等待TF数据可用
    print("Waiting for TF data to become available...")
    time.sleep(2.0)
    
    # 列出可用的坐标系（可选，用于调试）
    # tf_reader.list_available_frames()
    
    # 主循环，定期打印TF信息
    try:
        while rclpy.ok():
            if tf_reader.is_data_available():
                pose_info = tf_reader.get_pose_info()
                if pose_info:
                    pos = pose_info['position']
                    rpy_deg = pose_info['orientation_rpy_deg']
                    rpy_rad = pose_info['orientation_rpy_rad']
                    ts = pose_info['timestamp']
                    
                    print(f"Timestamp: {ts}")
                    print(f"Position: x={pos[0]:.6f}, y={pos[1]:.6f}, z={pos[2]:.6f}")
                    print(f"Orientation (rad): rx={rpy_rad[0]:.6f}, ry={rpy_rad[1]:.6f}, rz={rpy_rad[2]:.6f}")
                    print("-" * 50)
            else:
                print("Waiting for TF data...")
            
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
    
    tf_reader.stop_recv()
    tf_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
