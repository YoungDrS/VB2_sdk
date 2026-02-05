import numpy as np
from scipy.spatial.transform import Rotation as R


# 旋转矩阵
R_mat_arm0 = np.array([
#  [-0.36432, 0.931196, -0.0120144],
#  [ -0.624465,  -0.234703,   0.744955],
#  [   0.69088,   0.278905,   0.667006]

 [-0.36432,  -0.931196, -0.0120144],
 [0.624465,  -0.234703,  -0.744955],
 [0.69088,  -0.278905,   0.667006]
])

# 创建 Rotation 对象
rot = R.from_matrix(R_mat_arm0)

# 提取欧拉角 (ZYX 顺序 → roll around X, pitch around Y, yaw around Z)
rpy_rad = rot.as_euler('xyz', degrees=False)   # 弧度
rpy_deg = rot.as_euler('xyz', degrees=True)    # 角度

print("arm0 RPY (rad):", rpy_rad)
print("arm0 RPY (deg):", rpy_deg)

# 提取四元数 (顺序: x, y, z, w)
quaternion = rot.as_quat()
print("arm0 四元数 (x, y, z, w):", quaternion)

def left_to_right_hand_quaternion(translation, quaternion):
    """
    使用四元数进行左手到右手坐标系的转换
    
    参数:
    translation: [x, y, z] 位置向量
    quaternion: [x, y, z, w] 四元数
    
    返回:
    right_translation: 右手坐标系的位置 [x, y, z]
    right_quaternion: 右手坐标系的四元数 [x, y, z, w]
    """
    # 位置转换: z坐标取反
    right_translation = np.array([translation[0], -translation[1], translation[2]])
    
    # 绕Z轴180度四元数 (用于补偿Y轴取反)  
    flip_quat = np.array([0.0, 0.0, 1.0, 0.0])  # !(x, y, z, w)
        
    # 四元数乘法: q_right = q_left * q_flip
    x, y, z, w = quaternion
    flip_x, flip_y, flip_z, flip_w = flip_quat
    
    right_quaternion = np.array([
        w*flip_x + x*flip_w + y*flip_z - z*flip_y,
        w*flip_y - x*flip_z + y*flip_w + z*flip_x,
        w*flip_z + x*flip_y - y*flip_x + z*flip_w,
        w*flip_w - x*flip_x - y*flip_y - z*flip_z
    ])
    
    # 归一化四元数
    right_quaternion = right_quaternion / np.linalg.norm(right_quaternion)
    
    return right_translation, right_quaternion

def left_to_right_hand_matrix(translation, rotation_matrix):
    """
    使用旋转矩阵进行左手到右手坐标系的转换
    
    参数:
    translation: [x, y, z] 位置向量
    rotation_matrix: 3x3 旋转矩阵
    
    返回:
    right_translation: 右手坐标系的位置 [x, y, z]
    right_rotation_matrix: 右手坐标系的旋转矩阵 3x3
    """
    # 位置转换: z坐标取反
    right_translation = np.array([translation[0], -translation[1], translation[2]])
    
    # 旋转矩阵转换: 绕X轴旋转180度
    flip_z_matrix = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0,  1]
    ])
    
    right_rotation_matrix = rotation_matrix @ flip_z_matrix
    
    return right_translation, right_rotation_matrix

# 示例：左手坐标系的pose
left_translation = np.array([0.05327059, -0.03276486,  0.42499186])

# 方法1: 使用四元数转换
right_translation_quat, right_quaternion = left_to_right_hand_quaternion(
    left_translation, quaternion  # 这里传递四元数，不是旋转矩阵
)

# 方法2: 使用旋转矩阵转换
right_translation_mat, right_rotation_matrix = left_to_right_hand_matrix(
    left_translation, R_mat_arm0  # 这里传递旋转矩阵
)

print("\n=== 使用四元数转换 ===")
print("左手坐标系位置:", left_translation)
print("右手坐标系位置:", right_translation_quat)
print("右手四元数:", right_quaternion)

print("\n=== 使用旋转矩阵转换 ===")
print("左手坐标系位置:", left_translation)
print("右手坐标系位置:", right_translation_mat)
print("右手旋转矩阵:\n", right_rotation_matrix)

# 验证两种方法的一致性
rot_from_quat = R.from_quat(right_quaternion)
matrix_from_quat = rot_from_quat.as_matrix()

print("\n=== 验证一致性 ===")
print("从四元数得到的旋转矩阵:\n", matrix_from_quat)
print("直接转换的旋转矩阵:\n", right_rotation_matrix)
print("矩阵差异:\n", matrix_from_quat - right_rotation_matrix)
