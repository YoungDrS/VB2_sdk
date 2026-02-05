#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ================= 用户配置 =================
BAG_PATH = "rosbag2_2026_01_08-17_06_50"
TOPIC_NAME = "/joint_states"

LEFT_JOINTS = [
    "L_SHOULDER_P",
    "L_SHOULDER_R",
    "L_SHOULDER_Y",
    "L_ELBOW_Y",
    "L_WRIST_P",
    "L_WRIST_Y",
    "L_WRIST_R",
]

RIGHT_JOINTS = [
    "R_SHOULDER_P",
    "R_SHOULDER_R",
    "R_SHOULDER_Y",
    "R_ELBOW_Y",
    "R_WRIST_P",
    "R_WRIST_Y",
    "R_WRIST_R",
]
# ============================================


storage_options = StorageOptions(
    uri=BAG_PATH,
    storage_id="sqlite3"
)

converter_options = ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)

reader = SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topic_types}

if TOPIC_NAME not in type_map:
    raise RuntimeError(f"Topic {TOPIC_NAME} not found in bag")

msg_type = get_message(type_map[TOPIC_NAME])


time_list = []
pos_data = {}

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic != TOPIC_NAME:
        continue

    msg = deserialize_message(data, msg_type)
    time_list.append(t * 1e-9)

    for i, name in enumerate(msg.name):
        pos_data.setdefault(name, []).append(msg.position[i])


time = np.array(time_list)
time -= time[0]


# ================= 画图函数 =================
def plot_arm(joint_list, title):
    plt.figure(figsize=(10, 6))
    for joint in joint_list:
        if joint not in pos_data:
            print(f"[WARN] Joint {joint} not found in bag")
            continue
        plt.plot(time, pos_data[joint], label=joint)

    plt.xlabel("Time [s]")
    plt.ylabel("Position [rad]")
    plt.title(title)
    plt.legend()
    plt.grid(True)


# ================= 左右臂分图 =================
plot_arm(LEFT_JOINTS, "Left Arm Joint Positions")
plot_arm(RIGHT_JOINTS, "Right Arm Joint Positions")

plt.show()

