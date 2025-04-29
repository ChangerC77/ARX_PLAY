import numpy as np
import pickle
import rospy
import h5py
from tqdm import tqdm

from msg._JointInformation import JointInformation
from msg._JointControl import JointControl

class Replay():
    def __init__(self, data_path, rate=100):
        """
        left_arm: '/joint_control'
        right_arm: '/joint_control2'
        """
        if data_path.endswith(".pkl"):
            self.read_from_pickle(data_path)
        elif data_path.endswith(".hdf5"):
            self.read_from_hdf5(data_path)

        # init publisher
        self.ros_rate = rospy.Rate(rate)
        # left arm
        self.left_arm_pub = rospy.Publisher(
            name="/joint_control",
            data_class=JointControl,
            queue_size=10,
        )
        # right arm
        self.right_arm_pub = rospy.Publisher(
            name="/joint_control2",
            data_class=JointControl,
            queue_size=10,
        )

    def read_from_hdf5(self, hdf5_path):
        with h5py.File(hdf5_path, 'r') as root:
            # 打印文件结构以便调试
            print("HDF5 file structure:")
            def print_group(name, obj):
                print(name, obj)
            root.visititems(print_group)

            # 尝试读取可能包含关节角度的数据
            try:
                qpos_data = np.array(root["observations/qpos"][:], dtype=np.float32)
                print("Shape of qpos_data:", qpos_data.shape)
                # 假设qpos_data包含左右机械臂的数据，前7个是左臂，后7个是右臂
                left_arm_array = qpos_data[:, :7]  # 前7个是左臂关节角度
                right_arm_array = qpos_data[:, 7:14]  # 后7个是右臂关节角度
            except KeyError as e:
                print(f"Error: Unable to find joint angles data in HDF5 file. Available keys: {list(root.keys())}")
                raise

        self.arm_left_array = left_arm_array
        self.arm_right_array = right_arm_array
        self.data_length = self.arm_right_array.shape[0]
        print(f"[Replay] load array length: {self.arm_left_array.shape[0]}")
    
    def read_from_pickle(self, pickle_path):
        with open(pickle_path, 'rb') as f:
            data_array = pickle.load(f)

        self.arm_right_array = data_array["right"]
        self.arm_left_array = data_array["left"]
        self.data_length = self.arm_right_array.shape[0]
        print(f"[Replay] load array length: {self.arm_left_array.shape[0]}")

    def replay(self):
        for idx in tqdm(range(self.data_length)):
            if not rospy.is_shutdown():
                left_value = self.arm_left_array[idx]
                right_value = self.arm_right_array[idx]

                left_joint_state_msg = JointControl()
                right_joint_state_msg = JointControl()

                left_joint_state_msg.joint_pos = left_value[:7]
                right_joint_state_msg.joint_pos = right_value[:7]

                self.left_arm_pub.publish(left_joint_state_msg)
                self.right_arm_pub.publish(right_joint_state_msg)
                self.ros_rate.sleep()


if __name__ == "__main__":
    rospy.init_node('replay', anonymous=True)
    path = "/home/robotics/ARX/ARX_PLAY/mobile_aloha/datasets/episode_49.hdf5"
    replayer = Replay(
        data_path=path, 
        rate=30
    )
    replayer.replay()