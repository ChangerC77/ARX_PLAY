import h5py
import os

"""
check the dataset
"""

def check_hdf5_file(file_path):
    """
    检查单个HDF5文件的数据结构
    """
    try:
        with h5py.File(file_path, 'r') as root:
            # print(f"\n成功打开HDF5文件: {file_path}")
            # print(f"文件中的对象数量: {len(root.keys())}")
            
            def traverse_datasets(name, obj):
                if isinstance(obj, h5py.Dataset):
                    # 检查数据集形状
                    if name == 'action':
                        assert obj.shape == (obj.shape[0], 14), f"{file_path}中 action data wrong"
                    elif name == 'action_base':
                        assert obj.shape == (obj.shape[0], 6), f"{file_path}中 action_base 数据形状错误数据形状错误sdd"
                    elif name == 'action_eef':
                        assert obj.shape == (obj.shape[0], 14), f"{file_path}中 action_eef 数据形状错误"
                        
                    # 打印小型数据集内容
                    if obj.size < 20:
                        print(f"  {name}数据样本: {obj[()]}")
                        
                elif isinstance(obj, h5py.Group):
                    # 检查组内数据集形状
                    if name == 'observations/eef':
                        assert 'eef' in obj and obj['eef'].shape == (obj.shape[0], 14), f"{file_path}中observations/eef数据形状错误"
                    elif name == 'observations/effort':
                        assert 'effort' in obj and obj['effort'].shape == (obj.shape[0], 14), f"{file_path}中observations/effort数据形状错误"
                    elif name == 'observations/images/head':
                        assert 'head' in obj and obj['head'].shape == (obj.shape[0], 480, 640, 3), f"{file_path}中observations/images/head数据形状错误"
                    elif name == 'observations/images/left_wrist':
                        assert 'left_wrist' in obj and obj['left_wrist'].shape == (obj.shape[0], 480, 640, 3), f"{file_path}中observations/images/left_wrist数据形状错误"
                    elif name == 'observations/images/right_wrist':
                        assert 'right_wrist' in obj and obj['right_wrist'].shape == (obj.shape[0], 480, 640, 3), f"{file_path}中observations/images/right_wrist数据形状错误"
                    elif name == 'observations/qpos':
                        assert 'qpos' in obj and obj['qpos'].shape == (obj.shape[0], 14), f"{file_path}中observations/qpos数据形状错误"
                    elif name == 'observations/qvel':
                        assert 'qvel' in obj and obj['qvel'].shape == (obj.shape[0], 14), f"{file_path}中observations/qvel数据形状错误"
                    elif name == 'observations/robot_base':
                        assert 'robot_base' in obj and obj['robot_base'].shape == (obj.shape[0], 6), f"{file_path}中observations/robot_base数据形状错误"
            
            root.visititems(traverse_datasets)
            # print(f"文件 {os.path.basename(file_path)} 检查完成")
            
    except Exception as e:
        print(f"处理文件 {file_path} 时出错: {str(e)}")

def process_directory(directory_path):
    """
    遍历目录下的所有HDF5文件
    """
    if not os.path.isdir(directory_path):
        print(f"错误: {directory_path} 不是有效目录")
        return
    
    print(f"\n开始处理目录: {directory_path}")
    file_count = 0
    
    for root, dirs, files in os.walk(directory_path):
        for file in files:
            if file.endswith('.hdf5') or file.endswith('.h5'):
                file_path = os.path.join(root, file)
                check_hdf5_file(file_path)
                file_count += 1
                
    # print(f"\n目录处理完成,共检查 {file_count} 个HDF5文件")

if __name__ == "__main__":
    # 可以处理单个文件或整个目录
    path = "/home/robotics/ARX/ARX_PLAY/mobile_aloha/datasets"
    
    if os.path.isfile(path) and (path.endswith('.hdf5') or path.endswith('.h5')):
        print("处理单个HDF5文件")
        check_hdf5_file(path)
    elif os.path.isdir(path):
        print("处理目录下的所有HDF5文件")
        process_directory(path)
    else:
        print(f"错误: {path} 不是有效的HDF5文件或目录")