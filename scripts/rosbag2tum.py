import rosbag
import cv2
from cv_bridge import CvBridge
import os

# 设置文件路径
bag_file = '/home/vio/scripts/2024-06-18-15-52-46.bag'
output_dir = '/home/vio/Datasets/long-term/seq03/data.txt'
output_dir_left = '/home/vio/Datasets/long-term/seq03/cam0/'
output_dir_right = '/home/vio/Datasets/long-term/seq03/cam1/'
image_topic_left = '/mynteye/left/image_mono'
image_topic_right = '/mynteye/right/image_mono'


# 创建输出目录
if not os.path.exists(output_dir_left):
    os.makedirs(output_dir_left)
if not os.path.exists(output_dir_right):
    os.makedirs(output_dir_right)

# 初始化 CvBridge
bridge = CvBridge()

# 读取 rosbag 文件
with rosbag.Bag(bag_file, 'r') as bag:
    timestamp = None
    timestamps = []
    for topic, msg, t in bag.read_messages(topics=[image_topic_left, image_topic_right]):
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if t.to_nsec() < 1718697233596840101:
            continue

        if topic == image_topic_left:
            timestamp = str(t.to_nsec())
            timestamps.append(timestamp)
            # 生成输出文件名
            output_file = os.path.join(output_dir_left, f"{timestamp}.png")

            # 保存图像
            cv2.imwrite(output_file, cv_image)
            # cv2.imshow('image', cv_image)
            print(f"Saved {output_file}")
        elif topic == image_topic_right:
            # 生成输出文件名
            output_file = os.path.join(output_dir_right, f"{timestamp}.png")

            # 保存图像
            cv2.imwrite(output_file, cv_image)
            # cv2.imshow('image', cv_image)
            print(f"Saved {output_file}")
        # cv2.waitKey(1)
        with open(output_dir, 'w') as file:
            for item in timestamps:
                file.write(f"{item}\n")

print("All images have been saved.")
