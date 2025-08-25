#!/usr/bin/env python3
import rosbag
from cv_bridge import CvBridge
import cv2
import numpy as np
from tqdm import tqdm


def to_gray(cv_img, enc):
    if enc in ("mono8", "8UC1"):
        return cv_img.astype(np.uint8, copy=False)
    if enc in ("mono16", "16UC1", "16SC1"):
        return cv2.convertScaleAbs(cv_img, alpha=1.0/256.0)
    # Try BGR then RGB
    try:
        return cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    except cv2.error:
        return cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)


def convert(bag_path, in_topic, out_topic):
    if in_topic == out_topic:
        raise ValueError("out_topic must be different from in_topic to preserve originals.")

    bridge = CvBridge()
    count = 0

    # Open same file twice: one reader (fixed view of current contents), one appender
    with rosbag.Bag(bag_path, 'r') as reader, rosbag.Bag(bag_path, 'a', compression=reader.compression) as writer:
        # (Optional) sanity: ensure input topic is Image
        types = reader.get_type_and_topic_info().topics
        if in_topic not in types or types[in_topic].msg_type not in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
            raise ValueError(f"Topic {in_topic} is {types.get(in_topic).msg_type if in_topic in types else 'missing'}, expected sensor_msgs/Image or CompressedImage")

        # Stream ONLY the source topic so we don't re-read what we append
        for _, msg, ts in tqdm(reader.read_messages(topics=[in_topic]), desc="Appending grayscale"):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            gray = to_gray(cv_img, msg.encoding or "mono8")
            gray_msg = bridge.cv2_to_imgmsg(gray, encoding='mono8')
            gray_msg.header = msg.header  # keep frame_id + stamp
            writer.write(out_topic, gray_msg, ts)
            count += 1

    print(f"Wrote {count} grayscale frames to {out_topic}.")


def print_topics(bag):
    info = bag.get_type_and_topic_info()
    for topic_name, topic_info in info.topics.items():
        print(f"Topic: {topic_name}")
        print(f"Type: {topic_info.msg_type}")
        print(f"Message count: {topic_info.message_count}")
        print("----------------------")


if __name__ == "__main__":
    bag_path = '/host/coloradar_plus/bags/bike_path_run0.bag'
    # bag_path = '/host/Downloads/total.bag'
    topic = '/camera/color/image_raw'
    out_topic = 'camera/grayscale/image_raw'
    out_bag = bag_path.replace('.bag', '_gray.bag')

    # print_topics(rosbag.Bag(bag_path, 'r'))
    convert(bag_path, topic, out_topic)
