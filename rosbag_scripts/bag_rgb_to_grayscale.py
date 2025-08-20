#!/usr/bin/env python

import rosbag


def print_topics(bag):
    info = bag.get_type_and_topic_info()
    for topic_name, topic_info in info.topics.items():
        print(f"Topic: {topic_name}")
        print(f"Type: {topic_info.msg_type}")
        print(f"Message count: {topic_info.message_count}")
        print("----------------------")


if __name__ == "__main__":
    # bag_path = '/home/arpg/coloradar_plus/bags/bike_path_run0.bag'
    bag_path = '/host/Downloads/total.bag'
    
    bag = rosbag.Bag(bag_path)
    print_topics(bag)
    bag.close()
