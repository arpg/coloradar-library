FROM osrf/ros:noetic-desktop-full


RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y python3-pip && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


RUN pip3 install tqdm


WORKDIR /scripts
COPY bag_rgb_to_grayscale.py start_viewer.sh .
RUN chmod +x /scripts/start_viewer.sh


CMD ["bash"]
