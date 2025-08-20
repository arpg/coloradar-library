FROM osrf/ros:noetic-desktop-full


RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


WORKDIR /scripts
COPY bag_rgb_to_grayscale.py .


CMD ["bash"]
