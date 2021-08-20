FROM ros:noetic

# Remove user interactive
ENV DEBIAN_FRONTEND noninteractive

# Update repo packages
RUN apt-get update && \
    apt-get upgrade -y

# Python initialization
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN apt-get install -y python3-pip
RUN pip install --upgrade pip && \
    pip install --upgrade setuptools

# Install python packages
WORKDIR /workspace
ADD requirements.txt /workspace
RUN pip install -r requirements.txt

# Install ROS
RUN rosdep update

# ROS entrypoint
RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> /etc/bash.bashrc
