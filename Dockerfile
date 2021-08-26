FROM ros:noetic

# Remove user interactive
ENV DEBIAN_FRONTEND noninteractive

# Update repo packages
RUN apt-get update && \
    apt-get upgrade -y

# Install python
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN apt-get install -y python3-pip
RUN pip install --upgrade pip && \
    pip install --upgrade setuptools

# ROS environment
WORKDIR /workspace
RUN rosdep update
RUN mkdir src
RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> /etc/bash.bashrc && \
    echo "source /workspace/devel/setup.bash" >> /etc/bash.bashrc

# Install require dependencies
ADD requirements.txt .
RUN pip install -r requirements.txt

# Add catkin build script
ADD build.bash .
