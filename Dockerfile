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

# Install require dependencies
WORKDIR /workspace
ADD requirements.txt .
RUN pip install -r requirements.txt

# Setup ROS environment
RUN rosdep update
ADD docker-entrypoint.sh .
RUN chmod +x docker-entrypoint.sh
ENTRYPOINT ["/workspace/docker-entrypoint.sh"]
CMD ["roslaunch", "dm_generator", "dm_launcher.launch"]
