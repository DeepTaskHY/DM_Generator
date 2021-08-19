FROM ros:melodic-ros-core-bionic

# Repo update
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update

# Install python
RUN apt-get install -y python3 python3-distutils curl && \
    curl "https://bootstrap.pypa.io/get-pip.py" -o get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py

# Install python packages
WORKDIR /workspace
ADD requirements.txt /workspace
RUN pip install --upgrade pip \
 && pip install --upgrade google-auth-oauthlib \
 && pip install --upgrade pyasn1-modules \
 && pip install -r requirements.txt
