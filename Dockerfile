FROM ros:kinetic

# create a "mars" user with sudo privileges
# TODO: add tidying commands to shrink image size
RUN apt-get update && \
    apt-get install -y sudo && \
    useradd -ms /bin/bash mars && \
    usermod -a -G sudo mars && \
    sed -i "s/(ALL:ALL) ALL/(ALL) NOPASSWD: ALL/" "/etc/sudoers" && \
    mkdir -p /home/mars
USER mars
WORKDIR /home/mars

RUN git config --global url.https://github.com/.insteadOf git://github.com/

# create an empty catkin workspace
RUN . /opt/ros/kinetic/setup.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws && \
    catkin_make
WORKDIR /home/mars/catkin_ws

RUN sudo apt-get update && \
    sudo apt-get install -y vim wget

# add the source code for the shared "notifications" module
RUN git clone https://github.com/cmu-mars/notifications-p15 src/notifications

# install ply
RUN cd /tmp && \
    wget -q https://github.com/dabeaz/ply/archive/3.10.tar.gz && \
    tar -xf 3.10.tar.gz && \
    cd ply-3.10 && \
    sudo python setup.py install && \
    sudo rm /tmp/* -rf

# add the source code for this module
ADD ig_action_client src/ig_action_client
ADD ig_action_msgs src/ig_action_msgs
ADD ig_action_server src/ig_action_server

RUN . /opt/ros/kinetic/setup.sh && \
    sudo chown -R $(whoami):$(whoami) . && \
    catkin_make install

RUN . /opt/ros/kinetic/setup.sh && \
    . devel/setup.sh && \
    rosdep update && \
    rospack depends1 ig_action_server && \
    rospack depends1 ig_action_client
