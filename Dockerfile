FROM cmu-mars/base

# add the source code for the shared "notifications" module
RUN git clone https://github.com/cmu-mars/notifications-p15 \
      --depth 1 \
      src/notifications

# install ply
RUN cd /tmp && \
    wget -q https://github.com/dabeaz/ply/archive/3.10.tar.gz && \
    tar -xf 3.10.tar.gz && \
    cd ply-3.10 && \
    sudo python setup.py install && \
    sudo rm /tmp/* -rf

# install the ROS navigation stack (from source)
ENV ROS_NAVIGATION_MSGS_VERSION 1.13.0
RUN wget -q "https://github.com/ros-planning/navigation_msgs/archive/${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    tar -xvf "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    rm "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    mv "navigation_msgs-${ROS_NAVIGATION_MSGS_VERSION}" navigation_msgs && \
    rm navigation_msgs/README.md && \
    mv navigation_msgs/* src && \
    rm -rf navigation_msgs

# add the source code for this module
ADD ig_action_client src/ig_action_client
ADD ig_action_msgs src/ig_action_msgs
ADD ig_action_server src/ig_action_server

RUN rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro=kinetic

RUN . /opt/ros/kinetic/setup.sh && \
    sudo chown -R $(whoami):$(whoami) . && \
    catkin_make install

RUN . /opt/ros/kinetic/setup.sh && \
    . devel/setup.sh && \
    rospack depends1 ig_action_server && \
    rospack depends1 ig_action_client

ADD entrypoint.sh entrypoint.sh
ENTRYPOINT entrypoint.sh
