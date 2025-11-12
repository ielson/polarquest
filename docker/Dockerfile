FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-lc"]

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

# installing needed packages
RUN apt-get update && apt-get install -y \
    sudo \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator \
    neovim \
    git \
    eog \
    python3-pip

RUN pip3 install gekko

# making sure user is the same as mine so I can have autocomplete without problems
RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
  && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
  && usermod -aG sudo "${USERNAME}" \
  && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME} \
  && chmod 0440 /etc/sudoers.d/90-${USERNAME}

# source ros default env and my env if available
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/${USERNAME}/.bashrc \
 && echo '[ -f "$HOME/code/polarquest/ws/devel/setup.bash" ] && source "$HOME/code/polarquest/ws/devel/setup.bash"' >> /home/${USERNAME}/.bashrc \
 && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/code/polarquest/ws
COPY --chown=${USERNAME}:${USERNAME} init_ws.sh /usr/local/bin/init_ws.sh
RUN chmod +x /usr/local/bin/init_ws.sh

CMD ["bash","-lc","/usr/local/bin/init_ws.sh"]
