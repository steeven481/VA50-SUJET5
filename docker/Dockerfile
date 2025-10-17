ARG ROS_DISTRO=noetic

FROM palrobotics/tiago-devel-pc AS base-dependencies

ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris

RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirror.math.princeton.edu/pub/ubuntu/|g' /etc/apt/sources.list
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries
RUN wget -O - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update && apt-get install --no-install-recommends -y --fix-missing \
  # development tools
    build-essential \
    cmake \
    cmake-curses-gui \
    gdb \
    git \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-pip \
    python3-catkin-tools \
    python3-ipython \
    python3-ruamel.yaml \
  # editors
    featherpad \
    gedit \
    nano \
    vim \
  # system tools
    curl \
    gnupg2 \
    iproute2 \
    iputils-ping \
    locales \
    locate \
    lsb-release \
    mesa-utils \
    net-tools \
    rsync \
    ssh \
    sudo \
    tzdata \
  # ROS packages
    ros-${ROS_DISTRO}-actionlib-tools \
    ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-ros-numpy \
    ros-${ROS_DISTRO}-vision-msgs \
  # ViSP 3rd party dependencies
    libdc1394-22-dev \
    libeigen3-dev \
    liblapack-dev \
    libopencv-dev \
    libv4l-dev \
    libx11-dev \
    libzbar-dev \
    nlohmann-json3-dev \
  # Optional ViSP 3rd parties
    libcoin-dev \
    libdmtx-dev \
    libgsl-dev \
    libjpeg-dev \
    libogre-1.9-dev \
    libois-dev \
    libpcl-dev \
    libpng-dev \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install dependency for Tiago tutorials
RUN apt update && apt install -y \
  pal-gallium-aruco-ros

RUN echo "Set disable_coredump false" >> /etc/sudo.conf
# Installer les dépendances Python
RUN sudo apt-get update && sudo apt-get install -y python3-pip
RUN pip3 install opencv-python rospkg catkin-pkg
# Configure sshd server settings
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PubkeyAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_development \
  && mkdir /run/sshd

ENV USER=pal
ENV HOME=/home/${USER}
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_WORKSPACE=/home/${USER}/ros_ws

RUN locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8

# creating and configuring user
ARG UID=1000
ARG GID=1000
RUN set -eux; \
if getent passwd "${UID}" > /dev/null; then \
  EXISTING_USER="$(getent passwd ${UID} | cut -d: -f1)"; \
  echo "✅ User with UID ${UID} exists: ${EXISTING_USER}, renaming to ${USER}..."; \
  usermod -l "${USER}" "${EXISTING_USER}"; \
  groupmod -n "${USER}" "${EXISTING_USER}"; \
  usermod -d "/home/${USER}" -m "${USER}"; \
else \
  echo "✅ UID ${UID} is free, creating user ${USER}..."; \
  groupadd -g "${GID}" "${USER}"; \
  useradd -m -u "${UID}" -g "${GID}" -s /bin/bash "${USER}"; \
fi
RUN chown -R ${USER}:${USER} /home/${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_${USER} \
  && chmod 0440 /etc/sudoers.d/99_${USER} \
  && chown root:root /etc/sudoers.d/99_${USER}
RUN usermod -a -G video,dialout ${USER}

# configuring sshd entrypoint to authorise the new user for ssh access and
# optionally update UID and GID when invoking the container with the entrypoint
# script
COPY ./config/sshd_entrypoint.sh /sshd_entrypoint.sh
RUN chmod 744 /sshd_entrypoint.sh

# create the credentials to be able to pull private repos using ssh
RUN echo "session required pam_limits.so" | tee --append /etc/pam.d/common-session > /dev/null
RUN chmod 4755 /usr/bin/sudo
RUN mkdir /root/.ssh/ && ssh-keyscan github.com | tee -a /root/.ssh/known_hosts

# building ROS workspace
USER ${USER}
RUN mkdir -p ${ROS_WORKSPACE}/src
WORKDIR ${ROS_WORKSPACE}
ENV LD_LIBRARY_PATH=/usr/local/lib
COPY ./config/bashrc.sh ${HOME}/bashrc.sh
RUN cat ${HOME}/bashrc.sh >> ${HOME}/.bashrc && rm ${HOME}/bashrc.sh

### INSTALLING VISP FROM SOURCES

USER ${USER}
WORKDIR /home/${USER}
ENV VISP_WS=/home/${USER}/visp-ws

RUN mkdir -p ${VISP_WS} \
  && cd ${VISP_WS} \
  && git clone https://github.com/lagadic/visp-images.git \
  && git clone https://github.com/lagadic/visp --branch v3.5.0 \
  && mkdir build \
  && cd build \
  && cmake ../visp \
  && make -j$(nproc) \
  && sudo make install \
  && cd .. \
  && rm -rf build

### INSTALLING libPeR FROM SOURCES

RUN cd ${VISP_WS} \
  && git clone https://github.com/PerceptionRobotique/libPeR_base.git \
  && cd libPeR_base \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j$(nproc) \
  && sudo make install \
  && cd .. \
  && rm -rf build

CMD ["/bin/bash"]

# starting as ROS user
USER ${USER}
WORKDIR ${ROS_WORKSPACE}

# cleaning image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
