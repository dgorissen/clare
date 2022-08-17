FROM ros:noetic@sha256:dc89df4c67ffb8e35dcb232a71956399e39b3575cfa1e8389e74e1d5a0782638

SHELL ["/bin/bash", "-c"]

# Ros packages
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-noetic-desktop \
      python3-rosdep \
      python3-rosinstall \
      python3-rosinstall-generator \
      python3-wstool build-essential

# Utils
RUN apt-get install -y wget vim keychain tree screen git software-properties-common feh curl
  
#  Openvino dependencies
RUN apt-get install -y build-essential \
   unzip pkg-config libjpeg-dev libpng-dev \
   libtiff-dev libavcodec-dev libavformat-dev \
   libswscale-dev libv4l-dev libxvidcore-dev \
   libx264-dev libgtk-3-dev libcanberra-gtk* \
   libatlas-base-dev gfortran python3-dev

# upgrade cmake to avoid crosscompile bug in the ubuntu bundled cmake (3.16)
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
  apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' && \
  apt-cache policy cmake-data && \
  apt-cache policy cmake-curses-gui && \
  apt-get install -y cmake=3.20.2-0kitware1ubuntu20.04.1 cmake-data=3.20.2-0kitware1ubuntu20.04.1 cmake-curses-gui=3.20.2-0kitware1ubuntu20.04.1

# other dependencies
RUN apt-get update && apt-get install -y  \
  usbutils pulseaudio-utils lame mpg123 audacity libfftw3-dev libconfig-dev libasound2-dev portaudio19-dev \
  libprotobuf-dev protobuf-compiler libtbb-dev libusb-1.0-0-dev \
  alsa-utils i2c-tools pcl-tools libssl-dev mesa-utils nmap swig pulseaudio libpulse-dev libudev-dev \
  libxslt-dev libxml2-dev rsync \
  festival festvox-don festvox-us-slt-hts festvox-rablpc16k festvox-kallpc16k festvox-kdlpc16k \
  libttspico-utils espeak flite flac \
  teensy-loader-cli libncurses5 arduino arduino-builder \
  ir-keytable libinput-tools \
  g++-arm-linux-gnueabihf gcc-arm-linux-gnueabihf libgirepository1.0-dev libgl1-mesa-glx
  
# More ros libs
RUN apt-get update && apt-get install -y \
  ros-noetic-imu-filter-madgwick ros-noetic-robot-localization \
  ros-noetic-vision-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs\
  ros-noetic-sound-play ros-noetic-audio-play \
  ros-noetic-speech-recognition-msgs ros-noetic-audio-capture \
  ros-noetic-image-proc ros-noetic-rviz ros-noetic-realsense2-camera \
  ros-noetic-app-manager ros-noetic-catkin-virtualenv \
  ros-noetic-rosserial-python ros-noetic-rosserial-arduino ros-noetic-rosserial-client \
  ros-noetic-moveit-*

# nodejs & vue
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - && \
  apt-get install -y nodejs && \
  npm install -g @vue/cli

# Not pretty but ensure gpio, i2c groups exist and have same GIDs as on the host
RUN groupadd -g 997 gpio \
    && groupadd -g 999 spi \
    && groupmod -g 998 i2c

# Create user ensuring same uid as host
RUN groupadd -r -g 1001 dgorissen && \
  useradd -ms /bin/bash -u 1001 -g 1001 dgorissen && \
  usermod -a -G users dgorissen && \
  usermod -a -G dialout dgorissen && \
  usermod -a -G audio dgorissen && \
  usermod -a -G video dgorissen && \
  usermod -a -G i2c dgorissen && \
  usermod -a -G gpio dgorissen && \
  usermod -a -G spi dgorissen && \
  usermod -a -G sudo dgorissen && \
  # TODO input group on the host ends up as systemd-network group inside...
  # Needed for access to the IR
  usermod -a -G systemd-network dgorissen

RUN echo 'dgorissen:test' | chpasswd

# Enable passwordless sudo
RUN echo '%dgorissen ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER dgorissen
WORKDIR /home/dgorissen

# Update rosdep
RUN rosdep update

# Set-up necessary Env vars for PyEnv
ENV PYTHON_VERSION 3.7.12
ENV PYENV_ROOT /home/dgorissen/.pyenv
ENV PATH $PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH

# Install pyenv
RUN set -ex \
    && curl https://pyenv.run | bash \
    && pyenv update \
    # openvino needs a shared library, else we only get a static library
    && PYTHON_CONFIGURE_OPTS="--enable-shared" pyenv install $PYTHON_VERSION \
    && pyenv global $PYTHON_VERSION \
    && pyenv rehash

RUN set -ex \
  && pyenv version \
  && python --version \
  && which python \
  && which pip

RUN echo -e '\n### \n\
xhost +\n\
export PYENV_ROOT="$HOME/.pyenv" \n\
export PATH="$PYENV_ROOT/bin:$PATH" \n\
eval "$(pyenv init --path)" \n\
' >> ~/.bashrc

RUN python -m pip install --upgrade pip
RUN pip install wheel
RUN pip install numpy
RUN pip install scipy matplotlib

# install mimic
RUN git clone https://github.com/MycroftAI/mimic.git 
RUN cd mimic && \
  ./autogen.sh && \
  ./configure --prefix="/usr/local" && \
  make -j 2

USER root
RUN cd mimic && make install
USER dgorissen

RUN READTHEDOCS=True pip install picamera
RUN pip install \
  imutils flask flask_cors pyyaml rospkg pyserial platformio \
  netifaces pyopengl pyopengl_accelerate empy \
  pyusb click pyaudio pydub rpi.gpio \
  pocketsphinx webrtcvad respeaker hidapi speechrecognition \ 
  requests-oauthlib lxml catkin-tools

RUN pip install adafruit-extended-bus adafruit-circuitpython-bme680 \
    adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit \
    rpi_ws281x adafruit-circuitpython-neopixel adafruit-blinka evdev \
    adafruit-circuitpython-neopixel-spi

RUN pip install gobject PyGObject playsound gtts defusedxml

# TODO takes forever, does not complete..
# RUN pip install scikit-learn

# Insteall respeaker repos
RUN git clone https://github.com/respeaker/usb_4_mic_array.git && \
  git clone https://github.com/respeaker/pocketsphinx-data.git && \
  git clone --depth 1 https://github.com/respeaker/pixel_ring.git && \
  cd pixel_ring && \
  pip install -e . && \
  git clone https://github.com/introlab/odas.git && \
  mkdir odas/build && \
  cd odas/build && \
  cmake .. && \
  make -j 2

RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && \
  git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git && \
  source /opt/ros/noetic/setup.bash && \
  cd ~/catkin_ws && \
  catkin config --init && \
  catkin build respeaker_ros && \
  make -C build/respeaker_ros install

# Install realsense
RUN git clone --depth 1 --branch v2.50.0 https://github.com/IntelRealSense/librealsense.git && \
  cd librealsense && \
  mkdir build  && cd build && \
  cmake .. \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON

RUN cd librealsense/build && \
  make -j2

RUN cd librealsense/build && \
  cmake .. -DBUILD_EXAMPLES=true && \
  make -j2

RUN cd librealsense/build && \
  cmake .. \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=$(which python3) && \
  make -j1

USER root
RUN cd /home/dgorissen/librealsense/build && make install
USER dgorissen

# Install model zoo downloader
RUN git clone --depth 1 --branch 2021.4.2 https://github.com/openvinotoolkit/open_model_zoo && \
  cd open_model_zoo/tools/downloader && \
  pip install -r requirements.in && \
  python3 downloader.py --output_dir /home/dgorissen/openvino_models --name face-detection-retail-0004

# Install openvino
ARG vinodist=ubuntu20
# ARG vinodist=raspbian
ARG vinofile=l_openvino_toolkit_runtime_${vinodist}_p_2021.4.752
RUN wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2021.4.2/${vinofile}.tgz && \
  tar -xf ${vinofile}.tgz && \
  mv ${vinofile} openvino

RUN cd openvino && \
  source bin/setupvars.sh && \
  mkdir build_samples && cd build_samples && \
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=armv7-a" ~/openvino/deployment_tools/inference_engine/samples/cpp && \
  make -j 2

# TODO move earlier
RUN git clone https://github.com/Koromix/tytools \
  && mkdir -p tytools/build \
  && cd tytools/build \
  && cmake .. \
  && make -j2

# Get some basic packages in place
#RUN cd ~/clare/ros/src/clare_head && pio pkg install

USER root
RUN cd tytools/build && make install
RUN usermod -a -G plugdev dgorissen
USER dgorissen

RUN echo -e '\n### \n\
source ~/openvino/bin/setupvars.sh \n\
source /opt/ros/noetic/setup.bash \n\
source ~/catkin_ws/install/setup.bash \n\
source ~/clare/ros/devel/setup.bash \n\
export POCKETSPHINX_DATA=$HOME/pocketsphinx-data \n\
export ALSA_CARD=1 \n\
' >> ~/.bashrc

RUN mkdir ~/clare
WORKDIR /home/dgorissen/clare

EXPOSE 5000
EXPOSE 8080
EXPOSE 11311

ENTRYPOINT ["/home/dgorissen/clare/entrypoint.sh"]
# https://stackoverflow.com/questions/53543881/docker-run-pass-arguments-to-entrypoint
CMD [""]

#TODO:
# htop