FROM ros:melodic

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-yaml \
    python-catkin-tools \
    wget

RUN wget https://cmake.org/files/v3.18/cmake-3.18.2.tar.gz && \
    tar xvf cmake-3.18.2.tar.gz && \
    cd cmake-3.18.2 && \
    ./bootstrap && make && make install && \
    cd .. && rm -r cmake-3.18.2.tar.gz cmake-3.18.2 && \
    export PATH=$HOME/cmake-install/bin:$PATH && \
    export CMAKE_PREFIX_PATH=$HOME/cmake-install:$CMAKE_PREFIX_PATH

RUN pip3 install numpy

# TA-Lib
RUN wget 'http://prdownloads.sourceforge.net/ta-lib/ta-lib-0.4.0-src.tar.gz' && \
  tar xvzf ta-lib-0.4.0-src.tar.gz && \
  cd ta-lib/ && \
  ./configure --prefix=/usr && \
  make && \
  make install  && \
  cd ../ && \
  rm -rf ta-lib-0.4.0-src.tar.gz && \
  rm -rf ta-lib

ADD ./requirements.txt /root/

ADD ./src /root/catkin_ws/src/bf

RUN pip3 install -r /root/requirements.txt --no-cache-dir

#### discord ####

RUN apt-get update && apt-get install -y \
    libffi-dev \
    python-libnacl \
    python3-dev

RUN pip3 install discord.py
