FROM ros:melodic

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-yaml

#best practice ではない
RUN echo 'alias python=/usr/bin/python3.6' >> /root/.bashrc

RUN echo 'source /ros_entrypoint.sh'  >> /root/.bashrc

ARG project_dir=/root/src/

ADD ./requirements.txt $project_dir

WORKDIR $project_dir

RUN pip3 install -r requirements.txt --no-cache-dir