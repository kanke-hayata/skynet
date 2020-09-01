FROM python:3.8

RUN pip install numpy

# TA-Lib
RUN wget http://prdownloads.sourceforge.net/ta-lib/ta-lib-0.4.0-src.tar.gz && \
  tar -xvzf ta-lib-0.4.0-src.tar.gz && \
  cd ta-lib/ && \
  ./configure --prefix=/usr && \
  make && \
  make install

ARG project_dir=/root/src/

ADD ./requirements.txt $project_dir

WORKDIR $project_dir

RUN rm -R ta-lib ta-lib-0.4.0-src.tar.gz

RUN pip install -r requirements.txt --no-cache-dir
