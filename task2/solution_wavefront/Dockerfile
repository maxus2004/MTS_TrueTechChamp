FROM ubuntu:24.04

WORKDIR /app

RUN apt-get update && apt-get install -y g++ cmake

COPY ./deps ./deps
COPY ./CMakeLists.txt ./
COPY ./*.cpp ./
COPY ./*.h ./

RUN cmake -S . -B ./build 
RUN cmake --build ./build