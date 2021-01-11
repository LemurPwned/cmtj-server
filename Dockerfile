FROM ubuntu:20.04


ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y python3 python3-pip gcc cmake build-essential \
                                vim git libssl-dev
WORKDIR /app 
COPY *.hpp *.cpp conanfile.txt /app/
COPY third/*.hpp /app/third/

COPY ConanCmake.txt /app/CMakeLists.txt
RUN pip3 install conan && conan profile new default --detect && \
    conan profile update settings.compiler.libcxx=libstdc++11 default

RUN sed -i '/".\/third\/httplib.h"/c\#include <httplib.h>' /app/Engine.hpp && \
    sed -i '/".\/third\/httplib.h"/c\#include <httplib.h>' /app/TaskParser.hpp

RUN mkdir build && cd build && conan install .. --build missing
RUN cd build && cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && cmake --build .

EXPOSE 8080 
ENTRYPOINT [ "/app/build/bin/cmtj_server" ]