FROM ubuntu:20.04


ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y python3 python3-pip gcc cmake build-essential \
                                vim git libboost-all-dev libfftw3-dev nlohmann-json3-dev libssl-dev
WORKDIR /app 
COPY *.hpp *.cpp conanfile.txt /app/
COPY third/*.hpp /app/

COPY ConanCmake.txt /app/CMakeLists.txt
RUN pip3 install conan && conan profile new default --detect && \
    conan profile update settings.compiler.libcxx=libstdc++11 default
RUN git clone https://github.com/gabime/spdlog.git && \
    cd spdlog && mkdir build && cd build && \
    cmake .. && make -j


RUN mkdir build && cd build && conan install .. && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    cmake --build .

EXPOSE 8080 
ENTRYPOINT [ "/app/build/cmtj_server" ]