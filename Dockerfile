FROM ubuntu:20.04

ARG julia_binaries=https://julialang-s3.julialang.org/bin/linux/x64/1.7/julia-1.7.1-linux-x86_64.tar.gz

ENV DEBIAN_FRONTEND noninteractive

RUN  apt-get update \
  && apt-get install -y \
  wget \
  xvfb \
  build-essential \
  qtcreator \
  qt5-default

RUN apt-get clean

RUN mkdir julia_src
RUN wget $julia_binaries -P /julia_src/
RUN tar zxvf julia_src/julia-1.7.1-linux-x86_64.tar.gz -C /julia_src/
RUN rm /julia_src/*.gz

RUN ln -s /julia_src/julia-1.7.1/bin/julia julia
RUN /./julia -e 'using Pkg; Pkg.add("TrajectoryOptimization"); \
    Pkg.add("Altro"); \
    Pkg.add("TrajectoryOptimization"); \
    Pkg.add("Altro"); \
    Pkg.add("RobotDynamics"); \
    Pkg.add("StaticArrays"); \
    Pkg.add("LinearAlgebra"); \
    Pkg.add("RobotDynamics"); \
    Pkg.add("Plots"); \
    Pkg.add("RobotZoo"); \
    Pkg.add("HTTP"); \
    Pkg.add("ElectronDisplay");' 
    # Pkg.add("MechanismGeometries"); \
    # Pkg.add("RigidBodyDynamics"); \
    # Pkg.add("MeshCatMechanisms"); '

    