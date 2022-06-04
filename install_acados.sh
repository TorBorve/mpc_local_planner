#!/bin/bash
#This script is for installing acados
# The steps are from this guide: https://docs.acados.org/installation/index.html

if [ -z $1 ]
then
    echo -e "\033[31m Specifiy the location of the Ipopt source directory in the first argument. \033[0m"
    exit
fi

python3_version=$(python3 -c 'import sys; print(sys.version_info[1])')

if [ -z $python3_version ]
then
    echo -e "\033[31m Could not find python3 version. \033[0m"
    exit
fi

if ! command -v virtualenv &> /dev/null
then
    echo -e "\033[31m Could not find virtualenv command. Please install it.\033[0m"
    command virtualenv -v
    exit
fi

if ! command -v pip3 &> /dev/null
then 
    echo -e "\033[31m Could not find pip3 command. Please install it.\033[0m"
    command pip3 -v
    exit
fi

# exit if any errors occured
set -e 
cd $1

printf "\n\t--Cloning acados--\n"
git clone https://github.com/acados/acados.git acados
cd acados
acados_dir=$PWD

git submodule update --recursive --init

printf "\n\t--Installing with CMake--\n"
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4

printf "\n\t--Setting up python interface--\n"
virtualenv env --python=/usr/bin/python3.$python3_version
source env/bin/activate
pip3 install -e $acados_dir/interfaces/acados_template
pip3 install pyyaml

printf "\n\t--add variables to ~/.bashrc--\n"
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:'${acados_dir}/lib >> ~/.bashrc
echo "export ACADOS_SOURCE_DIR=$acados_dir" >> ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${acados_dir}/lib
export ACADOS_SOURCE_DIR=$acados_dir

printf "\n\t--running an example to test that it works as expected--\n"
source $acados_dir/build/env/bin/activate
cd $acados_dir/examples/acados_python/getting_started
# use yes to say yes to installing t_renderer
yes | python3 minimal_example_closed_loop.py

printf "\nSeems like the installation was sucessfull. Now you can try building the mpc as described in the readme.\n"