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
fi

exit
cd $1

echo "--Cloning acados--"
git clone https://github.com/acados/acados.git acados
cd acados
acados_dir=$PWD

git submodule update --recursive --init

echo "--Installing with CMake--"
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4

echo "--Setting up python interface--"
virtualenv env --python=/usr/bin/python3.$python3_version
source env/bin/activate
pip3 install -e $acados_dir/interfaces/acados_template
pip3 install pyyaml

echo "--add variables to ~/.bashrc--"

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:'${acados_dir}/lib >> ~/.bashrc
echo "export ACADOS_SOURCE_DIR=$acados_dir" >> ~/.bashrc
source ~/.bashrc

printf "\n Install finised. To test the installation you can run one of the examples in $ACADOS_SOURCE_DIR/examples. \
 Remember to source the virtual enviroment by running:\
 source $ACADOS_SOURCE_DIR/build/env/bin/activate before running python3 <your_python_file.py> \n"
