#!/bin/bash
set -x
wget https://repo1.maven.org/maven2/ant/optional/1.5.4/optional-1.5.4.jar 
mv optional-1.5.4.jar /usr/share/ant/lib


# For ow_ephemeris: change the mismatch of checksums of downloaded
# artifacts from NASA from SEND_ERROR to WARNING such that unnecessary hacking
# could be avoided
sed -i 's/SEND_ERROR/WARNING/' /ros_ws/src/ow_simulator/ow_ephemeris/CMakeLists.txt


# build PLEXIL
git clone https://git.code.sf.net/p/plexil/git /root/plexil
export PLEXIL_HOME=/root/plexil
export PATH="${PATH}:${PLEXIL_HOME}/scripts"
if [ -z "$LD_LIBRARY_PATH" ]
then
    export LD_LIBRARY_PATH="$_plexil_libpath"
else
    export LD_LIBRARY_PATH="$_plexil_libpath:$LD_LIBRARY_PATH"
fi
echo 'export PLEXIL_HOME=/root/plexil' >> ~/.bashrc
echo 'source $PLEXIL_HOME/scripts/plexil-setup.sh' >> ~/.bashrc
# This is called by Docker, which uses sh not bash
echo 'export PLEXIL_HOME=/root/plexil' >> ~/.profile
echo '. $PLEXIL_HOME/scripts/plexil-setup.sh' >> ~/.profile
. ~/.profile  
. ~/.bashrc
echo "PLEXIL_HOME=\"${PLEXIL_HOME}\""
cd ${PLEXIL_HOME}
make src/configure
cd src
./configure CFLAGS="-g -O2" CXXFLAGS="-g -O2" --prefix=/root/plexil --disable-static --disable-viewer --enable-ipc
cd ${PLEXIL_HOME}
make squeaky-clean
make universalExec plexil-compiler checkpoint

# build GSAP
git clone -b v1.0-OW https://github.com/nasa/GSAP.git /root/gsap
export GSAP_HOME=/root/gsap
echo 'export GSAP_HOME=/root/gsap' >> ~/.bashrc
echo 'export GSAP_HOME=/root/gsap' >> ~/.profile
. ~/.profile
. ~/.bashrc
cd ${GSAP_HOME}
mkdir build
cd build
cmake ..
make
