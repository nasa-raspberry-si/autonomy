#!/bin/bash
set -x
wget https://repo1.maven.org/maven2/ant/optional/1.5.4/optional-1.5.4.jar 
mv optional-1.5.4.jar /usr/share/ant/lib

#   The source codes of ow_simulator and ow_autnomy that are used
#   in our project are some versions between release 8 and realse 9.
#   Therefore, it needs to switch ow_simulator and ow_autonomy to
#   the corresponding commits after pulling down from the master
#   branch of the two repos (release 9).
#
#   -   ow_simulator: commit 662fea6
#   -   ow_autonomy: commit 34f36ab
#
#   The codes of the two repos are saved in /ros_ws/src/ow_simulator
#   and /ros_ws/src/ow_autonomy in the docker image.

#   The following commands should be commented/removed when our
#   project starts to use Realse 9 of ow_simulator and ow_autonomy.
cd /ros_ws/src/ow_simulator
git checkout 662fea6 -b v8-owplexil
cd /ros_ws/src/ow_autonomy
git checkout 34f36ab -b v8-owplexil
cd /ros_ws/src/ow_europa
git checkout tags/8.0 -b v8-owplexil
cd /ros_ws/src/irg_open
git checkout tags/8.0 -b v8-owplexil
cd /ros_ws

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
