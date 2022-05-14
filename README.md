# MAPE-K loop based autonomy realized by Rainbow framework

# This branch is based on the release 8 of ow_simulator

# Build Process For The Docker Image of ow_simulator
   1. Build the docker image for the release 8 of OceanWATERS simulator, raspberrysi/oceanworld:r8
      <br>`cd ./oceanworld && ./build.sh melodic r8`
   3. Build the autonomy

# Run rs\_autonomy
   export EVALUATION\_ROOT\_DIR=< evaluation directory >
   export OW\_PLEXIL\_LIB\_SOURCE\_DIR=< PLEXIL library source directory >
   export PLEXIL\_LIB\_COMPILED\_PLAN\_DIR=< the direcotry of compiled PLEXIL plan in devel dir >
   roslaunch rs\_autonomy rs\_autonomy mission\_spec\_filename:=Mission1.txt

