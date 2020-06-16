# Projekt-4-MAPR

STRUKTURA PROJEKTU:

Skrypt odczytujący mapę z pliku .bt, publikujący ją i wyświetlający markery
/projekt_4/src/publikuj_mape.cpp

Odczytywana mapa:
/projekt_4/maps/fr_078_tidyup.bt

Sterowanie dronem i planowanie ruchu
/projekt_4/src/exe/quadrotor_bag_builder_2.cpp

Mode URDF drona:
/projekt_4/urdf/xpp_quadrotor/urdf/quadrotor.urdf
---------------------------------------------------------------------------

WGRYWANIE PROJEKTU DO WORKSPACE CATKIN (Robot Operating System):

Wkleić folder projekt_4 do /catkin/src
cd ..
catkin_make
source devel/setup.bash

#Uruchamianie projektu
roslaunch projekt_4 quadrotor_MOJ.launch
