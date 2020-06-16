# Projekt-4-MAPR


## STRUKTURA PROJEKTU:<br/><br/>

### Skrypt odczytujący mapę z pliku .bt, publikujący ją i wyświetlający markery<br/>
/projekt_4/src/publikuj_mape.cpp<br/><br/>

### Odczytywana mapa:<br/><br/>
/projekt_4/maps/fr_078_tidyup.bt  

### Sterowanie dronem i planowanie ruchu:<br/><br/>
/projekt_4/src/exe/quadrotor_bag_builder_2.cpp  

### Model URDF drona: <br/><br/>
/projekt_4/urdf/xpp_quadrotor/urdf/quadrotor.urdf  

## WGRYWANIE PROJEKTU DO WORKSPACE CATKIN (Robot Operating System):<br/><br/> 
### Wkleić folder projekt_4 do /catkin/src\ <br/>
cd ..<br/>
catkin_make<br/>  
source devel/setup.bash<br/><br/>

### Uruchamianie projektu <br/>
roslaunch projekt_4 quadrotor_MOJ.launch<br/>  
