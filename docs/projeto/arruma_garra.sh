# Script gerado por - Licia e Rogério
# Correção do mimics (problema no fechamento assimétrico da garra do robo simulado)
cd ~/catkin_ws/src
rm -fr roboticsgroup_upatras_gazebo_plugins
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
cd ~/catkin_ws
catkin_make
