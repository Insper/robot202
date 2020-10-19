
# Projeto 1


<img src="./pista_virtual.jpg">

Comandos para atualizar os repositório:

    cd ~catkin_ws/src/my_bot
    git pull
    cd ~catkin_ws/src/y_simulation
    git pull
    cd ~catkin_ws/src/robot202
    git pull

Para executar:

	roslaunch my_simulation pista_s.launch
	
Para editar:

Sugerimos que crie um projeto próprio e se baseie no seguinte arquivo:

    catkin_ws/src/robot202/ros/projeto1/scripts/base_proj.py


Como atividade inicial, sugiro que tente fazer o robô *seguir a pista* . Você pode se basear em sua Atividade 3, ou ainda desenvolver uma abordagem baseada em centro de massa da linha amarela, como [encontrada neste link](https://github.com/osrf/rosbook/blob/master/followbot/follower_color_filter.py)


