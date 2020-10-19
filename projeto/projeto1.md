
# Projeto 1

Data ideal de entrega: 09/11/2020


```

    October 2020
Su Mo Tu We Th Fr Sa
             1  2  3
 4  5  6  7  8  9 10
11 12 13 14 15 16 17
18 19 20 21 22 23 24
25 26 27 28 29 30 31


   November 2020
Su Mo Tu We Th Fr Sa
 1  2  3  4  5  6  7
 8  9 10 11 12 13 14
15 16 17 18 19 20 21
22 23 24 25 26 27 28
29 30


```



<img src="./pista_virtual.jpg">

Comandos para atualizar os repositórios:

    cd ~catkin_ws/src/mybot_description
    git stash
    git pull
    cd ~catkin_ws/src/my_simulation
    git checkout master
    git pull
    cd ~catkin_ws/src/robot202
    git pull

Para executar:

	roslaunch my_simulation pista_s.launch

Para habilitar o controle da garra executar:

	roslaunch mybot_description mybot_control2.launch 	
	
Para editar:

Sugerimos que crie um projeto próprio e se baseie no seguinte arquivo:

    catkin_ws/src/robot202/ros/projeto1/scripts/base_proj.py


Como atividade inicial, sugiro que tente fazer o robô *seguir a pista* . Você pode se basear em sua Atividade 3, ou ainda desenvolver uma abordagem baseada em centro de massa da linha amarela, como [encontrada neste link](https://github.com/osrf/rosbook/blob/master/followbot/follower_color_filter.py)


