
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

# Rubricas

*Conceito C *

Robô é capaz de percorer toda a pista e se chocar contra um creeper da cor certa, voltando depois à pista 

*Conceito B* 

Pega o creeper da cor e ID corretos com a garra e volta para a pista.  

O código tem que estar bem modularizado 

*Conceito A**

Itens do conceito B + um uso de classes e objetos Python    

Só pode ter sleep dentro do `while` principal. 

Pegar o creeper da cor certa, com o ID certo, e deixar na base certa 

Fazer um dentre os três:
* Gravar e filmar no robô real funcionado
* Fazer um controle Proporcional ou PD para manter o robô na pista e fazer funcionar rápido 
* Usar ARUCO em modo 3D 



# Exemplo do ARUCO 

Exemplo de como programar usando os markers ARUCO 

[https://github.com/Insper/robot202/blob/master/ros/exemplos202/scripts/aruco.py](https://github.com/Insper/robot202/blob/master/ros/exemplos202/scripts/aruco.py)

# Intruções sobre os tópicos da garra 

[Fonte: https://github.com/arnaldojr/mybot_description/](https://github.com/arnaldojr/mybot_description/)

Launch para subir os controles da garra e RViz

    roslaunch mybot_description mybot_control2.launch 

Para publicar um topico da garra:

Joint1 = braço da garra. Valores máximos:

    Garra recolhida: -1
    Garra para frente: 0
    Garra levantada: 1.5
    
    No terminal:
    rostopic pub -1 /joint1_position_controller/command std_msgs/Float64 "data: 0"
    
Joint2 = Pinça da garra.

    Pinça fechada: 0
    Pinça aberta: -1
    
    No terminal:
    rostopic pub -1 /joint2_position_controller/command std_msgs/Float64 "data: 0"
    
Visualizar arvore:

    rosrun rqt_gui rqt_gui 
    
Exemple de codigo py

[https://github.com/Insper/robot202/blob/master/ros/exemplos202/scripts/move_garra.py](https://github.com/Insper/robot202/blob/master/ros/exemplos202/scripts/move_garra.py)


