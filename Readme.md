Controle de Tanget Bug
Este pacote ROS contém um nó que implementa Tangent Bug para um robô móvel. O código utiliza o ROS (Robot Operating System) e é projetado para trabalhar com mensagens padrão do ROS, como nav_msgs/Odometry e geometry_msgs/Twist.

Pré-requisitos
Este pacote foi desenvolvido e testado em um ambiente ROS. Certifique-se de ter o ROS instalado em seu sistema antes de tentar executar este código.

Dependências
Além do ROS, este pacote depende das seguintes bibliotecas e pacotes:

ROS (nav_msgs, geometry_msgs, tf)
Eigen (Biblioteca para álgebra linear)
Instalação
Para instalar este pacote, siga estes passos:

Clone o repositório para a pasta "src" do seu espaço de trabalho do ROS.
Compile o pacote usando o comando catkin_make na raiz do seu espaço de trabalho.
Parâmetros

O nó de controle de trajetória aceita os seguintes parâmetros:

"a": Metade da largura da lemniscata
"kp": Parâmetro de ganho proporcional para o controlador PID
"kp1": Parâmetro de ganho proporcional 1 para o controlador PID
"kp2": Parâmetro de ganho proporcional 2 para o controlador PID
"d": Parâmetro de largura da lemniscata
"x_goal": Parametro de objetivo de posição X do robo
"y_goal": Parametro de objetivo de posição Y do robo
"d": Parametro de controle de Feedback Linearization
"stage_ros" type="stageros" name="stageros" args="$(find catkinha_fc_tangent)/ecai21_1.world" />
"catkinha_fc_tangent" type="catkinha_fc_tangent_node" name="catkinha_fc_tangent_node" output="screen"/>

