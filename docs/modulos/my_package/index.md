# Gabarito do MyPackage

Para auxiliar na implementação do projeto, criamos um repositório com o gabarito do MyPackage. O repositório pode ser acessado [aqui](https://github.com/insper-classroom/robcomp-util).

## Instruções

1. Mova o seu pacote `my_package` para a pasta `Downloads` - ou qualquer outra pasta que não seja `colcon_ws/src/`, para fazer um backup.

2. Clone o repositório do gabarito do MyPackage na pasta `colcon_ws/src/` do seu SSD.

3. Execute o comando `colcon build` para compilar o pacote.

## MyPackage

Está versão do MyPackage contém os seguintes módulos:

1. `odom.py`: Módulo que implementa a classe `Odom` para lidar com a odometria do robô.

2. `laser.py`: Módulo que implementa a classe `Laser` para lidar com os dados do sensor laser.

3. `module_aruco.py`: Módulo que implementa a identificação de marcadores ArUco (não é um nó ROS).

4. `module_net.py`: Módulo que implementa a identificação utilizando a MobileNet (não é um nó ROS).

5. `atividade3.py`: Atividade 3 do módulo 6, que faz a identificação dos creepers, combinando cor e ID (não é um nó ROS).

6. `creeper_pub.py`: Nó ROS que publica a posição dos creepers na imagem.

7. `creeper_sub.py`: Nó ROS que se inscreve no tópico de creepers e imprime.

8. `rotate2.py`: Nó ROS que faz o robô para uma dada orientação.

9. `run_rotate2.py`: Exemplo de multiplas chamadas ao `rotate2.py`.
