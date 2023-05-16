# Atualizações do projeto

Vamos colocar aqui mudanças ou avisos importantes para o projeto. Cada um deles tem um item "Marcar como feito" para que vocês possam controlar se cada um foi feito corretamente. 

Todos eles devem ser feitos por todos elementos do grupo, já que estão relacionados à infra local de cada um.

!!! exercise
    Mudamos o primeiro AruCo de lugar, para depois da primeira encruzilhada. Execute os comandos abaixo para puxar a nova versão. 

        cd ~/catkin_ws/src/mybot_description
        git pull
        cd ~/catkin_ws/src/my_simulation
        git pull


!!! exercise 
    Precisamos fazer uma mudança na infra para a garra funcionar no robô simulado. Baixe o script abaixo e coloque-o na sua pasta home (`/home/borg`)

    [Baixar script](arruma_garra.sh)

    Então execute abaixo

        cd 
        chmod +x ~/arruma_garra.sh
        ./arruma_garra.sh

