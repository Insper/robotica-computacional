# Instalando OpenCV no IPython notebook

Precisamos usar Python 2 este é o Python mais suportado e testado na versão do ROS que usaremos. Assim fica mais fácil aproveitar código feito agora no robô.


Abra um terminal, e veja se já não tem um ambiente com Python 2.

    conda env list

Para saber qual é o ambiente ativo atualmente dê o comando:

    conda info --envs

A saída deverá ser algo assim:

    # conda environments:
    #
                            //anaconda/envs/Python27
    base                  *  /anaconda3/anaconda3


O asterisco marca o ambiente que está sendo usando

Para ativar outro ambiente:

    conda activate **nome do ambiente**


Para criar um environment faça:

    conda create -n robotica

Agora, quando listamos os ambientes, deve aparecer o **robotica** que foi criado:

    conda env list

A saída vai ficar assim:

    # conda environments:
    #
                            //anaconda/envs/Python27
    base                  *  /anaconda3/anaconda3
    robotica                 /anaconda3/anaconda3/envs/robotica


Depois, para mudar para o novo ambiente, faça:

    conda activate robotica

No que ao invés de **robotica** você pode usar o nome que quiser.

Para instalar a OpenCV e Jupyter

    conda install  -c conda-forge opencv jupyterlab  jupyter

Você também vai precisar instalar os softwares básicos para trabalhar:

    conda install matplotlib
    

Para sair do ambiente **robotica** e voltar para seu 