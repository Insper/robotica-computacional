# 04 - ROS básico

!!! pdf
    ![](slides-1.pdf)

Neste módulo iremos trabalhar com conceitos e implementações relacionados à robótica utilizando a ROS.

## Atividades
As atividades deste módulo focam em introduzir técnicas úteis para 

1. Entender o conceito de nó, tópico e mensagens da ROS.
2. Criar novos tópicos.
3. Controlar o robô, real e simulado na ROS, utilizando o método "dead-reckoning".
4. Receber e processar mensagens dos tópicos de imagens, laser.

Estes conceitos são explorados nas seguintes atividades,

- [Atividade 1](atividades/atividade1.md) - Utilizando o Simulador do Turtlebot
- [Atividade 2](atividades/atividade2.md) - Explorando o conceito de tópicos e mensagens na ROS
- [Atividade 3](atividades/atividade3.md) - Explorando as ferramentas de visualização da ROS
- [Atividade 4](atividades/atividade4.md) - Lista de exercícios explorando implementação de comunicação entre tópicos, controle do robô e utilização dos tópicos de imagem e laser.
- [Atividade 5](atividades/atividade5.md) - Lista de exercícios expandindo a atividade 4.

[Download das atividades](atividades-modulo04-aluno.zip){ .ah-button }
Extrai os arquivos dentro da diretório `~/catkin_ws/src` e depois executa o comando:
```bash
cd ~/catkin_ws/
catkin_make
```

## Conceitos em Robótica

!!! pdf
    ![](slides-2.pdf) -->

<!-- ## Para entregar

Os arquivos para entrega encontram-se na pasta `APS04` dos repositórios de cada grupo no Classroom. Se essa pasta não aparece, siga o [guia para atualizar os enunciados](../../guias-infra/aps.md#recebendo-atualizacoes-e-novas-aps). Mova o diretório `APS04` para o diretorio `~/catkin_ws/src` e depois executa o comando:
```bash
cd ~/catkin_ws/
catkin_make
``` -->


## Para entregar

!!! exercise
    Clique no link abaixo para ser direcionado para o Github Classroom da APS 4.

    As APSs são em dupla dentro da mesma turma, no link acima você deve escolher seu parceiro e criar um grupo.

    Mova o seu repositório para o diretorio `~/catkin_ws/src` e depois execute o comando:
    ```bash
    cd ~/catkin_ws/
    catkin_make
    ```

    Depois, modifique os arquivos `.py` para executavel com o comando:
    ```bash
    cd roscd aps4/scripts
    chmod +x *.py
    ```

    As entregas da APS 4 são vídeos, siga o [guia de configuração da APS](../../aps/screen_record.md) para saber como fazer a gravação do vídeo no Ubuntu. Depois, de upload do vídeo no Youtube e coloque o link no arquivo `README.md` do seu repositório.

    [APS 4 - Github Classroom](https://classroom.github.com/a/LxO7SjNZ)

    A data final de entrega é **{{ data_APS4 }}**.
