# Criando um novo Pacote na ROS 2

Agora, vamos aprender a criar um novo pacote na ROS 2. Abra um terminal (`Ctrl+Alt+T`) e execute os comandos abaixo:

```bash
cd ~/colcon_ws/src
ros2 pkg create --build-type ament_python my_package --dependencies rclpy std_msgs geometry_msgs
```

Aqui enviamos dois comandos no terminal. O primeiro muda o diretório atual para a pasta `src` do workspace `colcon_ws` - é nela que **todos** os pacotes da ROS 2 são criados.

O segundo comando cria o pacote `my_package` dentro de `src`. Vamos entender cada parte:

* `ros2` - interface de linha de comando da ROS 2.
* `pkg create` - subcomando para **criar** um novo pacote.
* `--build-type ament_python` - define que o pacote será em **Python** (layout e build próprios do `ament_python`).
* `my_package` - nome do pacote a ser criado.
* `--dependencies rclpy std_msgs geometry_msgs` - dependências declaradas no `package.xml` (e referenciadas no setup), como `rclpy`, `std_msgs` e `geometry_msgs`.

Ao final, será gerada a estrutura básica do pacote (por exemplo, `package.xml`, `setup.py`/`setup.cfg`, diretório `my_package/` com `__init__.py`, e `resource/`).

Na ROS 2, após criar um pacote, é necessário **"compilá‑lo"** para que possa ser usado. Para compilar os pacotes do workspace, execute:

```bash
cd ~/colcon_ws
colcon build
```

!!! dica
    Depois de compilar, **atualize o ambiente** no terminal atual (ou abra um novo terminal) para que o sistema reconheça o pacote:

    ```bash
    source ~/colcon_ws/install/setup.bash
    ```

    Você pode verificar se o pacote foi encontrado com:

    ```bash
    ros2 pkg list | grep my_package
    ```

Muito bem! Com o pacote criado e compilado, siga para a próxima atividade para aprender a criar um **nó** na ROS 2.
