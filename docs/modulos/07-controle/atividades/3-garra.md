# Utilizando a Garra

# Entendendo a utilização da garra
A utilização da garra no nosso robô é bem simples. Para mover o **ombro** publique no tópico `/joint1_position_controller/command` uma mensagem do tipo `std_msgs.Float64`.

* Para colocar o ombro pra cima, publique um valor de `1.5`.
* Para colocar o ombro pra baixo, publique um valor de `-1.0`.
* Para colocar o ombro pra frente, publique um valor de `0.0`.

Para mover a **garra** publique no tópico `/joint2_position_controller/command` uma mensagem do tipo `std_msgs.Float64`.

* Para abrir a garra, publique um valor de `-1.0`.
* Para fechar a garra, publique um valor de `0.0`.

## Módulo da Garra

Vamos encapsular os comandos da garra em uma classe que pode ser facilmente reaproveitada em qualquer nó da ROS 2.

Dentro do pacote `robcomp_util`, crie um arquivo denominado `garra.py` e uma classe chamada `Garra`. Ela é um **mixin**: uma classe que não é um nó, mas acrescenta métodos e atributos a um nó por **herança múltipla**. Essa classe deve:

* Não herdar de `Node` nem iniciar um nó. Importe apenas `Float64` de `std_msgs.msg` e `time`. O construtor é `__init__(self, delay: float = 2.0)`, sem parâmetro `node`.

* Inicialize uma variável `self.delay` com o valor do parâmetro `delay` (padrão `2.0`), que será utilizada para controlar o tempo de espera, em segundos, enquanto a garra se move.

* Definir os publishers `self.ombro_pub` e `self.garra_pub`, criados com `self.create_publisher`, que publicam no tópico `/joint1_position_controller/command` e `/joint2_position_controller/command`, respectivamente. O `self.create_publisher` existe porque `self` é uma instância da classe concreta que herda de `Node` **e** de `Garra`; `Garra` sozinha não é um nó e não tem `create_publisher`.

* Definir um método `controla_garra` que recebe um commando do tipo `str` e executa o movimento da garra de acordo com o comando recebido. O método deve:

    * Se receber o comando **abrir** abre a garra e espera `self.delay` segundos.
    * Se receber o comando **fechar** fecha a garra e espera `self.delay` segundos.
    * Se receber o comando **cima** move o ombro para cima e espera `self.delay` segundos.
    * Se receber o comando **baixo** move o ombro para baixo e espera `self.delay` segundos.
    * Se receber o comando **frente** move o ombro para frente e espera `self.delay` segundos.
    * Para esperar `self.delay` segundos, utilize a função `time.sleep(self.delay)`.
    * Se receber qualquer outro comando, levante um `ValueError` **antes** de publicar qualquer mensagem ou esperar.
    * Publique cada comando uma única vez.

### Usando a Garra em um nó

Para usar a garra, o seu nó herda de `Node` **e** de `Garra`. Como `Garra` não é um nó, ela não faz `spin`: quem executa `rclpy.spin` (ou `spin_once`) é o nó.

```python
from rclpy.node import Node
from robcomp_util.garra import Garra

class MeuNo(Node, Garra):
    def __init__(self):
        Node.__init__(self, 'meu_no')  # 1º: cria o nó
        Garra.__init__(self)           # 2º: cria os publishers da garra (precisa do nó pronto)

    # em algum estado ou callback:
    #     self.controla_garra('abrir')
```

Pontos de atenção:

* **`Node` primeiro** na lista de bases (`class MeuNo(Node, Garra)`): assim os métodos do nó (`get_logger`, `create_timer` etc.) têm prioridade e nenhum deles é mascarado por um método de mesmo nome da `Garra`.
* **Inicialize cada base explicitamente**, `Node.__init__(self, ...)` e depois `Garra.__init__(self)`. Não confie em `super().__init__()` para chamar as duas: ele só executa o `__init__` da próxima classe na ordem de resolução (MRO). A ordem importa porque `Garra.__init__` usa `self.create_publisher`, que só funciona depois que o nó foi inicializado.
* Para mudar o tempo de espera, passe o valor ao mixin: `Garra.__init__(self, delay=1.0)`.

#### Com a Ação do módulo 3

Se a sua ação já herda da classe `Acao` do [Nó Base de Ação](https://insper.github.io/robotica-computacional/modulos/03-laser-e-robo/util/base_action.py), acrescente `Garra` às bases, depois de `Acao`. Inicialize `Acao` antes de `Garra`, pois é o `__init__` da ação que inicializa o nó:

```python
class PegaCreeper(Acao, Garra):
    def __init__(self):
        Acao.__init__(self)   # 1º: inicializa o nó e a ação
        Garra.__init__(self)  # 2º: publishers da garra
        # ... suas variáveis e estados ...
```

Em seguida, chame `self.controla_garra(...)` no estado em que a garra deve se mover.

!!! warning "Atenção"
    No executor de uma única thread usado nos exemplos, `time.sleep` dentro de um callback impede o processamento dos demais callbacks (imagem, laser, timer) durante a espera. Antes de manipular a garra, **publique um `Twist` zerado** para parar o robô; apenas zerar a variável não envia o comando. Mantenha o robô parado durante a manipulação e só volte a andar depois que o comando terminar.

<!-- ## Prática 2
Baseando-se no codigo [Nó Base de Ação](https://insper.github.io/robotica-computacional/modulos/03-laser-e-robo/util/base_action.py) e na sua solução do "Exercício 2 - Identifica Creeper" e no "Desafio" da [APS 6](https://insper.github.io/robotica-computacional/modulos/06-visao-p3/aps/enunciado/), implemente a **Ação de Pegar o Creeper** com o seguindo comportamento:

1. A Ação de Pegar o Creeper deve finalizar após pegar o creeper.
2. Mude o nome da classe para `PegaCreeper` e o nome do nó para `pega_creeper_node`.
3. Mude também a chamada da classe na função `main()` para `PegaCreeper()`.
4. Deve receber a cor e o id do creeper a ser pego como parâmetros na função `reset()`.
3. A ação pode ter quantos estados forem nescessários, mas recomendamos ter pelo menos os estados `procura`, `aproximar`, `finaliza`, `pega`, `stop` e `done`.
    * `procura`: o robô deve procurar o creeper girando no local.
    * `aproximar`: o robô deve se aproximar do creeper, até ficar a uma distância apropriada, centralizando o mesmo na imagem usando controle proporcional.
    * `finaliza`: prepare a garra e assumindo que o creeper está no centro da imagem, pode andar uma distância fixa com base na leitura do laser do front ou continuar se aproximando até utilizando controle proporcional.
    * `pega`: quando creeper esta na garra do robo, feche a garra, levanta o ombro e finalize a ação. -->
