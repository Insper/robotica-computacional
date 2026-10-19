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

Vamos encapsular a os comandos da garra em uma classe que pode ser facilmente importado em qualquer nó na ROS 2.

Dentro do pacote `robcomp_util`, crie um arquivo denominado `garra.py` e uma classe chamada `Garra` sem herança. Essa classe deve:

* Não herdar de `Node` nem iniciar um nó. Em vez disso, receba o nó que a utilizará no construtor, `__init__(self, node: Node, delay: float = 2.0)` (composição). Importe `Node` de `rclpy.node` e `Float64` de `std_msgs.msg`.

* Inicialize uma variável `self.delay` com o valor do parâmetro `delay` (padrão `2.0`), que será utilizada para controlar o tempo de espera, em segundos, enquanto a garra se move.

* Definir os publishers `self.ombro_pub` e `self.garra_pub`, criados com `node.create_publisher`, que publicam no tópico `/joint1_position_controller/command` e `/joint2_position_controller/command`, respectivamente.

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

Como `Garra` não é um nó, ela não faz `spin`: quem executa `rclpy.spin` (ou `spin_once`) é o nó que a hospeda. Dentro de um nó que já existe, passe o próprio nó (`self`) ao construtor:

```python
from robcomp_util.garra import Garra

class MeuNo(Node):
    def __init__(self):
        super().__init__('meu_no')
        self.garra = Garra(self)  # usa este nó; não cria outro

    # em algum estado ou callback:
    #     self.garra.controla_garra('abrir')
```

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
