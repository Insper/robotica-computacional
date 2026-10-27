# Projeto - Missões de Navegação e Resgate

Neste projeto, grupos de até **4 alunos** deverão programar o robô para conquistar missões de dificuldade crescente, combinando segue-linha, navegação, visão computacional e controle da garra.

O cenário tem uma **região inicial**, uma linha até a **entrada do labirinto** e outra linha que liga a **saída do labirinto** à região inicial. Dentro do labirinto, o robô deverá navegar e procurar os alvos de cada missão.

O percurso completo é:

**Região inicial → linha de ida → entrada → labirinto → saída → linha de volta → região inicial.**

## Simulação para desenvolvimento

Execute os comandos abaixo para atualizar os pacotes da simulação:

```bash
cd ~/colcon_ws/src/my_simulation
git add .
git stash
git checkout main
git pull
cb
```

Para iniciar a simulação do projeto:

```bash
ros2 launch my_gazebo pista-26B.launch.py
```

## Repositório e entrega

**Prazo de entrega: até as 23h59 do primeiro dia do período de provas finais, conforme o calendário acadêmico.**

O repositório do projeto é informado no Blackboard. Siga as instruções de [Sobre as APS](../aps.md) para configurar o repositório e entregar o link e o SHA completo do commit que deve ser avaliado.

O repositório deve conter:

- Nome completo dos integrantes no `README.md`;
- Código e instruções para executar as missões;
- Relatório técnico curto;
- Links dos vídeos, identificando as missões demonstradas.

Os vídeos devem ser publicados no YouTube, com acesso disponível para a correção. **Não adicione os arquivos de vídeo ao Git**, apenas os links.

## Execução e comprovação

Todas as missões deverão ser comprovadas **no robô real**.

O grupo pode escolher quais missões demonstrar separadamente e quais comprovar em uma execução integrada. Um mesmo vídeo pode comprovar várias missões, desde que cada conquista fique clara.

Para comprovar separadamente as missões intermediárias do labirinto desconhecido (5 a 8), é necessário também completar e comprovar a missão 4. **A missão 9, por outro lado, pode ser demonstrada sem a demonstração da missão 4**.

Nas demonstrações desacopladas, o robô pode começar na posição e nas condições indicadas na missão. O relatório deve informar essas condições. Após a preparação e o início da demonstração, a execução deve ser autônoma.

As **missões 4 e 9 avaliam a integração**: cada uma deve ser demonstrada em um vídeo contínuo, mostrando o percurso completo, sem cortes, reposicionamento do robô, comandos manuais ou reinicialização dos nós entre os trechos. Depois de iniciado, o robô deve realizar a missão autonomamente.

Entregar o creeper significa **transportá-lo, soltá-lo dentro da região inicial e parar o robô nessa região com a garra abaixada e fechada**.

## Missões e pontuação

### Parte 1 - Labirinto conhecido (4 pontos)

Nesta parte, o grupo pode mapear previamente o labirinto e conhecer a posição do creeper, além de definir os pontos usados na navegação.

| Missão | Conquista | Pontos |
| --- | --- | ---: |
| 1 - Atravessar o labirinto | Partindo da entrada, navegar pelo labirinto conhecido e chegar à saída. | 1,0 |
| 2 - Buscar o creeper | Partindo da entrada, chegar ao creeper em posição conhecida, pegá-lo com a garra e transportá-lo até a saída. | 1,0 |
| 3 - Seguir as linhas | Seguir a linha da região inicial até a entrada; em outra demonstração, se necessário, partir da saída, seguir a linha de volta e parar dentro da região inicial. | 1,0 |
| 4 - Expedição conhecida | Em uma execução contínua, partir da região inicial, seguir a linha de ida, entrar no labirinto, buscar o creeper, sair pela saída, seguir a linha de volta e entregá-lo na região inicial. | **1,0** |

### Parte 2 - Labirinto desconhecido (6 pontos)

Nesta parte, o robô deve construir o mapa durante a exploração e decidir autonomamente por onde navegar. **Cada demonstração deve começar sem um mapa prévio do labirinto**, inclusive na missão 8. O mapa e as posições do creeper e do AprilTag não podem ser fornecidos previamente ao programa, nem substituídos por uma sequência de pontos preparada para aquele labirinto. Antes de cada demonstração, peça ao professor ou aos técnicos para posicionar o AprilTag e/ou o creeper, conforme a missão.

| Missão | Conquista | Pontos |
| --- | --- | ---: |
| 5 - Explorar e encontrar a saída | Partindo da entrada, explorar e mapear autonomamente o labirinto até encontrar e alcançar a saída. | 1,0 |
| 6 - Encontrar o creeper | Partindo da entrada, procurar o creeper em posição desconhecida, aproximar-se dele e pegá-lo com a garra. | 1,0 |
| 7 - Encontrar o AprilTag | Partindo da entrada, procurar o AprilTag escondido e registrar seu ID e sua posição estimada no mapa. | 1,0 |
| 8 - Resgatar e retornar | Partindo de dentro do labirinto com o creeper já coletado e sem mapa prévio, explorar e mapear autonomamente até encontrar a saída, seguir a linha de volta e entregá-lo na região inicial. | 1,0 |
| 9 - Expedição completa | Em uma execução contínua, partir da região inicial, seguir a linha de ida, explorar o labirinto desconhecido, identificar e localizar o AprilTag, pegar o creeper, sair pela saída, seguir a linha de volta e entregá-lo na região inicial. Os dois alvos podem ser encontrados em qualquer ordem. | **2,0** |

## Relatório

Escreva um **relatório técnico curto**, em texto contínuo e dividido em seções como metodologia, resultados e dificuldades. Ele deve explicar como o sistema funciona e como as missões foram resolvidas.

Descreva:

- Como os nós e os comportamentos do robô se organizam e como ocorrem as transições entre segue-linha, navegação, busca e coleta;
- Como o robô se localiza, representa o mapa, escolhe destinos, planeja trajetórias e evita obstáculos e áreas ainda desconhecidas;
- Como o creeper e o AprilTag são detectados e como a coleta e a entrega são realizadas;
- Quais missões foram conquistadas, quais condições iniciais foram usadas nos vídeos e quais dificuldades ou limitações permanecem.

Vocês podem usar diagramas para complementar o texto, ilustrando a organização dos nós, os fluxos de execução e as transições entre comportamentos. Inclua imagens dos mapas e das trajetórias quando ajudarem a explicar a solução. Associe cada missão aos links dos vídeos e aos arquivos ou classes principais que a implementam.

**Todo método, algoritmo ou técnica utilizado que não tenha sido apresentado em aula deve ser explicado no relatório.** Por exemplo: A*, exploração por fronteiras, campo de custo ou suavização de trajetórias. Explique a ideia do método, por que ele foi escolhido e como foi aplicado no código do grupo.

## Uso de IA

**O uso de IA é livre em todas as partes do projeto**, incluindo planejamento, programação, testes e redação do relatório. O grupo deve conferir se o relatório corresponde ao código entregue e ao funcionamento demonstrado nos vídeos.

**O relatório será avaliado por IA**, verificando sua concordância com o código do repositório e a coerência lógica do funcionamento descrito. Ele deve também servir como **material de estudo para a prova**, explicada a seguir.

**Cada integrante deve compreender e conseguir explicar a solução entregue**, incluindo os trechos produzidos com auxílio de IA e as técnicas que não foram apresentadas em aula.

## Prova

Parte das questões da prova será **gerada com auxílio de IA a partir do relatório e do repositório do grupo**. Essas questões poderão abordar o funcionamento do código, os métodos utilizados, as decisões tomadas e as limitações da solução.
