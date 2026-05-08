# Projeto - SLAM e Navegação

Neste projeto, grupos de até **4 alunos** deverão desenvolver um sistema de navegação para que o robô real seja capaz de se localizar no mapa, planejar trajetórias e sair de um labirinto desconhecido. O projeto está dividido em duas etapas.

Na **Etapa 1**, a equipe vai estudar o conceito de navegação na ROS 2 e fazer com que o robô real saia de um labirinto conhecido. Para isso, será necessário compreender a relação entre a odometria do robô e as coordenadas do mapa fornecidas pelo Cartographer, além de adaptar o `GOTO` para utilizar as coordenadas globais estimadas pelo nó AMCL.

Na **Etapa 2**, a equipe deverá implementar uma solução de SLAM e planejamento em grid. O sistema deve construir ou atualizar o mapa, gerar trajetórias automaticamente e fazer com que o robô complete um labirinto desconhecido de forma autônoma.

Uma etapa será considerada completa quando:

1. Existir pelo menos um vídeo demonstrando claramente o funcionamento dos itens principais da rubrica. Vídeos intermediários também podem ser enviados, desde que sejam descritos no relatório.
2. O grupo enviar, no próprio GitHub, um relatório explicando como os problemas foram abordados e resolvidos. O formato esperado do relatório está descrito abaixo.

## Repositório

Link do GitHub Classroom [TODO](TODO)

## Execução

O projeto pode ser desenvolvido e testado no simulador. No entanto, **apenas execuções no robô real serão consideradas para nota**.

Utilize o comando abaixo para gerar um mapa com um labirinto:

```bash
ros2 launch my_gazebo labirinto.launch.py
```

## Relatório

O relatório deve ser escrito como um **relatório técnico curto**, em texto contínuo, mas dividido em seções claras, como introdução, metodologia, resultados e dificuldades. O objetivo não é apenas listar códigos ou funções, mas explicar o que foi feito, por que foi feito e como cada problema da rubrica foi resolvido.

Para cada etapa do projeto, o grupo deve incluir pequenos parágrafos explicando a solução adotada, imagens do mapa e das trajetórias planejadas quando fizer sentido, além de evidências de funcionamento no robô real. As demonstrações em vídeo devem ser incluídas por meio de links, acompanhadas de uma breve explicação do que cada vídeo mostra.

Caso o grupo utilize algum método, algoritmo ou técnica que não foi apresentado em sala de aula, como A*, variações de planejamento, suavização de caminho ou campo de custo, o relatório deve explicar brevemente o funcionamento dessa escolha.

# Rubrica - Projeto SLAM e Navegação

Todas as rubricas: o relatório deve descrever o que foi feito e explicar como os principais problemas de cada item foram resolvidos. O relatório também deve explicar o funcionamento de qualquer método ou algoritmo que não foi apresentado em sala de aula.

## Etapa 1 - Navegação em mapa conhecido (4)

### 1. Localização global no mapa (0.5)
- Roda o nó de localização/AMCL.
- Consegue obter `x`, `y` e `yaw` do robô no frame `map`.

### 2. Conversão entre imagem/grid e coordenadas do mapa (0.5)
- Em um script auxiliar:
    - Implementou uma função para conversão de coordenadas da imagem/grid para coordenadas reais do mapa.
    - Implementou uma função para a conversão inversa: mapa para imagem/grid.
- Considere `resolution`, `origin`, largura, altura e possivel inversão do eixo `y`.

### 3. Navegação ponto a ponto com `GOTO` (1.0)
- Adapta o `GOTO` para usar AMCL no lugar do ODOM.
- O robô consegue navegar de um ponto até outro no mapa conhecido.

### 4. Completar o labirinto conhecido (2.0)
- O grupo define uma sequência válida de pontos para fazer o robô sair do labirinto conhecido.
- O robô executa a sequência usando `GOTO`.
- O robô consegue sair do labirinto conhecido de forma consistente.

---

## Etapa 2 - SLAM, planejamento e labirinto desconhecido (6)

### 1. Leitura e visualização do mapa (0.5)
- Implementa subscriber para o mapa.
- Converte o mapa recebido em uma estrutura útil para planejamento.
- Desenha o mapa usando `matplotlib`, `cv2` ou ferramenta equivalente.

### 2. Planejador em grid (0.75)
- Implementa A* ou algoritmo similar.
- Considera paredes/obstáculos como regiões não navegáveis.
- Gera caminho entre a posição atual do robô e um ponto final hipotético, longe da posição atual do robô.
- Desenha a trajetória planejada sobre a imagem do mapa.

### 3. Tratamento de região conhecida (0.25)
- Evita planejar por áreas desconhecidas.
- Corta ou restringe o caminho para permanecer dentro da região conhecida.
- Desenha a trajetória navegável com outra cor no mapa, sem incluir pontos desconhecidos.

### 4. Campo de custo em relação às paredes (0.5)
- Gera um campo de custo ao redor das paredes.
- Faz o planejador preferir caminhos afastados dos obstáculos.
- O planejador prioriza a navegação pelo centro dos corredores.

### 5. Otimização e execução parcial do caminho (1.0)
- Reduz ou suaviza pontos redundantes do caminho.
- O caminho é simples e contém apenas pontos significativos, sem redundância repetitiva.
- O robô consegue executar pelo menos uma trajetória planejada.

### 6. Completar o labirinto desconhecido de forma autônoma (3.0)
- O robô atualiza o mapa enquanto explora.
- O sistema chama o planejador múltiplas vezes conforme executa as trajetórias planejadas.
- O robô consegue completar um labirinto desconhecido sem sequência manual de pontos e sem colidir com as paredes.

