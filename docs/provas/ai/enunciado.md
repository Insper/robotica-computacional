# Robótica Computacional 2026.2 - AI

Instruções para a avaliação:

* A prova tem duração de **4 horas**.
* Inicie a prova no Blackboard para a ferramenta do Smowl ser iniciada. 
* O Smowl é obrigatório durante toda a prova.
* Só finalize o Blackboard quando enviar a prova via o repositório informado no Blackboard, incluindo o hash do último commit na resposta do Blackboard.
* Durante a prova vamos registrar, a camera, a tela, as páginas visitadas, os acessos online e os registro do teclado.
* Coloque seu `nome` e `email` no `README.md` do seu repositório.
* A prova deverá ser realizada de forma individual.
* Não é permitido consultar a internet, com exceção do site da disciplina, do `Blackboard` e do repositório da avaliação informado no Blackboard.
* `Não é permitido o uso de ferramentas de **IA** como ChatGPT, Copilot, Gemini ou similares durante a prova`.
* `Não é permitido o uso de ferramentas colaborativas como Google Docs, Google Slides, Notion, ou similares durante a prova`.
* `Não é permitido o uso de ferramentas de comunicação como Discord, WhatsApp, Telegram ou similares durante a prova`.
* `Não é permitido o uso de editores de codigo com IA como Cursor ou Windsurf durante a prova, sendo permitido apenas o uso do **VSCode**`.
* `Não é permitido o uso do Copilot durante a prova. Então desative-o antes de iniciar a prova`.
* `Não é permitido o uso de redes sociais, fóruns ou plataformas de comunicação durante a prova`.
* Só é permitido acessar o repositório da avaliação informado no Blackboard, o site da disciplina e o Blackboard.
* Só é permitido acessar documentos e arquivos estáticos em PDF, TXT, MD, DOCX, DOC utilizando ferramentas offline.
* Não é permitido push e pull de repositórios que não sejam o da avaliação informado no Blackboard.
* Faça commits e pushes regularmente de sua avaliação.
* Eventuais avisos importantes serão realizados em sala durante a prova.
* Escreva a frase "yey" como a resposta da soma no arquivo `README.md` como teste de sua atenção.
* A responsabilidade por *`infraestrutura`*, *`configurações`* e *`setup`* em funcionamento pleno, é de cada estudante.
* **SÓ SERÃO ACEITOS REPOSITÓRIOS DE ALUNOS QUE ASSINARAM A LISTA DE PRESENÇA.**

* **BOA PROVA!**

## Atualização do Pacote (ROS 2)

Execute os comandos abaixo para atualizar os pacotes da `ros2` obrigatórios para a prova:

```bash
cd ~/colcon_ws/src/my_simulation
git add .
git stash
git checkout main
git pull
cb
```

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** O repositório é informado no Blackboard (veja [Sobre as APS](https://insper.github.io/robotica-computacional/aps/)). Clone-o **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote chamado `avaliacao_ai`. TODO: Ajustar para o novo metodo

    - **Dica:** Para utilizar os módulos desenvolvidos no capitulo 3, inclua o pacote `robcomp_util` e o pacote `robcomp_interfaces` como dependência do seu pacote, e então, importe como nos exemplos do capitulo 3.

---

# Exercício 0 (Gate: Falha Limita a nota para 2.0)- Organização & Qualidade
Este exercício avalia a organização e a qualidade dos vídeos dos exercícios e do arquivo `README.md`.

## Critérios de Avaliação:
* O pacote foi corretamente configurado.
* As dependências do pacote estão corretas.
* Os diretórios e arquivos estão organizados de forma adequada.
* Todos os scripts estão na pasta `avaliacao_ai` dentro do pacote `avaliacao_ai`.
* A configuração dos nós foi realizada corretamente.
* Os nós da ROS 2 foram executados utilizando o comando `ros2 run`.
* **Vídeo (Se houver):** A ação do robô é claramente compreensível pelo vídeo.
* **README.md:** O link do vídeo, se houver, foi adicionado corretamente no campo indicado.
* **README.md:** O arquivo `README.md` contém o nome completo e o e-mail do estudante.
---

# Exercício 1 (ROS 2) - Segue Comando

Robo recebe de um tipico, com um tipo de msangem e envia para outro topico, com outro tipo de mensagem.
recebe comandos do tipo antes 2m, girar 180o graos, fazer um circulo, fazer um quadrado, fazer um triangulo.

Quem envia comando, (Comandador?) pode dizer para o robo parar, em qualquer momento, onde ele devera parar imaediantamente e perguntar quanto falta para ele terminar (quantos metros do andar, quandos graos do girar). O robo devera responder com a quantidade de metros, graus ou lados que faltam para terminar a tarefa. Depois de fazer uma das 3 formas, o Comandador vai pedir o erro do robo, desdo começo da ação até o fim.

O Comandadador sempre pede pra girar e depois andar ou andar e girar, sempre interropendo depois de # segundos. E depois pede um dentre os 3 movimentos de forma aleatoria, no fim pedindo o erro do robo, que deve ser calculado e enviado para o Comandador.


---

# Exercício 2 (OpenCV) - Contador de Batatas

Deve ter uma classe chamada ProcessoImage com uma função chamada roda video, que recebe um video. A classe deve abrir o video, que vai ter um fundo verde onde batatas de cor marron passam aleatoriamente. A função deve contar quantas batatas aparecem no video e retornar o valor e desenhar na imagem a trajedoria da batata, que é sempre linear.

Dica com como rodar um video com OpenCV:

```python
...
```
