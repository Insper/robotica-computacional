# Entregável 4 de Robótica Computacional

* Certifique-se que seu repositório está no diretório da ROS, `catkin_ws/src` e que você executou o comando `catkin_make` no diretório `catkin_ws`.

* Depois use o comando `chmod +x catkin_ws/src/aps4/scripts/*.py` para tornar os arquivos `.py` executáveis.

# Exercício 1
Usando o simulador, modifique o arquivo `cor.py` para criar um nó da ROS que faça o seguinte.
1. Gira em malha aberta para procurar um "creeper" azul.

2. Quando o creeper estiver no frame da imagem, deve entrar no estado de centralizar no creeper para que o mesmo se encontre no centro da imagem.

3. Utilize controle proporcional para centralizar o creeper no centro da imagem.

4. Quando o creeper estiver **aproximadamente** no centro da imagem, o robô deve começar a se mover em sua direção.

4. Continue utilizando controle proporcional para centralizar o creeper no centro da imagem enquanto o robô se aproxima.

6. O robô deve parar a **30cm** do creeper sem tocar no creeper.

7. **Grave um vídeo** mostrando o robô realizando todos os objetivos, publique no Youtube e escreva o link no arquivo `README.md` do seu repositório.

**SUGESTÃO** - Use a **Q5 da Atividade 4** como um nó separado para processar a imagem e publicar o centro do creeper no tópico `center_publisher`.

# Exercício 2
Usando o simulador, modifique o arquivo `aleatorio.py` para criar um nó da ROS que faça o seguinte.
1. O robô deve girar em malha aberta a uma taxa de 0.5 rad/s por uma valor aleatório de tempo. **Não utilize rospy.sleep()**. 

2. Escreve no terminal, utilizando `rospy.loginfo`, o tempo alvo que passou e o tempo total, como no exemplo a seguir:
```bash
[INFO] [1677872660.744132, 294.919000]: Estou rodando há 0.638 segundos de 0.710825667 segundos
```

3. Dependo da cor da caixa **mais prevalente** no frame o robô deve fazer uma das seguintes ações.

4. Se a caixa for magenta, o robô deve se aproximar da caixa e parar a **20cm**, depois deve voltar para a posição de origem.

5. Se a caixa for amarela, o robô deve se deslocar para trás, enquanto centraliza na caixa cyano, e parar a **20cm** de algum obstáculo, depois deve voltar para a posição de origem.

6. Se não encontrar nada, rode novamente.

7. **Grave um vídeo** mostrando o robô realizando todos os objetivos, publique no Youtube e escreva o link no arquivo `README.md` do seu repositório.

**SUGESTÃO** - Crie uma maquina de estados para para a caixa magenta e outra para a caixa amarela e teste cada uma delas separadamente. Depois, o use o metodo `update()` para atualizar a maquina de estados de acordo com a cor da caixa, como no exemplo a seguir:

```python
	def checar(self) -> None:
		"""
		Stop the robot
		"""
		if self.selected_color == "magenta":
			# append magenta_machine to robot_machine
			self.robot_machine.update(self.magenta_machine)
			self.robot_state = "aproxima"

		elif self.selected_color == "yellow":
			# append yellow_machine to robot_machine
			self.robot_machine.update(self.yellow_machine)
			self.robot_state = "afasta"
```

Onde, depois de girar em malha aberta, o robô checa a cor da caixa e atualiza a maquina de estados de acordo.

# Exercício 3
Usando o robô real, modifique o arquivo `pista.py` para criar um nó da ROS que faça com que o robô siga as linhas amarelas da pista, utilizando o controle proporcional.

* **Grave um vídeo** mostrando o **robô real** realizando todos os objetivos, publique no Youtube e escreva o link no arquivo `README.md` do seu repositório.

**SUGESTÃO 1** - Primeiramente teste no simulador e depois execute no robô real.

**SUGESTÃO 2** - Siga o desafio da APS 2, centralizando no amarelo de forma similar ao creeper.