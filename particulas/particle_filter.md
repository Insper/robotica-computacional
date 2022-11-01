
# Filtro de partículas

Percepção:

*	Mapeamento de leituras dos sensores em representações internas do ambiente

*	Difícil porque sensores têm ruído

*	Ambiente é parcialmente observável

*	Imprevisível

*	Dinâmico


# Variáveis de crença

Belief state - posterior probability over the environment state variables

Estado de crença (belief state) - posterior sobre as variáveis de estado do ambiente

$X_t$ é o estado do mundo - incluindo robô - no instante $t$

$Z_t$ é a observação recebida no instante $t$

$A_t$ é a ação tomada depois que a observação é recebida


# Como representar um estado?
Têm todos os problemas de estimação de estados vistos na seção 15.2

Boas representações internas têm:

* Contém informação suficiente para tomada de boas decisões

* Estruturadas de modo a serem atualizadas de forma eficiente

* São naturais - variáveis corresponde a variáveis de estado presentes no mundo físico

$X_{t}$ é o estado do ambiente (incluindo robô) no instante $t$. $Z_{t}$ é a observação recebida no instante $t$  e $A_t$ é a ação tomada após a observação ser recebida.

# Como incorporar a leitura?

Desejamos calcular o novo estado de crença
$P(X_{t+1}|z_{1:t+1},a_{1:t})$ a partir do estado de crença atual $P(X_t|z_{1:t}, a_{1:t-1})$
e da nova observação
z_{t+1}.

# Para uma leitura de sensores

Modificado (em relação ao Cap. 15)

$$P(X_{t+1}|z_{1:t+1}, a_{1:t}) = \alpha P(z_{t+1}|X_{t+1}) \int P(X_{t+1}|x_{t},a_{t}) P(x_{t} | z_{1:t}, a_{1:t-1})dx_t $$


Temos que:

$P(X_{t+1}|x_t, a_t)$  é o modelo de transição ou de movimento

e $P(z_{t+1}|X_{t+1})$ é o modelo sensorial

## Localização e mapeamento

Vamos assumir um robô que se move no plano

O estado do robô é definido por sua posição

$$ X_{t} = \left( x_t, y_t, \theta_t \right)^T$$

# Movimento do robô

Modelo simplificado de movimento do robô, previsão determinística:

$$
\widehat{X_{t+1}} = f(X_t, v_t, \omega_{t}) = X_t + 
\left(
\begin{array}{c}
v_t \Delta_t cos \theta_t \\
v_t \Delta_t sen \theta_t \\
\omega_t \Delta_t
\end{array}
\right)
$$

$\widehat{X}$ é uma previsão determinística de estado. Robôs reais são um pouco imprevisíveis.

# Leituras

Esta incerteza é modelada com média $\widehat{X}$ e covariância $\Sigma_{x}$.


$$
P(X_{t+1}| X_t, v_t, \omega_t) = N(\widehat{X}_{t+1}, | \Sigma_{x})
$$

Precisamos também de um modelo de sensores. Consideraremos dois tipos de sensor.

O primeiro assume que os sensores detectam features estáveis e reconhecíveis do ambiente chamadas landmarks.

Para cada landmark, o alcançe e a direção são reportados.



Suponha que o robô esteja em:

$$
x_t = \left(  x_t, y_t, \omega_t \right)^T 
$$

E ele detecte um landmark cuja localização é sabida ser

$$( x_i, y_i)^T$$

Sem ruído, podemos calcular a distância e direção usando geometria simples

$$
\hat{z}_t = h(x_t) = \left(
	\begin{array}{c}
	\sqrt { (x_t - x_i)^2 + (y_t - y_i)^2}
	\\
	arctan \frac{y_i - y_t}{x_i - x_t} - omega_t
	\end{array}
\right)
$$

O ruído distorce as medidas. Assumamos ruído gaussiano com covariância $\Sigma_z$ que nos leva ao modelo de sensores:

$$
P(z_t|x_t) = N(\widehat{z}_t, \Sigma_z)
$$

No caso de um array de sensores laser, cada qual tem uma direção fixa em relação ao robô, temos que as leituras são:

$$
z_t = (z_1, z_2, ..., z_M)^T
$$

Dada uma pose do robô $x_t$, seja $\hat{z}_j$ a distância exata ao longo da direção $j$ de $x_t$ ao obstáculo mais próximo. 

Esta leitura será corrompida por ruído gaussiano, tipicamente assumimos os erros em diferentes direções como i.i.d, de modo que temos:

$$
P(z_t|x_t) = \alpha \prod_{j=1}^M e^{\frac{-(z_j - \hat{z}_j)}{2\sigma^2}}
$$

# Para um raio

Lembrando que a normal é:

$$
f(x) = \frac{1}{\sqrt{2\pi}\sigma}e^{\frac{-(x - \mu)^2}{2 \sigma^2}}
$$

# Algoritmo

Passos de um filtro de partículas

1. Crie uma função que gera n partículas aleatórias distribuídas uniformemente dentro de uma área `minx`, `miny`, `maxx`, `maxy`, `n_particulas`

2. Crie uma função que aplica o deslocamento pelo qual o robô padrão passa a todas as partículas.

3. Calcule $P(D|H)$ . Programe a aplicação da equação <font color=red>modificada</font> baseada na que está na pág. 853 do livro do Norvig $$P(z_t | x_t) = \alpha \sum_{j}^M{e^{\frac{-(z_j - \hat{z_j})^2}{2\sigma^2}}}$$

Para cada raio real medido do robô $\hat{z_j}$ e leitura simulada da partícula $z_j$ calcule a probabilidade daquela leitura da partícula ter acontecido se o robô estivesse na posição em que a partícula está

4. Reamostre as partículas de forma proporcional à probabilidade


# Atividade guiada 

Realizaremos a atividade que está em  [https://github.com/Insper/404/blob/master/tutoriais/robotica/navigation_gazebo_simulador.md](https://github.com/Insper/404/blob/master/tutoriais/robotica/navigation_gazebo_simulador.md)


Vídeo com demonstração do Gmapping

[https://www.youtube.com/watch?v=yv0FhmqPfUo&t=312s](https://www.youtube.com/watch?v=yv0FhmqPfUo&t=312s)



# Atividade autônoma 

Realize você o mapeamento e o controle de trajetória do robô num mapa diferente.

Launch file:

    roslaunch my_simulation quarto_andar.launch
