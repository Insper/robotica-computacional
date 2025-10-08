# Segue Linha

Agora que aprendemos o conceito de controle proporcional, vamos implementar a **Ação Segue Linha** para que o robô siga a linha amarela do controle de forma suave e precisa.

## Conceito

Relembrando o controle proporcional: a saída é ajustada **proporcionalmente ao erro**.

```python
rot = K_p * erro
```

* `rot` é a velocidade angular do robô (ex.: `Twist.angular.z`),
* `K_p` é a constante proporcional,
* `erro` erro da medição.

### Como calcular o erro para seguir a linha:
Para andar em direção a um alvo, o alvo deve estar **centralizado** na horizontal na imagem, ou seja, o **x do centróide do alvo** deve ser igual ao **x do centro da imagem**. Com isso em mente o erro é a distância horizontal (x) entre o centro da imagem e o centróide do segmento amarelo.

### Como calcular na prática:
No `image_callback`, após detectar o centróide do segmento amarelo:

* Guarde `self.cx_linha` (x do centróide) e **`self.w` = metade da largura da imagem**.
* Calcule o erro:

  * `self.erro = self.cx_linha - self.w` (em pixels)
  * Se a linha amarela não 
* Então:

  * `erro_px = self.cx_linha - self.w` (em pixels)
  * *Opcional (recomendado):* `erro = erro_px / self.w` (normalizado ≈ `[-1, 1]`)

Se **não houver linha**, defina `self.cx_linha = -1` e `self.cy = -1` e trate o erro como ausente (ex.: `erro = None`).

## Prática

Implemente a **Ação Segue Linha** a partir do `action_base`, **incorporando** o `vision_sub_base`, seguindo os passos:

1. **Estados da Ação**
   Implemente três estados: `centraliza`, `segue` e `para`.

   * `centraliza`: alinha o robô com o segmento amarelo mais relevante.
   * `segue`: faz o robô avançar seguindo a linha.
   * `para`: após completar **uma volta** na pista, zera as velocidades e encerra a ação.

2. **Assinatura da câmera (imagem comprimida)**
   Use o `vision_sub_base` para adicionar um **subscriber** ao tópico de imagem **comprimida**, chamando `image_callback`.
   Garanta que o `image_callback` **só execute** se `self.running == True`.

3. **`image_callback` (detecção + erro já no callback)**

   * **Filtre o amarelo** (ex.: HSV com máscara + morfologia).
   * Selecione o **segmento mais próximo/relevante** e calcule o centróide.
   * Preencha:

     * `self.cx_linha` e `self.cy`
     * `self.w` = metade da largura da imagem (pode salvar uma única vez se fixo)
   * **Calcule o erro aqui**:

     ```python
     if self.running:
         if self.cx_linha != -1:
             erro_px = self.cx_linha - self.w
             self.erro = erro_px / self.w      # normalizado (opcional)
         else:
             self.erro = None
     ```
   * Se **não houver contorno**, defina `self.cx_linha = -1` e `self.cy = -1`.

4. **Parâmetros e inicialização**
   No `__init__`:

   * Defina `self.kp` (ganho proporcional), `self.v_const` (velocidade linear), `self.w_max` (limite de giro) e `self.running = True`.
   * Armazene a **posição inicial** do robô (ex.: odometria/TF) em `self.pose_inicial` para detectar o fechamento da volta.

5. **Lógica do estado `centraliza`**

   * Se `self.erro is None` (linha não vista), **gire lentamente** para procurar (ex.: `v = 0.0`, `w = w_busca`).
   * Se houver erro, use controle proporcional apenas no giro (ex.: `v` pequeno ou `0.0`, `w = clip(-self.kp * self.erro, ±self.w_max)`).
   * Ao **reduzir |erro| abaixo de um limiar** (ex.: `0.05`), transicione para `segue`.

6. **Lógica do estado `segue`**

   * Use **velocidade linear constante**: `v = self.v_const`.
   * Controle o giro com P: `w = clip(-self.kp * self.erro, ±self.w_max)`.
   * Se `self.erro is None` por algumas iterações, volte para `centraliza` (estratégia de busca).

7. **Lógica do estado `para` (volta completa)**

   * Detecte quando o robô retorna próximo de `self.pose_inicial` **após ter percorrido distância/tempo mínimos** (para evitar parar imediatamente).
   * Ao confirmar a **volta completa**, publique `v = 0.0`, `w = 0.0`, defina `self.running = False` e finalize a Ação.

8. **Restrições importantes**

   * **Não use `while` nem `sleep`** (exceto um `sleep` curto de *boot*, se necessário).
   * Estruture tudo via **callbacks** e/ou **Timers** do ROS (ex.: `rospy.Timer` ou equivalente) para a função de controle.

9. **Ajuste fino (tuning)**

   * Teste e ajuste `self.kp` para um seguimento **suave e preciso**.
   * Faça *clamp* em `w` (`±self.w_max`) para evitar giros bruscos.
   * *Opcional:* use um limiar de erro para decidir a transição `centraliza` → `segue`.

10. **Validações esperadas** (alinhadas ao enunciado)

    * Filtragem correta do **amarelo** e cálculo do **erro** no `image_callback`.
    * Ação com os três estados (`centraliza`, `segue`, `para`) funcionando.
    * **Navegação** pela pista sem colisões.
    * **Parada** após completar **uma volta**.

---

### Observação sobre nomes de variáveis

Para manter consistência com este handout:

* Use `self.cx_linha` para o x do centróide da linha detectada,
* `self.cy` para o y,
* `self.w` para **metade da largura** da imagem,
* `self.erro` para o erro (normalizado, se optar).

Isso facilita a leitura do código da **Ação Segue Linha** e o reuso da lógica de controle.
