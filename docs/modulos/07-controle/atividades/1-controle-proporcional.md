---
layout: default
title: Controle Proporcional
---

# Introdução ao Controle
O controle é uma disciplina fundamental da engenharia que lida com a gestão dinâmica dos sistemas para que eles se comportem de maneira desejada. A essência do controle está em monitorar e ajustar automaticamente a operação de sistemas, desde simples dispositivos mecânicos até complexas redes de comunicação e sistemas robóticos.

O princípio básico é regular as variáveis de um sistema mantendo-as próximas a um valor desejado ou setpoint. Isso é feita através do calculo do erro entre o valor desejado e o valor medido da variável controlada, e então aplicando ações corretivas para minimizar esse erro.

## Tipos de Controle
Existem dois tipos principais de controle: o controle em malha aberta e o controle em malha fechada:

1. Controle em Malha Aberta: Não há feedback do processo. As ações de controle são baseadas em um conjunto pré-definido de instruções que não se alteram em resposta ao estado atual do sistema. Um exemplo simples seria um forno elétrico programado para ficar ligado por um tempo determinado, independentemente da temperatura real dentro do forno.

2. Controle em Malha Fechada (ou Controle com Feedback): Usa o feedback do estado atual do sistema para tomar decisões de controle. Este tipo de controle é mais adaptativo e pode corrigir desvios em relação ao setpoint. Um exemplo seria adicionar um termostato ao forno elétrico para desligá-lo quando a temperatura desejada for atingida.

## Controle Proporcional (P)
Um dos métodos mais simples e amplamente utilizados no controle em malha fechada é o controle proporcional. Este método ajusta a saída do controlador de forma proporcional à diferença (erro) entre o valor desejado (setpoint) e o valor medido da variável controlada. O coeficiente que determina a relação entre o erro e a ação de controle é conhecido como ganho proporcional (K_p). Portanto uma ação de controle proporcional é dada pela seguinte equação:

```python
Ação de Controle = K_p * erro
```

Na esqueção acima, podemos ver que a ação de controle é proporcional ao erro. Se o erro for grande, a ação de controle será grande. Se o erro for pequeno, a ação de controle será pequena. Portanto no exemplo do forno elétrico, ao ligar o forno frio, o erro será grande e a ação de controle será grande, aumentando a potência do forno e elevando a temperatura rapidamente. À medida que a temperatura se aproxima do setpoint, o erro diminui reduzindo a potência do forno, evitando que a temperatura ultrapasse o setpoint drasticamente.

## Resposta da Ação de Controle
Outros conceitos importantes no controle são a resposta da ação de controle e o conceito de estabilidade. A resposta da ação de controle é a reação do sistema à ação de controle. Se a resposta for muito lenta, o sistema pode não atingir o setpoint ou pode crescer indefinidamente, levando o sistema a um estado instável. Se a resposta for muito rápida, o sistema pode ultrapassar o setpoint e oscilar em torno dele. Portanto, é importante ajustar o ganho proporcional para obter uma resposta rápida e estável.

# Exemplo de Controle Proporcional
A seguir, um simulador simples de controle proporcional. Um drone tenta manter uma altitude desejada (o **alvo**). A cada atualização, o controlador mede a altitude, calcula o **erro** (alvo − altitude) e pede ao drone uma velocidade de subida ou descida proporcional a esse erro. Aqui você pode ajustar o ganho proporcional (K_p) e ver como ele muda a **resposta** do drone.

A figura abaixo ilustra o sistema:

![Drone](figs/drone.png)

Em todo o simulador, **o controle P é o que está ligado por padrão**. É ele que você vai usar para centralizar um alvo na imagem (Segue Linha) e na APS 7.

## Exemplo: Controle Proporcional de Altitude em um Drone

<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
.ctrlp { max-width: 760px; margin: 1em 0; font-family: sans-serif; line-height: 1.45; }
.ctrlp [hidden] { display: none !important; }
.ctrlp .ctrlp-campo { margin: 0 0 0.9em 0; }
.ctrlp .ctrlp-campo label { display: block; font-weight: bold; margin-bottom: 0.2em; }
.ctrlp .ctrlp-campo label.ctrlp-exato { display: inline; font-weight: normal; margin: 0 0.3em 0 0.8em; }
.ctrlp .ctrlp-campo input[type="range"] { width: 100%; max-width: 340px; vertical-align: middle; }
.ctrlp .ctrlp-campo input[type="number"] { width: 6.5em; font: inherit; padding: 0.1em 0.3em; }
.ctrlp .ctrlp-campo output { display: inline-block; min-width: 3.5em; margin-left: 0.5em; font-variant-numeric: tabular-nums; }
.ctrlp fieldset { margin: 1em 0; padding: 0.6em 0.9em; border: 1px solid #bbb; border-radius: 8px; }
.ctrlp legend { font-weight: bold; padding: 0 0.4em; }
.ctrlp .ctrlp-opcao { margin: 0 0 0.8em 0; }
.ctrlp .ctrlp-check { font-weight: bold; cursor: pointer; }
.ctrlp .ctrlp-check input { width: 1.1em; height: 1.1em; margin-right: 0.4em; vertical-align: middle; }
.ctrlp .ctrlp-dica { margin: 0.2em 0 0.5em 0; font-size: 0.95em; color: #333; }
.ctrlp details.ctrlp-mais { margin: 0.8em 0; padding: 0.4em 0.9em; border: 1px solid #ddd; border-radius: 8px; }
.ctrlp details.ctrlp-mais summary { cursor: pointer; font-weight: bold; }
.ctrlp .ctrlp-botoes { margin: 0.8em 0; }
.ctrlp .ctrlp-botoes button { margin: 0 0.4em 0.4em 0; }
.ctrlp button { font: inherit; padding: 0.4em 0.9em; border: 1px solid #666; border-radius: 6px; background: #f4f4f4; color: #111; cursor: pointer; }
.ctrlp button:hover { background: #e6e6e6; }
.ctrlp input:focus-visible, .ctrlp button:focus-visible, .ctrlp summary:focus-visible { outline: 3px solid #0b5fff; outline-offset: 2px; }
.ctrlp .ctrlp-status { padding: 0.8em 0.9em; margin: 0.8em 0; border-radius: 8px; border: 1px solid #bbb; background: #f3f3f3; color: #111; }
.ctrlp .ctrlp-status p { margin: 0 0 0.4em 0; }
.ctrlp .ctrlp-status ul { margin: 0.3em 0 0 1.2em; padding: 0; }
.ctrlp .ctrlp-tom-ok { background: #e3f4e6; border-color: #7bb88a; }
.ctrlp .ctrlp-tom-info { background: #e8f0fb; border-color: #8aa9d6; }
.ctrlp .ctrlp-tom-aviso { background: #fff4d6; border-color: #d6b24a; }
.ctrlp .ctrlp-tom-ruim { background: #fbe3e3; border-color: #d68a8a; }
.ctrlp .ctrlp-grafico { width: 100%; min-height: 320px; }
.ctrlp .ctrlp-grafico svg { width: 100%; height: auto; display: block; }
.ctrlp .ctrlp-aviso-grafico { font-size: 0.9em; color: #333; margin: 0.3em 0; }
@media (max-width: 600px) { .ctrlp .ctrlp-campo input[type="range"] { max-width: 100%; } }
</style>
<div class="ctrlp" id="ctrlp-raiz">
  <p>Mexa nos controles abaixo e observe três coisas no gráfico: o <b>alvo</b> (altitude desejada), a <b>resposta</b> (altitude do drone) e o <b>erro</b> (alvo − altitude).</p>
  <div class="ctrlp-campo">
    <label for="ctrlp-kp">Kp (ganho proporcional)</label>
    <input type="range" id="ctrlp-kp" min="0" max="50" step="0.1" value="1">
    <label class="ctrlp-exato" for="ctrlp-kp-num">valor exato</label>
    <input type="number" id="ctrlp-kp-num" min="0" max="50" step="any" value="1">
  </div>
  <div class="ctrlp-campo">
    <label for="ctrlp-dt">dt (intervalo entre duas atualizações do controlador, em s)</label>
    <input type="range" id="ctrlp-dt" min="0.02" max="0.5" step="0.01" value="0.1">
    <output id="ctrlp-dt-val" for="ctrlp-dt">0,10 s</output>
  </div>
  <div class="ctrlp-campo">
    <label for="ctrlp-alvo">Alvo (altitude desejada, em m)</label>
    <input type="range" id="ctrlp-alvo" min="1" max="10" step="0.5" value="5">
    <output id="ctrlp-alvo-val" for="ctrlp-alvo">5,0 m</output>
  </div>
  <div class="ctrlp-botoes" role="group" aria-labelledby="ctrlp-exp-titulo">
    <p class="ctrlp-dica" id="ctrlp-exp-titulo">Experimentos prontos: cada botão carrega um Kp e um dt (só P, sem limites, sem carga). Descubra o que acontece em cada um.</p>
    <button type="button" id="ctrlp-exp-a">Experimento A</button>
    <button type="button" id="ctrlp-exp-b">Experimento B</button>
    <button type="button" id="ctrlp-exp-c">Experimento C</button>
  </div>
  <div class="ctrlp-opcao">
    <label class="ctrlp-check"><input type="checkbox" id="ctrlp-limites" aria-describedby="ctrlp-dica-limites">Ligar limites físicos (velocidade máxima e chão em 0 m)</label>
    <p class="ctrlp-dica" id="ctrlp-dica-limites">Desligado por padrão: sem limites, o comando pode ser tão grande quanto Kp × erro e a altitude pode ficar negativa (valor do modelo matemático, sem chão). Ligado, fica mais realista, mas os limites podem esconder o que o controle está fazendo.</p>
    <div class="ctrlp-campo" id="ctrlp-campo-lim" hidden>
      <label for="ctrlp-lim">Limite de velocidade do drone (em m/s)</label>
      <input type="range" id="ctrlp-lim" min="2" max="10" step="0.5" value="6">
      <output id="ctrlp-lim-val" for="ctrlp-lim">6,0 m/s</output>
    </div>
  </div>
  <details class="ctrlp-mais">
    <summary>Mais opções do cenário</summary>
    <div class="ctrlp-campo">
      <label for="ctrlp-carga">Carga ou vento (puxa o drone para baixo, em m/s)</label>
      <input type="range" id="ctrlp-carga" min="0" max="1.5" step="0.1" value="0">
      <output id="ctrlp-carga-val" for="ctrlp-carga">0,0 m/s</output>
    </div>
  </details>
  <div class="ctrlp-botoes">
    <button type="button" id="ctrlp-reset">Restaurar valores iniciais (só P, sem limites)</button>
  </div>
  <div class="ctrlp-status" id="ctrlp-status" role="status" aria-live="polite"></div>
  <p class="ctrlp-aviso-grafico" id="ctrlp-aviso-escala" hidden></p>
  <div class="ctrlp-grafico" id="ctrlp-grafico" role="img" aria-label="Gráfico do alvo, da altitude do drone e do erro ao longo do tempo"></div>
  <fieldset>
    <legend>Curiosidade (opcional)</legend>
    <p class="ctrlp-dica">Só o P é necessário para a aula e para a APS 7. Os termos I e D abaixo são apenas para quem quiser espiar como o controle pode ser refinado. Ao ligá-los, o gráfico acima muda.</p>
    <div class="ctrlp-opcao">
      <label class="ctrlp-check"><input type="checkbox" id="ctrlp-usa-i" aria-describedby="ctrlp-dica-i">Ligar o termo integral (I)</label>
      <p class="ctrlp-dica" id="ctrlp-dica-i">O I vai somando o erro ao longo do tempo e empurra cada vez mais enquanto o erro não some. Experimente junto com uma carga em ‘Mais opções do cenário’.</p>
      <div class="ctrlp-campo" id="ctrlp-campo-ki" hidden>
        <label for="ctrlp-ki">Ki (ganho integral)</label>
        <input type="range" id="ctrlp-ki" min="0" max="2" step="0.05" value="0.3">
        <output id="ctrlp-ki-val" for="ctrlp-ki">0,30</output>
      </div>
    </div>
    <div class="ctrlp-opcao">
      <label class="ctrlp-check"><input type="checkbox" id="ctrlp-usa-d" aria-describedby="ctrlp-dica-d">Ligar o termo derivativo (D)</label>
      <p class="ctrlp-dica" id="ctrlp-dica-d">O D olha com que rapidez a altitude está mudando e reage a isso, como aliviar o acelerador antes de chegar. Experimente valores diferentes de Kd: mais nem sempre é melhor.</p>
      <div class="ctrlp-campo" id="ctrlp-campo-kd" hidden>
        <label for="ctrlp-kd">Kd (ganho derivativo)</label>
        <input type="range" id="ctrlp-kd" min="0" max="3" step="0.05" value="0.5">
        <output id="ctrlp-kd-val" for="ctrlp-kd">0,50</output>
      </div>
    </div>
  </fieldset>
</div>
<script>
(function () {
  "use strict";

  /* ---------- Modelo (didático) ----------
     Altitude y (m), velocidade real v (m/s) e comando u (m/s):
       y' = v - carga        (a carga/vento puxa o drone para baixo)
       v' = (u - v) / TAU    (inércia: a velocidade real demora ~TAU s para chegar em u)
     O controlador roda a cada dt e segura u constante até a próxima atualização.
     Entre duas atualizações o drone é resolvido de forma EXATA, então o passo de
     desenho (0,02 s) não muda a física. Com a = exp(-dt/TAU) e c = TAU*(1 - a):
       v[k+1] = a*v[k] + (1 - a)*u[k]
       y[k+1] = y[k] + (dt - c)*u[k] + c*v[k] - carga*dt
     u = Kp*e + Ki*soma - Kd*(y[k] - y[k-1])/dt, com e = alvo - y e soma += e*dt
     (o D usa a medição, então não dá "chute" quando o alvo muda).

     Sem limites (padrão) o sistema é linear. Critério: rho = maior |autovalor| da
     matriz da malha fechada (por atualização). rho < 1: a oscilação diminui;
     rho = 1: amplitude constante; rho > 1: cresce. Só P, estado (e, v):
       z^2 - (1 + a - Kp*(dt - c))*z + (a + Kp*D) = 0,   D = c - a*dt > 0.
     O produto dos autovalores é a + Kp*D; o par complexo cruza o círculo unitário
     quando a + Kp*D = 1, ou seja Kp_crit = (1 - a)/D (≈ 2/dt para dt pequeno;
     20,34 com dt = 0,1 e 10,34 com dt = 0,2). As outras condições de Jury
     (P(1) = Kp*(1 - a)*dt > 0 e P(-1) > 0) só falham para Kp muito maior.
     Com I e/ou D os autovalores são calculados numericamente (mesma matriz).
     Classificação: "limite" (marginal) exige par complexo com |rho - 1| <= TOL_MARGINAL;
     "perto" é só quando rho difere de 1 além disso, mas a amplitude muda menos que
     FAIXA_PERTO em DURACAO (amplitude APROXIMADAMENTE constante; ainda cresce ou diminui).

     Com "limites físicos": |u| <= limite e chão em 0 m (o sistema deixa de ser
     linear) e o I tem anti-windup. Os limites podem segurar uma oscilação que,
     sem eles, cresceria: isso não é estabilidade.
     Proteção da página: se |erro| passar de FATOR_CORTE*alvo a simulação para ali.
     Não é um limite físico; a curva fica só com os pontos calculados até o corte. */
  var TAU = 1.0;
  var DURACAO = 20;
  var PASSO_DESENHO = 0.02;
  var FATOR_CORTE = 100;
  var FAIXA_PERTO = 0.05;
  var TOL_MARGINAL = 1e-6;
  var DT_EXPERIMENTOS = 0.2;
  var LIMITES = {
    kp: [0, 50], ki: [0, 2], kd: [0, 3], alvo: [1, 10],
    dt: [0.02, 0.5], lim: [2, 10], carga: [0, 1.5]
  };
  var PADRAO = {
    kp: 1, ki: 0.3, kd: 0.5, alvo: 5, dt: 0.1, lim: 6, carga: 0,
    usaI: false, usaD: false, limites: false
  };

  function sanitizar(entrada) {
    var e = entrada || {};
    var p = { usaI: e.usaI === true, usaD: e.usaD === true, limites: e.limites === true };
    Object.keys(LIMITES).forEach(function (k) {
      var v = Number(e[k]);
      if (!isFinite(v)) { v = PADRAO[k]; }
      p[k] = Math.min(LIMITES[k][1], Math.max(LIMITES[k][0], v));
    });
    return p;
  }

  function kpCritico(dt) {
    var a = Math.exp(-dt / TAU), c = TAU * (1 - a);
    return (1 - a) / (c - a * dt);
  }

  /* Experimentos A, B e C: só P, dt = DT_EXPERIMENTOS. A e B usam uma fração de
     Kp_crit arredondada em 3 casas; C usa Kp_crit em precisão dupla, sem arredondar. */
  function experimentos() {
    var kc = kpCritico(DT_EXPERIMENTOS);
    var r3 = function (x) { return Math.round(x * 1000) / 1000; };
    return { a: r3(1.2 * kc), b: r3(0.7 * kc), c: kc };
  }

  /* Matriz da malha fechada sem limites, estados (y, v, [soma], [y anterior]). */
  function matrizMalha(entrada) {
    var p = sanitizar(entrada);
    var ki = p.usaI ? p.ki : 0, kd = p.usaD ? p.kd : 0, dt = p.dt;
    var a = Math.exp(-dt / TAU), c = TAU * (1 - a), b = dt - c;
    var n = 2 + (ki > 0 ? 1 : 0) + (kd > 0 ? 1 : 0);
    var iS = ki > 0 ? 2 : -1, iY = kd > 0 ? n - 1 : -1;
    var g = [], A = [], i, j;
    for (i = 0; i < n; i++) { g.push(0); }
    g[0] = -(p.kp + kd / dt);
    if (iS >= 0) { g[iS] = ki; }
    if (iY >= 0) { g[iY] = kd / dt; }
    for (i = 0; i < n; i++) { A.push([]); for (j = 0; j < n; j++) { A[i].push(0); } }
    for (j = 0; j < n; j++) { A[0][j] = b * g[j]; A[1][j] = (1 - a) * g[j]; }
    A[0][0] += 1; A[0][1] += c; A[1][1] += a;
    if (iS >= 0) { A[iS][0] = -dt; A[iS][iS] = 1; }
    if (iY >= 0) { A[iY][0] = 1; }
    return A;
  }

  /* Polinômio característico (Faddeev-LeVerrier): coef[i] multiplica z^i. */
  function polinomio(A) {
    var n = A.length, coef = [], M = [], i, j, k, l;
    for (i = 0; i <= n; i++) { coef.push(0); }
    coef[n] = 1;
    for (i = 0; i < n; i++) { M.push([]); for (j = 0; j < n; j++) { M[i].push(0); } }
    var mult = function (X, Y) {
      var R = [];
      for (i = 0; i < n; i++) {
        R.push([]);
        for (j = 0; j < n; j++) { var s = 0; for (l = 0; l < n; l++) { s += X[i][l] * Y[l][j]; } R[i].push(s); }
      }
      return R;
    };
    for (k = 1; k <= n; k++) {
      M = mult(A, M);
      for (i = 0; i < n; i++) { M[i][i] += coef[n - k + 1]; }
      var AM = mult(A, M), tr = 0;
      for (i = 0; i < n; i++) { tr += AM[i][i]; }
      coef[n - k] = -tr / k;
    }
    return coef;
  }

  /* Raízes complexas [re, im] pelo método de Durand-Kerner. */
  function raizes(coef) {
    var n = coef.length - 1, z = [], pot = [1, 0], i, j, it;
    var mul = function (x, y) { return [x[0] * y[0] - x[1] * y[1], x[0] * y[1] + x[1] * y[0]]; };
    for (i = 0; i < n; i++) { z.push(pot); pot = mul(pot, [0.4, 0.9]); }
    for (it = 0; it < 1000; it++) {
      var passo = 0;
      for (i = 0; i < n; i++) {
        var num = [coef[n], 0], den = [1, 0];
        for (j = n - 1; j >= 0; j--) { num = mul(num, z[i]); num[0] += coef[j]; }
        for (j = 0; j < n; j++) { if (j !== i) { den = mul(den, [z[i][0] - z[j][0], z[i][1] - z[j][1]]); } }
        var m2 = den[0] * den[0] + den[1] * den[1];
        if (!(m2 > 0)) { continue; }
        var q = [(num[0] * den[0] + num[1] * den[1]) / m2, (num[1] * den[0] - num[0] * den[1]) / m2];
        z[i] = [z[i][0] - q[0], z[i][1] - q[1]];
        passo = Math.max(passo, Math.sqrt(q[0] * q[0] + q[1] * q[1]));
      }
      if (passo < 1e-14) { break; }
    }
    return z;
  }

  function autovalores(entrada) {
    var z = raizes(polinomio(matrizMalha(entrada)));
    var dom = z[0], mod = function (w) { return Math.sqrt(w[0] * w[0] + w[1] * w[1]); };
    z.forEach(function (w) { if (mod(w) > mod(dom)) { dom = w; } });
    var rho = mod(dom);
    return { rho: rho, dominante: dom, raizes: z,
      oscilatorio: Math.abs(dom[1]) > 1e-7 * Math.max(1, rho) || dom[0] < 0 };
  }

  /* Regime do modelo linear (sem limites) pelo critério dos autovalores.
     "limite": marginal de verdade, par complexo com |rho - 1| <= TOL_MARGINAL.
     "perto": rho diferente de 1, mas a amplitude muda menos que FAIXA_PERTO em DURACAO. */
  function regimeLinear(lin, dt) {
    var F = Math.pow(lin.rho, Math.round(DURACAO / dt));
    if (lin.oscilatorio && Math.abs(lin.rho - 1) <= TOL_MARGINAL) { return "limite"; }
    if (Math.abs(F - 1) <= FAIXA_PERTO) {
      if (lin.oscilatorio) { return "perto"; }
      return lin.rho < 1 ? "lento" : "instavel";
    }
    return lin.rho > 1 ? "instavel" : "estavel";
  }

  function simular(entrada) {
    var p = sanitizar(entrada);
    var kp = p.kp;
    var ki = p.usaI ? p.ki : 0;
    var kd = p.usaD ? p.kd : 0;
    var dt = p.dt;
    var lim = p.limites ? p.lim : Infinity;
    var n = Math.max(1, Math.ceil(dt / PASSO_DESENHO - 1e-9));
    var h = dt / n;
    var a = Math.exp(-h / TAU);
    var c = TAU * (1 - a);
    var passos = Math.round(DURACAO / dt);
    var corte = FATOR_CORTE * p.alvo;
    var y = 0, v = 0, soma = 0, yAnterior = 0, saturados = 0, rodados = 0, tCorte = null;
    var t = [], alt = [], erro = [], cmd = [];
    for (var k = 0; k < passos && tCorte === null; k++) {
      var e = p.alvo - y;
      var termoD = -kd * (y - yAnterior) / dt;
      yAnterior = y;
      var bruto = kp * e + ki * soma + termoD;
      var u = Math.min(lim, Math.max(-lim, bruto));
      var saturou = (u !== bruto);
      if (saturou) { saturados++; }
      rodados++;
      if (ki > 0) {
        if (!p.limites) { soma += e * dt; }
        else {
          if (!saturou || (bruto > lim && e < 0) || (bruto < -lim && e > 0)) { soma += e * dt; }
          soma = Math.min(lim / ki, Math.max(-lim / ki, soma));
        }
      }
      if (k === 0) { t.push(0); alt.push(y); erro.push(e); cmd.push(u); }
      for (var j = 0; j < n; j++) {
        var yNovo = y + (u - p.carga) * h + (v - u) * c;
        var vNovo = u + (v - u) * a;
        if (p.limites && yNovo < 0) { yNovo = 0; if (vNovo < 0) { vNovo = 0; } }
        if (!isFinite(yNovo) || !isFinite(vNovo)) { tCorte = t[t.length - 1]; break; }
        y = yNovo; v = vNovo;
        t.push((k * n + j + 1) * h);
        alt.push(y);
        erro.push(p.alvo - y);
        cmd.push(u);
        if (Math.abs(p.alvo - y) > corte) { tCorte = t[t.length - 1]; break; }
      }
    }
    var valido = [t, alt, erro, cmd].every(function (s) { return s.every(isFinite); });
    return {
      params: p, t: t, alt: alt, erro: erro, cmd: cmd,
      fracSaturada: rodados > 0 ? saturados / rodados : 0,
      cortado: tCorte !== null, tCorte: tCorte, corte: corte, valido: valido
    };
  }

  /* Conta quantas vezes a curva inverte de sentido com amplitude maior que "folga". */
  function reversoes(ys, folga) {
    var dir = 0, topo = ys[0], fundo = ys[0], cont = 0;
    for (var i = 0; i < ys.length; i++) {
      var y = ys[i];
      if (y > topo) { topo = y; }
      if (y < fundo) { fundo = y; }
      if (dir <= 0 && y - fundo > folga) { if (dir < 0) { cont++; } dir = 1; topo = y; }
      else if (dir >= 0 && topo - y > folga) { if (dir > 0) { cont++; } dir = -1; fundo = y; }
    }
    return cont;
  }

  var DESCRICAO = {
    estavel: "estável (a oscilação e o erro diminuem)",
    lento: "estável, mas muito lento",
    limite: "no limite (oscilação de amplitude constante)",
    perto: "perto do limite (amplitude aproximadamente constante em " + DURACAO + " s)",
    instavel: "instável (cresce sem parar)"
  };

  function analisar(res) {
    var p = res.params;
    var kiAtivo = p.usaI && p.ki > 0;
    var n = res.alt.length;
    var tFim = res.t[n - 1];
    var tol = 0.05 * p.alvo;
    var eq = kiAtivo ? 0 : (p.kp > 0 ? p.carga / p.kp : 0);
    var jan = 0.25 * tFim;
    var ampIni = 0, ampFim = 0, maxAlt = -Infinity, minAlt = Infinity, maxDesvio = 0;
    var tChegada = null, ultimoFora = -1, cauda = [];
    for (var i = 0; i < n; i++) {
      var d = Math.abs(res.erro[i] - eq);
      if (res.t[i] <= jan && d > ampIni) { ampIni = d; }
      if (res.t[i] >= tFim - jan) { if (d > ampFim) { ampFim = d; } cauda.push(res.alt[i]); }
      if (res.alt[i] > maxAlt) { maxAlt = res.alt[i]; }
      if (res.alt[i] < minAlt) { minAlt = res.alt[i]; }
      if (Math.abs(res.erro[i]) > maxDesvio) { maxDesvio = Math.abs(res.erro[i]); }
      if (tChegada === null && Math.abs(res.erro[i]) <= tol) { tChegada = res.t[i]; }
      if (Math.abs(res.erro[i] - eq) > tol) { ultimoFora = i; }
    }
    var lin = autovalores(p);
    var regime = regimeLinear(lin, p.dt);
    var m = {
      alvo: p.alvo, equilibrio: eq, ultrapasse: Math.max(0, maxAlt - p.alvo),
      erroFinal: res.erro[n - 1], tFim: tFim, tChegada: tChegada,
      assentou: ultimoFora < n - 1, janela: jan, ampIni: ampIni, ampFim: ampFim,
      rho: lin.rho, fatorPorSegundo: Math.pow(lin.rho, 1 / p.dt), oscilatorio: lin.oscilatorio,
      regimeLinear: regime, oscilaNoFim: reversoes(cauda, tol) >= 2,
      minAlt: minAlt, maxDesvio: maxDesvio, fracSaturada: res.fracSaturada,
      cortado: res.cortado, tCorte: res.tCorte, corte: res.corte
    };
    m.ultrapassePct = 100 * m.ultrapasse / p.alvo;
    var fator = "a cada segundo, a amplitude é multiplicada por " + fmtFator(m.fatorPorSegundo);
    var r;
    if (!res.valido) {
      r = { id: "invalido", tom: "ruim", nome: "Resultado inválido", texto: "A simulação gerou valores inválidos. Restaure os valores iniciais." };
    } else if (p.kp === 0 && !kiAtivo) {
      r = { id: "parado", tom: "info", nome: "Sem controle (Kp = 0)",
        texto: "Com Kp = 0 (e sem I), o comando não depende do erro: nada puxa o drone para o alvo." };
    } else if (!p.limites) {
      if (regime === "instavel") {
        r = { id: "instavel", tom: "ruim",
          nome: lin.oscilatorio ? "Instável: a oscilação cresce" : "Instável: o erro cresce",
          texto: "Pelo critério do modelo, " + fator + " (maior que 1), então ela cresce sem parar." };
      } else if (regime === "limite") {
        r = { id: "limite", tom: "aviso", nome: "No limite (marginal): oscilação de amplitude constante",
          texto: "Pelo critério do modelo, a oscilação não cresce nem diminui: " + fator +
            ", igual a 1 dentro da tolerância numérica (1e-6 por atualização)." };
      } else if (regime === "perto") {
        r = { id: "perto", tom: "aviso", nome: "Perto do limite: amplitude aproximadamente constante",
          texto: "Em " + DURACAO + " s a amplitude muda menos de " + fmt(100 * FAIXA_PERTO, 0) + "%, então parece constante, mas o critério do modelo ainda distingue: " +
            fator + (lin.rho > 1 ? " (maior que 1), então com tempo suficiente a oscilação cresceria." :
              " (menor que 1), então com tempo suficiente a oscilação diminuiria.") };
      } else if (regime === "lento") {
        r = { id: "lento", tom: "aviso", nome: "Estável, mas muito lenta",
          texto: "Nada oscila, mas o drone se move tão devagar que quase nada muda em " + DURACAO + " s." };
      } else {
        r = { id: "estavel", tom: m.assentou ? "ok" : "aviso", nome: "Estável",
          texto: "Pelo critério do modelo, o que resta de oscilação diminui: " + fator + " (menor que 1)." +
            (m.assentou ? "" : " Em " + DURACAO + " s o drone ainda não se acomodou, mas a diferença continua diminuindo.") };
      }
    } else if (m.oscilaNoFim) {
      r = { id: "oscila-limites", tom: regime === "estavel" || regime === "lento" ? "aviso" : "ruim",
        nome: "Ainda oscila no fim (com limites)",
        texto: "No fim dos " + DURACAO + " s o drone ainda sobe e desce. Com limites, o comando e o chão seguram os números, então o gráfico não mostra se a oscilação cresceria. Isso não é estabilidade. Sem os limites, este mesmo ajuste é " + DESCRICAO[regime] + "." };
    } else if (regime === "instavel" || regime === "limite") {
      r = { id: "escondido-limites", tom: "ruim", nome: "Parece comportado só por causa dos limites",
        texto: "Sem os limites, este mesmo ajuste é " + DESCRICAO[regime] + ". O limite não deixa o controle estável." };
    } else {
      r = { id: "estavel-limites", tom: m.assentou ? "ok" : "aviso",
        nome: m.assentou ? "Com limites: o drone se acomoda" : "Com limites: ainda não se acomodou",
        texto: "Sem os limites, este mesmo ajuste também é " + DESCRICAO[regime] + "." };
    }
    r.metricas = m;
    return r;
  }

  function fmt(x, casas) { return Number(x).toFixed(casas).replace(".", ","); }
  function fmtKp(x) { return String(Math.round(x * 1000) / 1000).replace(".", ","); }
  /* Perto de 1 mostra 6 casas, para distinguir 1,000000 (marginal) de 0,999896. */
  function fmtFator(x) { return fmt(x, Math.abs(x - 1) < 0.01 ? 6 : 3); }
  function kpArredondado(x) { return Math.abs(Math.round(x * 1000) / 1000 - x) > 1e-9; }

  /* ---------- Interface ---------- */
  var raiz = (typeof document !== "undefined") ? document.getElementById("ctrlp-raiz") : null;
  if (raiz) {
    var $ = function (id) { return document.getElementById("ctrlp-" + id); };
    var usarFallback = false;

    var lerKp = function () {
      var x = parseFloat($("kp-num").value);
      return isFinite(x) ? x : parseFloat($("kp").value);
    };

    var ler = function () {
      return {
        kp: lerKp(), alvo: parseFloat($("alvo").value),
        dt: parseFloat($("dt").value), lim: parseFloat($("lim").value),
        carga: parseFloat($("carga").value),
        ki: parseFloat($("ki").value), kd: parseFloat($("kd").value),
        usaI: $("usa-i").checked, usaD: $("usa-d").checked, limites: $("limites").checked
      };
    };

    var atualizarSaidas = function (p) {
      $("alvo-val").textContent = fmt(p.alvo, 1) + " m";
      $("dt-val").textContent = fmt(p.dt, 2) + " s";
      $("lim-val").textContent = fmt(p.lim, 1) + " m/s";
      $("carga-val").textContent = fmt(p.carga, 1) + " m/s";
      $("ki-val").textContent = fmt(p.ki, 2);
      $("kd-val").textContent = fmt(p.kd, 2);
      $("campo-ki").hidden = !p.usaI;
      $("campo-kd").hidden = !p.usaD;
      $("campo-lim").hidden = !p.limites;
    };

    var escreverStatus = function (r, p) {
      var m = r.metricas;
      var termos = "P" + (p.usaI ? " + I" : "") + (p.usaD ? " + D" : "");
      var itens = [];
      itens.push("Controle ligado: <b>" + termos + "</b> (Kp " + (kpArredondado(p.kp) ? "≈ " : "= ") + fmtKp(p.kp) + ", dt = " + fmt(p.dt, 2) + " s)" +
        (kpArredondado(p.kp) ? "; a tela arredonda, o campo ‘valor exato’ guarda o Kp completo" : ""));
      itens.push(p.limites ? "Modo: <b>com limites físicos</b> (velocidade até " + fmt(p.lim, 1) + " m/s e chão em 0 m)" :
        "Modo: <b>sem limites</b> (sem limite de velocidade e sem chão)");
      if (r.id !== "parado") {
        itens.push((p.limites ? "Critério do modelo sem limites" : "Critério do modelo") +
          ": a cada segundo a amplitude é multiplicada por <b>" + fmtFator(m.fatorPorSegundo) + "</b>");
        itens.push("Maior distância ao ponto de equilíbrio nos primeiros " + fmt(m.janela, 1) + " s: <b>" + fmt(m.ampIni, 2) +
          " m</b>; nos últimos " + fmt(m.janela, 1) + " s: <b>" + fmt(m.ampFim, 2) + " m</b>");
      }
      itens.push("Ultrapasse máximo: <b>" + fmt(m.ultrapasse, 2) + " m</b> (" + fmt(m.ultrapassePct, 0) + "% do alvo)");
      itens.push("Erro no " + (m.cortado ? "momento do corte" : "fim da simulação") + ": <b>" + fmt(m.erroFinal, 2) + " m</b>");
      itens.push("Chega perto do alvo (±5%) pela primeira vez em: <b>" + (m.tChegada === null ? "não chega" : fmt(m.tChegada, 1) + " s") + "</b>");
      if (p.limites) {
        itens.push("Comando no limite de velocidade em: <b>" + fmt(100 * m.fracSaturada, 0) + "%</b> das atualizações");
      }
      var extra = "";
      if (m.cortado) {
        extra += "<p><b>Simulação interrompida em t = " + fmt(m.tCorte, 2) + " s:</b> a altitude ficou a mais de " +
          fmt(m.corte, 0) + " m do alvo. A curva mostra só o que foi calculado até esse ponto; depois os números ficariam absurdos. Esse corte só protege a página: não é um limite físico do drone.</p>";
      }
      if (!p.limites && m.minAlt < 0) {
        extra += "<p>A altitude ficou negativa. Sem chão neste modo, o gráfico mostra o valor do modelo matemático (um desvio abaixo do ponto de partida), não algo que um drone real faria.</p>";
      }
      var el = $("status");
      el.className = "ctrlp-status ctrlp-tom-" + r.tom;
      el.innerHTML = "<p><b>" + r.nome + ".</b> " + r.texto + "</p>" + extra +
        "<ul><li>" + itens.join("</li><li>") + "</li></ul>";
    };

    var escreverAvisoEscala = function (m) {
      var el = $("aviso-escala");
      var vezes = m.maxDesvio / m.alvo;
      el.hidden = !(vezes > 10);
      el.innerHTML = el.hidden ? "" : "<b>Atenção à escala:</b> o erro chegou a cerca de " + fmt(vezes, 0) +
        " vezes o alvo, então o alvo e o começo da curva aparecem achatados no gráfico.";
    };

    var ticks = function (v0, v1, qtd) {
      var out = [];
      for (var i = 0; i <= qtd; i++) { out.push(v0 + (v1 - v0) * i / qtd); }
      return out;
    };

    /* Gráfico simples em SVG, usado quando o Plotly não carrega ou dá erro. */
    var svgGrafico = function (res) {
      var p = res.params;
      var W = 720, H = 440, ml = 64, mr = 14;
      var xs = function (t) { return ml + (W - ml - mr) * t / DURACAO; };
      var maxA = Math.max(p.alvo, Math.max.apply(null, res.alt));
      var minA = Math.min(0, Math.min.apply(null, res.alt));
      var topoA = maxA + 0.1 * (maxA - minA) + 0.2;
      var maxE = Math.max(0.5, Math.max.apply(null, res.erro));
      var minE = Math.min(-0.5, Math.min.apply(null, res.erro));
      var pa = { y0: 34, h: 250, v0: minA, v1: topoA };
      var pe = { y0: 330, h: 70, v0: minE, v1: maxE };
      var ys = function (pn, v) { return pn.y0 + pn.h * (1 - (v - pn.v0) / (pn.v1 - pn.v0)); };
      var linha = function (pn, vals, cor, largura, tracejado) {
        var pts = vals.map(function (v, i) { return xs(res.t[i]).toFixed(1) + "," + ys(pn, v).toFixed(1); });
        return '<polyline fill="none" stroke="' + cor + '" stroke-width="' + largura + '"' +
          (tracejado ? ' stroke-dasharray="6 4"' : "") + ' points="' + pts.join(" ") + '"/>';
      };
      var eixos = function (pn, rotulo) {
        var s = '<rect x="' + ml + '" y="' + pn.y0 + '" width="' + (W - ml - mr) + '" height="' + pn.h + '" fill="none" stroke="#888"/>';
        ticks(pn.v0, pn.v1, 4).forEach(function (v) {
          s += '<text x="' + (ml - 6) + '" y="' + (ys(pn, v) + 4).toFixed(1) + '" text-anchor="end" font-size="11">' + fmt(v, 1) + "</text>";
        });
        s += '<text x="12" y="' + (pn.y0 + pn.h / 2) + '" font-size="12" text-anchor="middle" transform="rotate(-90 12 ' + (pn.y0 + pn.h / 2) + ')">' + rotulo + "</text>";
        return s;
      };
      var s = '<svg viewBox="0 0 ' + W + " " + H + '" role="img" aria-labelledby="ctrlp-svg-t ctrlp-svg-d" xmlns="http://www.w3.org/2000/svg" font-family="sans-serif">';
      s += '<title id="ctrlp-svg-t">Gráfico simplificado: alvo, altitude e erro</title>';
      s += '<desc id="ctrlp-svg-d">Alvo de ' + fmt(p.alvo, 1) + ' m. A linha azul é a altitude do drone, a tracejada é o alvo e, no gráfico de baixo, a laranja é o erro (alvo menos altitude).' +
        (res.cortado ? " A simulação foi interrompida em " + fmt(res.tCorte, 2) + " s." : "") + "</desc>";
      s += '<rect width="' + W + '" height="' + H + '" fill="#fff"/>';
      s += eixos(pa, "Altitude (m)") + eixos(pe, "Erro (m)");
      s += '<line x1="' + ml + '" x2="' + (W - mr) + '" y1="' + ys(pe, 0).toFixed(1) + '" y2="' + ys(pe, 0).toFixed(1) + '" stroke="#888" stroke-dasharray="2 3"/>';
      if (minA < 0) {
        s += '<line x1="' + ml + '" x2="' + (W - mr) + '" y1="' + ys(pa, 0).toFixed(1) + '" y2="' + ys(pa, 0).toFixed(1) + '" stroke="#888" stroke-dasharray="2 3"/>';
      }
      s += '<line x1="' + xs(0) + '" x2="' + xs(DURACAO) + '" y1="' + ys(pa, p.alvo).toFixed(1) + '" y2="' + ys(pa, p.alvo).toFixed(1) + '" stroke="#444" stroke-width="2" stroke-dasharray="6 4"/>';
      s += linha(pa, res.alt, "#0072B2", 3, false);
      s += linha(pe, res.erro, "#D55E00", 2, false);
      if (res.cortado) {
        var xc = xs(res.tCorte).toFixed(1);
        s += '<line x1="' + xc + '" x2="' + xc + '" y1="' + pa.y0 + '" y2="' + (pe.y0 + pe.h) + '" stroke="#B00020" stroke-width="2" stroke-dasharray="4 3"/>';
        s += '<text x="' + (Number(xc) + 4) + '" y="' + (pa.y0 + 14) + '" font-size="12" fill="#B00020">simulação interrompida</text>';
      }
      ticks(0, DURACAO, 5).forEach(function (tt) {
        s += '<text x="' + xs(tt).toFixed(1) + '" y="' + (pe.y0 + pe.h + 16) + '" text-anchor="middle" font-size="11">' + fmt(tt, 0) + "</text>";
      });
      s += '<text x="' + (ml + (W - ml - mr) / 2) + '" y="' + (H - 6) + '" text-anchor="middle" font-size="12">Tempo (s)</text>';
      s += '<line x1="' + ml + '" x2="' + (ml + 24) + '" y1="14" y2="14" stroke="#444" stroke-width="2" stroke-dasharray="6 4"/><text x="' + (ml + 30) + '" y="18" font-size="12">Alvo (altitude desejada)</text>';
      s += '<line x1="' + (ml + 200) + '" x2="' + (ml + 224) + '" y1="14" y2="14" stroke="#0072B2" stroke-width="3"/><text x="' + (ml + 230) + '" y="18" font-size="12">Resposta (altitude do drone)</text>';
      s += '<line x1="' + (ml + 430) + '" x2="' + (ml + 454) + '" y1="14" y2="14" stroke="#D55E00" stroke-width="2"/><text x="' + (ml + 460) + '" y="18" font-size="12">Erro = alvo − altitude</text>';
      return s + "</svg>";
    };

    var desenharFallback = function (res, motivo) {
      usarFallback = true;
      var alvoEl = $("grafico");
      if (typeof Plotly !== "undefined" && Plotly.purge) { try { Plotly.purge(alvoEl); } catch (e) { /* sem efeito */ } }
      alvoEl.innerHTML = '<p class="ctrlp-aviso-grafico"><b>Gráfico simplificado:</b> ' + motivo + "</p>" + svgGrafico(res);
    };

    var desenharPlotly = function (res) {
      var p = res.params;
      var dados = [
        { x: [0, DURACAO], y: [p.alvo, p.alvo], mode: "lines", name: "Alvo (altitude desejada)", line: { color: "#444444", dash: "dash", width: 2 } },
        { x: res.t, y: res.alt, mode: "lines", name: "Resposta (altitude do drone)", line: { color: "#0072B2", width: 3 } },
        { x: res.t, y: res.erro, mode: "lines", name: "Erro = alvo − altitude", yaxis: "y2", line: { color: "#D55E00", width: 2 } }
      ];
      var formas = [], notas = [];
      if (res.cortado) {
        formas.push({ type: "line", xref: "x", yref: "paper", x0: res.tCorte, x1: res.tCorte, y0: 0, y1: 1,
          line: { color: "#B00020", width: 2, dash: "dot" } });
        notas.push({ x: res.tCorte, y: 1, xref: "x", yref: "paper", xanchor: "left", yanchor: "top",
          text: "simulação interrompida", showarrow: false, font: { color: "#B00020" } });
      }
      var layout = {
        height: 480,
        xaxis: { title: "Tempo (s)", anchor: "y2", range: [0, DURACAO] },
        yaxis: { title: "Altitude (m)", domain: [0.36, 1], zeroline: true },
        yaxis2: { title: "Erro (m)", domain: [0, 0.27], zeroline: true },
        legend: { orientation: "h", x: 0, y: 1.12 },
        margin: { t: 50, r: 20, l: 60, b: 50 },
        shapes: formas, annotations: notas
      };
      var promessa = Plotly.react("ctrlp-grafico", dados, layout, { responsive: true, displayModeBar: false });
      if (promessa && promessa.catch) {
        promessa.catch(function () {
          desenharFallback(res, "o gráfico interativo (Plotly) falhou ao desenhar.");
        });
      }
    };

    var atualizar = function () {
      var p = ler();
      atualizarSaidas(p);
      var res = simular(p);
      var r = analisar(res);
      escreverStatus(r, res.params);
      escreverAvisoEscala(r.metricas);
      $("grafico").setAttribute("aria-label",
        "Gráfico do alvo, da altitude do drone e do erro ao longo do tempo. " + r.nome + "." +
        (res.cortado ? " Curva interrompida em " + fmt(res.tCorte, 2) + " s." : ""));
      if (!usarFallback && typeof Plotly !== "undefined") {
        try { desenharPlotly(res); } catch (e) {
          desenharFallback(res, "o gráfico interativo (Plotly) falhou ao desenhar.");
        }
      } else {
        desenharFallback(res, "a biblioteca de gráficos (Plotly) não carregou; os números acima continuam valendo.");
      }
    };

    var definirKp = function (kp) { $("kp-num").value = String(kp); $("kp").value = String(kp); };

    var carregar = function (valores) {
      definirKp(valores.kp);
      ["alvo", "dt", "lim", "carga", "ki", "kd"].forEach(function (k) { $(k).value = valores[k]; });
      $("usa-i").checked = valores.usaI;
      $("usa-d").checked = valores.usaD;
      $("limites").checked = valores.limites;
      atualizar();
    };

    raiz.addEventListener("input", function (ev) {
      if (ev.target === $("kp")) { $("kp-num").value = $("kp").value; }
      else if (ev.target === $("kp-num") && isFinite(parseFloat($("kp-num").value))) { $("kp").value = $("kp-num").value; }
      atualizar();
    });
    raiz.addEventListener("change", function (ev) {
      if (ev.target === $("kp-num")) { definirKp(sanitizar({ kp: lerKp() }).kp); }
      atualizar();
    });
    ["a", "b", "c"].forEach(function (x) {
      $("exp-" + x).addEventListener("click", function () {
        carregar(Object.assign({}, PADRAO, { kp: experimentos()[x], dt: DT_EXPERIMENTOS }));
      });
    });
    $("reset").addEventListener("click", function () { carregar(PADRAO); });
    atualizar();
  }

  if (typeof window !== "undefined") {
    window.RobcompControleP = {
      simular: simular, analisar: analisar, autovalores: autovalores, matrizMalha: matrizMalha,
      polinomio: polinomio, raizes: raizes, regimeLinear: regimeLinear, kpCritico: kpCritico,
      experimentos: experimentos, PADRAO: PADRAO, LIMITES: LIMITES, TAU: TAU, DURACAO: DURACAO,
      FATOR_CORTE: FATOR_CORTE, FAIXA_PERTO: FAIXA_PERTO, TOL_MARGINAL: TOL_MARGINAL, DT_EXPERIMENTOS: DT_EXPERIMENTOS
    };
  }
})();
</script>

### O que este simulador simplifica
- O drone só sobe e desce (uma dimensão), sem ruído de sensor.
- O controlador pede uma **velocidade** e a mantém até a próxima atualização (a cada dt). O drone leva cerca de 1 s para alcançar a velocidade pedida (inércia).
- **Por padrão não há limites**: o comando pode ser tão grande quanto Kp × erro e não existe chão. Assim dá para ver, sem disfarce, se a oscilação diminui, fica do mesmo tamanho ou cresce. Nesse modo a altitude pode ficar negativa: leia esse valor como um desvio no modelo matemático, não como algo que um drone real faria.
- Com **limites físicos** ligados, o comando fica entre −limite e +limite e o drone para no chão (0 m). É mais realista, mas os limites podem segurar uma oscilação que, sem eles, cresceria. Isso **não** torna o controle estável.
- Se a altitude se afastar do alvo mais de 100 vezes o valor do alvo, a simulação é interrompida e o gráfico mostra a curva só até ali. Esse corte protege a página; não é um limite do drone.
- A carga (ou vento) é uma velocidade constante para baixo.

!!! warning "Atenção"
    Os números do simulador (alvo, Kp, limite de velocidade) valem só para este drone de mentirinha. No robô real, comece com ganhos pequenos e respeite os limites de velocidade da atividade (por exemplo, ±0,5 rad/s na velocidade angular do [Segue Linha](2-segue-linha.md)).

## Perguntas para explorar
Use o simulador para investigar. Anote o que você observou no gráfico e no quadro de resultado. Aqui não há gabarito: discuta com a sua dupla.

1. Com os valores iniciais, suba o Kp aos poucos. O que muda no tempo para chegar perto do alvo? E no quanto o drone passa do alvo?
2. Agora desça o Kp até valores bem pequenos. O que muda?
3. O que acontece com Kp = 0?
4. Carregue os Experimentos A, B e C. Em cada um, a oscilação diminui, fica do mesmo tamanho ou cresce? O que no gráfico mostrou isso?
5. Parta do experimento em que a oscilação fica do mesmo tamanho e mude o Kp só um pouco, para cima e para baixo (use o campo de valor exato). O que acontece?
6. Escolha um Kp e mude só o dt. O que acontece com a resposta? Existe um Kp que funciona com um dt e deixa de funcionar com outro?
7. Quando o erro fica negativo, para que lado o comando manda o drone? Como isso aparece no [Segue Linha](2-segue-linha.md)?
8. Ligue os limites físicos em um caso em que a oscilação cresce. O que muda no gráfico? O que o quadro de resultado diz sobre o mesmo ajuste sem limites?
9. Em ‘Mais opções do cenário’, coloque uma carga. O drone chega exatamente no alvo? O que muda quando você aumenta o Kp?
