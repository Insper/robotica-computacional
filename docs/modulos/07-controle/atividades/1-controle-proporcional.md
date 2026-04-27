---
layout: default
title: Simulação Drone PID
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
A seguir, implementamos um exemplo simples de controle proporcional em Python. Neste exemplo, simulamos um sistema de controle de um drone que tenta manter a altitude constante para uma dada entrada de altitude, setpoint. Neste exemplo, você pode ajustar o ganho proporcional (K_p) e a resposta do sistema para visualizar como o controle proporcional afeta a resposta do sistema.

A figura abaixo ilustra o sistema:

![Drone](figs/drone.png)

Durante seus experimentos, tente responder às seguintes perguntas:

1. O que acontece se o ganho proporcional for muito baixo? E se for muito alto?
2. Qual a relação entre o ganho proporcional e a resposta do sistema para o momento em que o sistema nunca atinge o setpoint, ou seja, o sistema é instável?

---
layout: default
title: Simulação Drone PID
---

## Exemplo Controle Proporcional de Altitude (Drone)

Experimente ajustar o ganho proporcional **Kp** e o tempo de atualização do controlador para ver como o drone reage para atingir a altitude desejada.

## Exemplo: Controle Proporcional de Altitude em um Drone

Experimente ajustar o ganho proporcional **Kp** e o tempo de atualização do controlador para observar como o drone tenta atingir a altitude desejada.

Nesta simulação, o controlador é atualizado em intervalos discretos, mas a física do drone é simulada com um passo interno menor. Isso evita instabilidades numéricas artificiais e deixa o comportamento mais próximo de um sistema real.

<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>

<div style="margin-top:1em; font-family:sans-serif; max-width:760px">

  <div style="margin-bottom:1em">
    <label for="kp"><b>Kp: Ganho proporcional:</b></label><br>
    <input 
      type="range" 
      id="kp" 
      min="0" 
      max="8" 
      value="1.5" 
      step="0.1" 
      style="width:320px"
      oninput="atualizarValores(); updatePlot();"
    >
    <span id="kp_val">1.5</span>
  </div>

  <div style="margin-bottom:1em">
    <label for="controlDt"><b>Tempo de atualização do controlador:</b></label><br>
    <input 
      type="range" 
      id="controlDt" 
      min="0.02" 
      max="1.00" 
      value="0.20" 
      step="0.02" 
      style="width:320px"
      oninput="atualizarValores(); updatePlot();"
    >
    <span id="controlDt_val">0.20</span> s
  </div>

  <div 
    id="status" 
    style="
      padding:0.8em; 
      border-radius:8px; 
      background:#f3f3f3; 
      margin-bottom:1em;
      line-height:1.4;
    "
  >
  </div>

</div>

<div id="grafico" style="width:100%; max-width:760px; height:440px;"></div>

<script>
function clamp(valor, min, max) {
  return Math.max(min, Math.min(max, valor));
}

function simular(Kp, controlDt) {
  const setpoint = 10.0;
  const tempoFinal = 12.0;

  // Passo interno fixo da simulação física.
  // Ele não depende do tempo de atualização do controlador.
  const simDt = 0.005;

  // Modelo físico simplificado
  let altitude = 0.0;
  let velocidade = 0.0;
  let aceleracaoComandada = 0.0;

  // Limites físicos simplificados
  const aceleracaoMax = 8.0;   // m/s²
  const amortecimento = 0.9;   // resistência proporcional à velocidade

  let proximaAtualizacaoControle = 0.0;

  const tempos = [];
  const altitudes = [];
  const setpoints = [];
  const velocidades = [];
  const comandos = [];

  let overshootMax = 0.0;
  let erroFinal = 0.0;

  for (let t = 0; t <= tempoFinal; t += simDt) {

    // O controlador só atualiza a cada controlDt segundos
    if (t >= proximaAtualizacaoControle) {
      const erro = setpoint - altitude;

      // Controle proporcional:
      // erro positivo gera aceleração para cima;
      // erro negativo gera aceleração para baixo.
      aceleracaoComandada = Kp * erro;

      // Saturação: motores não conseguem gerar aceleração infinita
      aceleracaoComandada = clamp(aceleracaoComandada, -aceleracaoMax, aceleracaoMax);

      proximaAtualizacaoControle += controlDt;
    }

    // Física simplificada:
    // aceleração efetiva = comando do controlador - amortecimento aerodinâmico
    const aceleracaoEfetiva = aceleracaoComandada - amortecimento * velocidade;

    velocidade += aceleracaoEfetiva * simDt;
    altitude += velocidade * simDt;

    // Impede altitude negativa
    if (altitude < 0) {
      altitude = 0;
      velocidade = Math.max(0, velocidade);
    }

    const overshootAtual = Math.max(0, altitude - setpoint);
    overshootMax = Math.max(overshootMax, overshootAtual);
    erroFinal = setpoint - altitude;

    // Guarda menos pontos para o gráfico ficar leve
    if (Math.round(t / simDt) % 10 === 0) {
      tempos.push(t);
      altitudes.push(altitude);
      setpoints.push(setpoint);
      velocidades.push(velocidade);
      comandos.push(aceleracaoComandada);
    }
  }

  return {
    tempos,
    altitudes,
    setpoints,
    velocidades,
    comandos,
    setpoint,
    overshootMax,
    erroFinal
  };
}

function classificarComportamento(Kp, controlDt, overshootMax, erroFinal) {
  const produto = Kp * controlDt;

  if (Kp === 0) {
    return {
      texto: "Sem controle: Kp = 0, então o drone não reage ao erro de altitude.",
      cor: "#fff3cd"
    };
  }

  if (overshootMax > 3.0 || produto > 3.0) {
    return {
      texto: "Resposta agressiva: o drone tende a passar bastante da altitude desejada e pode oscilar.",
      cor: "#f8d7da"
    };
  }

  if (Math.abs(erroFinal) > 1.0) {
    return {
      texto: "Resposta lenta ou ainda não estabilizada: o drone está se aproximando, mas ainda possui erro relevante.",
      cor: "#fff3cd"
    };
  }

  return {
    texto: "Resposta estável: o drone se aproxima da altitude desejada com erro final pequeno.",
    cor: "#d4edda"
  };
}

function atualizarValores() {
  const Kp = parseFloat(document.getElementById("kp").value);
  const controlDt = parseFloat(document.getElementById("controlDt").value);

  document.getElementById("kp_val").textContent = Kp.toFixed(1);
  document.getElementById("controlDt_val").textContent = controlDt.toFixed(2);
}

function updatePlot() {
  const Kp = parseFloat(document.getElementById("kp").value);
  const controlDt = parseFloat(document.getElementById("controlDt").value);

  const resultado = simular(Kp, controlDt);

  const {
    tempos,
    altitudes,
    setpoints,
    velocidades,
    comandos,
    overshootMax,
    erroFinal
  } = resultado;

  const comportamento = classificarComportamento(Kp, controlDt, overshootMax, erroFinal);

  const status = document.getElementById("status");
  status.style.background = comportamento.cor;
  status.innerHTML = `
    <b>Leitura da simulação:</b> ${comportamento.texto}<br>
    <b>Overshoot máximo:</b> ${overshootMax.toFixed(2)} m &nbsp; | &nbsp;
    <b>Erro final:</b> ${erroFinal.toFixed(2)} m
  `;

  const traceAltitude = {
    x: tempos,
    y: altitudes,
    mode: "lines",
    name: "Altitude do drone",
    line: { width: 3 }
  };

  const traceSetpoint = {
    x: tempos,
    y: setpoints,
    mode: "lines",
    name: "Altitude desejada",
    line: { dash: "dash", width: 2 }
  };

  const traceVelocidade = {
    x: tempos,
    y: velocidades,
    mode: "lines",
    name: "Velocidade vertical",
    yaxis: "y2",
    line: { width: 2, dash: "dot" },
    visible: "legendonly"
  };

  const traceComando = {
    x: tempos,
    y: comandos,
    mode: "lines",
    name: "Aceleração comandada",
    yaxis: "y2",
    line: { width: 2, dash: "dot" },
    visible: "legendonly"
  };

  const layout = {
    title: {
      text: `Controle Proporcional de Altitude (Kp=${Kp.toFixed(1)}, atualização=${controlDt.toFixed(2)}s)`,
      font: { size: 16 }
    },
    xaxis: {
      title: "Tempo (s)"
    },
    yaxis: {
      title: "Altitude (m)",
      range: [0, 16]
    },
    yaxis2: {
      title: "Velocidade / comando",
      overlaying: "y",
      side: "right",
      showgrid: false
    },
    legend: {
      orientation: "h",
      y: -0.25
    },
    margin: {
      t: 70,
      r: 60,
      l: 55,
      b: 90
    }
  };

  const config = {
    responsive: true,
    displayModeBar: false
  };

  Plotly.react(
    "grafico",
    [traceAltitude, traceSetpoint, traceVelocidade, traceComando],
    layout,
    config
  );
}

atualizarValores();
updatePlot();
</script>

Observe que aumentar Kp nem sempre melhora o controle. Um ganho proporcional muito baixo faz o drone subir lentamente. Um ganho muito alto faz o drone reagir de maneira agressiva, podendo ultrapassar a altitude desejada e oscilar. Além disso, quanto maior o intervalo de atualização do controlador, mais "atrasada" fica a reação do sistema, o que também pode aumentar as oscilações.