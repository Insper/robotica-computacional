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

  <p>
    Experimente ajustar o ganho proporcional <b>Kp</b> e o tempo de atualização <b>dt</b>.
    A estabilidade depende diretamente do produto <b>Kp * dt</b>.
  </p>

  <div style="margin-bottom:1em">
    <label for="kp"><b>Kp - ganho proporcional:</b></label><br>
    <input
      type="range"
      id="kp"
      min="0"
      max="20"
      value="5"
      step="0.1"
      style="width:340px"
      oninput="atualizarValores(); updatePlot();"
    >
    <span id="kp_val">5.0</span>
  </div>

  <div style="margin-bottom:1em">
    <label for="dt"><b>dt - tempo de atualização do controlador:</b></label><br>
    <input
      type="range"
      id="dt"
      min="0.02"
      max="1.00"
      value="0.25"
      step="0.01"
      style="width:340px"
      oninput="atualizarValores(); updatePlot();"
    >
    <span id="dt_val">0.25</span> s
  </div>

  <div
    id="status"
    style="
      padding:0.9em;
      border-radius:8px;
      background:#f3f3f3;
      margin-bottom:1em;
      line-height:1.45;
      border:1px solid #ddd;
    "
  ></div>

</div>

<div id="grafico" style="width:100%; max-width:760px; height:440px;"></div>

<script>
function classificarRegime(Kp, dt) {
  var produto = Kp * dt;
  var eps = 0.005;

  if (Kp === 0) {
    return {
      nome: "Sem controle",
      texto: "Kp = 0. O drone não reage ao erro.",
      cor: "#eeeeee"
    };
  }

  if (produto > 0 && produto < 0.5) {
    return {
      nome: "Estável, porém lento",
      texto: "O erro diminui sem oscilar, mas a aproximação pode ser lenta demais para uma aplicação real.",
      cor: "#e2e3e5"
    };
  }

  if (produto >= 0.5 && produto < 1) {
    return {
      nome: "Estável sem oscilação",
      texto: "O erro diminui sem trocar de sinal. A resposta é suave e mais rápida que no regime lento.",
      cor: "#d4edda"
    };
  }

  if (Math.abs(produto - 1) < eps) {
    return {
      nome: "Ideal discreto",
      texto: "Neste modelo simplificado, o drone chega exatamente ao alvo em uma única atualização.",
      cor: "#cce5ff"
    };
  }

  if (produto > 1 && produto < 1.6) {
    return {
      nome: "Estável oscilatório / rápido",
      texto: "O erro troca de sinal a cada passo, mas diminui rapidamente. É uma região útil quando se aceita pequena oscilação em troca de resposta mais rápida.",
      cor: "#d4edda"
    };
  }

  if (produto >= 1.6 && produto < 2) {
    return {
      nome: "Oscilatório agressivo",
      texto: "O sistema ainda é estável, mas oscila bastante antes de convergir. Pode ser inadequado em um drone real.",
      cor: "#fff3cd"
    };
  }

  if (Math.abs(produto - 2) < eps) {
    return {
      nome: "Perfeitamente instável",
      texto: "O erro troca de sinal, mas não diminui. O sistema oscila indefinidamente com a mesma amplitude.",
      cor: "#ffe0b2"
    };
  }

  if (produto > 2) {
    return {
      nome: "Super instável",
      texto: "O erro troca de sinal e aumenta a cada passo. A altitude diverge em oscilações cada vez maiores.",
      cor: "#f8d7da"
    };
  }

  return {
    nome: "Caso limite",
    texto: "Regime de transição.",
    cor: "#eeeeee"
  };
}

function simular(Kp, dt) {
  var setpoint = 10.0;
  var altitudeInicial = 0.0;
  var tempoFinal = 10.0;

  var tempos = [];
  var altitudes = [];
  var setpoints = [];
  var erros = [];

  var altitude = altitudeInicial;
  var maxAbsPlot = 100;
  var divergiu = false;

  for (var t = 0; t <= tempoFinal + 1e-9; t += dt) {
    var erro = setpoint - altitude;

    tempos.push(Number(t.toFixed(3)));
    altitudes.push(altitude);
    setpoints.push(setpoint);
    erros.push(erro);

    var comando = Kp * erro;
    altitude = altitude + comando * dt;

    if (Math.abs(altitude) > maxAbsPlot) {
      divergiu = true;

      var tFinal = t + dt;
      var erroFinal = setpoint - altitude;

      tempos.push(Number(tFinal.toFixed(3)));
      altitudes.push(altitude);
      setpoints.push(setpoint);
      erros.push(erroFinal);

      break;
    }
  }

  return {
    tempos: tempos,
    altitudes: altitudes,
    setpoints: setpoints,
    erros: erros,
    divergiu: divergiu
  };
}

function atualizarValores() {
  var Kp = parseFloat(document.getElementById("kp").value);
  var dt = parseFloat(document.getElementById("dt").value);

  document.getElementById("kp_val").textContent = Kp.toFixed(1);
  document.getElementById("dt_val").textContent = dt.toFixed(2);
}

function updatePlot() {
  if (typeof Plotly === "undefined") {
    document.getElementById("status").innerHTML =
      "<b>Erro:</b> Plotly não carregou. Verifique se a prévia permite carregar scripts externos.";
    return;
  }

  var Kp = parseFloat(document.getElementById("kp").value);
  var dt = parseFloat(document.getElementById("dt").value);

  var produto = Kp * dt;
  var fatorErro = 1 - produto;

  var regime = classificarRegime(Kp, dt);
  var resultado = simular(Kp, dt);

  var status = document.getElementById("status");
  status.style.background = regime.cor;
  status.innerHTML =
    "<b>Regime:</b> " + regime.nome + "<br>" +
    regime.texto + "<br><br>" +
    "<b>Kp * dt:</b> " + produto.toFixed(3) + " &nbsp; | &nbsp; " +
    "<b>fator do erro:</b> 1 - Kp * dt = " + fatorErro.toFixed(3) +
    (resultado.divergiu ? "<br><b>Observação:</b> a simulação foi interrompida porque a altitude cresceu demais." : "");

  var traceAltitude = {
    x: resultado.tempos,
    y: resultado.altitudes,
    mode: "lines+markers",
    name: "Altitude do drone",
    line: { width: 3 },
    marker: { size: 6 }
  };

  var traceSetpoint = {
    x: resultado.tempos,
    y: resultado.setpoints,
    mode: "lines",
    name: "Altitude desejada",
    line: { dash: "dash", width: 2 }
  };

  var traceErro = {
    x: resultado.tempos,
    y: resultado.erros,
    mode: "lines+markers",
    name: "Erro",
    yaxis: "y2",
    line: { width: 2, dash: "dot" },
    marker: { size: 5 },
    visible: "legendonly"
  };

  var yValores = resultado.altitudes.concat(resultado.setpoints);
  var yMin = Math.min.apply(null, yValores);
  var yMax = Math.max.apply(null, yValores);
  var margem = Math.max(2, 0.15 * (yMax - yMin));

  var layout = {
    title: {
      text: "Controle Proporcional Discreto - Kp=" + Kp.toFixed(1) + ", dt=" + dt.toFixed(2) + "s",
      font: { size: 16 }
    },
    xaxis: {
      title: "Tempo (s)"
    },
    yaxis: {
      title: "Altitude (m)",
      range: [yMin - margem, yMax + margem],
      zeroline: true
    },
    yaxis2: {
      title: "Erro",
      overlaying: "y",
      side: "right",
      showgrid: false
    },
    shapes: [
      {
        type: "line",
        x0: 0,
        x1: resultado.tempos[resultado.tempos.length - 1],
        y0: 0,
        y1: 0,
        line: {
          width: 1,
          dash: "dot"
        }
      }
    ],
    legend: {
      orientation: "h",
      y: -0.25
    },
    margin: {
      t: 70,
      r: 65,
      l: 55,
      b: 90
    }
  };

  var config = {
    responsive: true,
    displayModeBar: false
  };

  Plotly.react(
    "grafico",
    [traceAltitude, traceSetpoint, traceErro],
    layout,
    config
  );
}

window.addEventListener("load", function() {
  atualizarValores();
  updatePlot();
});
</script>

Observe que aumentar Kp nem sempre melhora o controle. Um ganho proporcional muito baixo faz o drone subir lentamente. Um ganho muito alto faz o drone reagir de maneira agressiva, podendo ultrapassar a altitude desejada e oscilar. Além disso, quanto maior o intervalo de atualização do controlador, mais "atrasada" fica a reação do sistema, o que também pode aumentar as oscilações.