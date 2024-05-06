<style>
section.progress-section.show {
    width: 1024px;
}


section.progress-section.show iframe {
    width: 100%;
    height: 80vh;
}

</style>

## Critérios de aprovação

A nota será composta da seguinte forma:

```
N = 0,3*NAPS + 0,2*Projeto + 0,2*AI + 0,3*AF;

NAPS = (APS_UNIDADE_1 + APS_UNIDADE_2 + 0,5*APS_UNIDADE_3) / 2,5;
```
onde:

* `APS_UNIDADE_#` :
```
APS_UNIDADE_1 = ( **MAIOR**(APS_1 ; APS_2) + APS_3 ) / 2;
APS_UNIDADE_2 = ( APS_4 + APS_5 ) / 2;
APS_UNIDADE_3 = APS_6;
```

* `NAPS` é a média das notas das APS de cada unidade;

* `Projeto` é a nota do projeto;

* `AI` é a nota da Avaliação Intermediária. Será realizada na semana do dia {{ data_AI }};

* `AF` é a nota da Avaliação Final. Será realizada na semana do dia {{ data_PF }}.

Será aprovado o estudante que siga as seguites restrições:

1. `NAPS >= 5`;
2. `AI + AF >= 10`;
3. `AI >= 4 E AF >= 4`;
4. nota de projeto maior que 4.

!!! important "Segundas chances"
    1. Alunos que não atenderem ao critério 2 ou ao critério 3, mas atenderem o critério 1, poderão fazer uma prova DELTA para substituir a nota da menor avaliação;
    2. Podem também fazer uma prova DELTA alunos que precisarem de nota na média para serem aprovados - desde que atendam a todos os outros critérios de aprovação;


