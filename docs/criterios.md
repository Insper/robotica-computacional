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
N = 0,3*NAPS + 0,3*Projeto + 0,2*AI + 0,2*AF;

NAPS = (APS_UNIDADE_1 + APS_UNIDADE_2 + APS_UNIDADE_3) / 3;

APS_UNIDADE_# = ( **MAIOR**(APS_1o ; APS_2o) + APS_3o ) / 2;
```
onde:

* `APS_UNIDADE_#` média aritmética entre a nota da terceira APS (da unidade) e a maior nota entre a primeira e a segunda APS (da unidade). Para a unidade 3, a média é feita entre as duas melhores APSs;

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


