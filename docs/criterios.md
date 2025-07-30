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
N = 0,2 * NAPS + 0,2 * Projeto + 0,3 * AI + 0,3 * AF;

NAPS = (APS_UNIDADE_1 + APS_UNIDADE_2 + APS_UNIDADE_3) / 3;
```

onde:

* `APS_UNIDADE_#` :
```
APS_UNIDADE_1 = APS_1 * 0,25 + APS_2 * 0,25 + APS_3 * 0,5;
APS_UNIDADE_2 = APS_4 * 0,20 + APS_5 * 0,30 + APS_6 * 0,5;
APS_UNIDADE_3 = APS_7 * 0,5 + APS_8 * 0,5;
```

* `NAPS` é a média das notas das APS de cada unidade;

* `Projeto` é a nota dos projetos, calculada da seguinte forma:

* `AI` é a nota da Avaliação Intermediária.

* `AF` é a nota da Avaliação Final.

Alunos que não atingirem a média para aprovação poderão fazer uma prova **DELTA** para substituir a nota da menor avaliação, desde que atendam aos seguintes critérios. **A prova DELTA será a aplicada logo após a prova SUB.**

1. `NAPS >= 5`;
2. `AI >= 4 OU AF >= 4`;
3. nota de projeto maior que 4.

Neste caso, a nota da prova **DELTA** substituirá a menor nota entre `AI` e `AF`, limitada ao mínimo necessário para aprovação.

## Datas das avaliações

Algumas datas importantes para o semestre:

!!! info
    Datas com `~` estão sujeitas a alterações.

* **Avaliação Intermediária**: {{ data_AI }}

* **Avaliação Final**: {{ data_PF }}

* **Prova SUB**: {{ data_SUB }}

* **Prova DELTA**: {{ data_DELTA }}


