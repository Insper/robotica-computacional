
# Resolvendo Conflitos de Versão no Git

Neste guia assumimos que vocês já criaram o repositório no Github e adicionaram
todos os membros do grupo como colaboradores. Antes de começar, vamos dar
algumas dicas práticas do fluxo de trabalho com git.

## Dicas práticas de git

1. Procure sempre dar um `git pull` antes de começar a trabalhar. Assim você
diminui as chances de ter algum conflito de versões;
2. Faça `commits` curtos e frequentes. Isso facilita o `merge` automático dos
arquivos;
3. Não deixe para fazer o `push` ao final do dia. Assim que tiver uma versão
funcional faça o commit e o `push`;
4. Evite deixar mudanças sem `commit` ao final do dia, assim você diminui a
chance de ter que lidar com um conflito de versões quando voltar a trabalhar.

## Resolvendo os conflitos

Em alguns casos quando você faz um `git pull` e recebe uma mensagem parecida
com essa:

    Auto-merging [ARQUIVO1]
    CONFLICT (content): Merge conflict in [ARQUIVO2]
    Automatic merge failed; fix conflicts and then commit the result.

Isso significa que a versão do `ARQUIVO2` no repositório estava diferente da sua
versão local e o git não conseguiu juntar os dois. Nesse caso precisamos
juntar os arquivos manualmente.

### Método 1

Se abrirmos o `ARQUIVO2` veremos marcações em algumas linhas:

    Algumas linhas iguais nas duas versões
    <<<<<<< HEAD
    essa linha está diferente em uma versão
    =======
    na outra versão a mesma linhas está assim...
    >>>>>>> [algum código]

Isso quer dizer que o `ARQUIVO2` no Github contém:

    Algumas linhas iguais nas duas versões
    essa linha está diferente em uma versão

E na sua versão local:

    Algumas linhas iguais nas duas versões
    na outra versão a mesma linhas está assim...

Para resolver o conflito basta apagar as marcações (`<<<<<<<`, `=======`,
`>>>>>>>`), deixando a versão que você quer deixar (ou uma combinação das duas).

### Método 2

Podemos também usar uma ferramenta para nos auxiliar nesse processo. Em um
terminal digite:

    git mergetool

Um programa mostrando os conflitos será aberto. Para cada conflito selecione a
versão do código que você quer manter. Depois de resolver todos os conflitos,
salve o arquivo e feche o programa.

É possível usar diversos programas como `mergetool`. Veja [este link](https://developer.atlassian.com/blog/2015/12/tips-tools-to-solve-git-conflicts/)
para uma lista de programas que podem ser utilizados como `mergetool` e
[este outro link](https://gist.github.com/karenyyng/f19ff75c60f18b4b8149) para
falar para o git qual programa você quer usar.

### Concluindo a Resolução dos Conflitos

Depois de resolver os conflitos com qualquer um dos métodos acima, você precisa
concluir a resolução com um `commit`. Adicione os arquivos modificados
(`git add .`), faça o commit (`git commit -am "Sua mensagem de commit"` -
passando a opção `-a` você não precisa do `git add .`) e o
`git push`.
