# Configuração da APS

Todas as entregas da disciplina serão feita via um repositório Git para cada aluno. Acesse abaixo para aceitar o convite e iniciar seu trabalho. 

[Link de convite](https://classroom.github.com/a/csDvCJXy){ .ah-button }

O trabalho será em duplas, então será necessário criar sua equipe para as APS. 

!!! warning
    As duplas serão as mesmas para todas APS. No projeto serão formados novos grupos.

## Primeiro acesso

Todo o código de suporte da disciplina está público no repositório [Robótica Computacional APS]({{ repo_aps }}). Nesse guia iremos configurar seu repositório privado para acompanhar esse repo público.

!!! important
    Você deve ter recebido um repo novo privado. Copie o endereço do seu repo abaixo. Ele será usado no restante do guia. 

    ![](endereco-repo-privado.png)

Para começar, crie uma pasta nova para seu repositório de entregas e inicialize um repo vazio:

```bash
mkdir entregas-robotica
cd entregas-robotica
git init
```

Primeiro vamos adicionar o repositório remoto dos arquivos de suporte e baixar o branch `main` (que contém os arquivos deste semestre)

```bash
git remote add insper {{ repo_aps_git }}
git fetch
git switch -c main
```

Agora vamos adicionar o repositório das suas entregas e já enviar o código de suporte:

```bash
git remote add entregas endereco_do_seu_repo_privado
git push --set-upstream entregas main
```

Pronto! Com isso você já deve ter seu repositório local configurado e apontando para dois repositórios remotos:

- **insper**: este repo contém todo o código de suporte para as atividades. É compartilhado por toda a sala e ninguém tem permissão de dar push nele.
- **entregas**: este repo é só seu e contém seu trabalho apenas. Aqui irão somente as modificações feitas por você :)

Você pode checar se tudo deu certo rodando `git branch -avv`. Você deve ver algo parecido como o abaixo:

```bash
deck@ubuntu-dev:~/Documents/entregas-robotica$ git branch -avv
* main                  7329318 [entregas/main] Add readme.md
  remotes/entregas/main 7329318 Add readme.md
  remotes/insper/main   7329318 Add readme.md
```

!!! exercise
    Para testar, edite o arquivo `README.md` e 
    
    - adicione seu nome no local indicado
    - faça um commit
    - envie as modificações para seu repositório privado com `git push`

## Recebendo atualizações e novas APS

Ao longo do semestre será liberado código das novas APS e possíveis atualizações nos testes. Siga este guia para atualizar seus arquivos de suporte.

Vamos iniciar baixando as novidades do repositório de suporte:

```bash
git fetch insper
```

Vamos então incorporar as novidades no seu repositório local e enviar os novos arquivos pro seu repo privado. 

```bash
git switch main
git merge insper/main
git push
```

!!! exercise
    Agora é só verificar que seus commits aparecem no seu repositório privado. Você pode ver isso rodando `git log`. Algo como o abaixo deve aparecer. Também pode checar online se os commits estão no repositório do *Github*.

    ```bash
    commit f0a444787de5bcf6c869cfda043e069a20a3300a (HEAD -> main, entregas/main)
    Merge: 1996777 0f42021
    Author: Igor Montagner <igordsm@gmail.com>
    Date:   Tue Jan 31 11:50:35 2023 -0300

        Merge

    commit 1996777386d12aea6c2869e7943c06e6895e3b08
    Author: Steam Deck User <deck@ubuntu-dev.steamdeck>
    Date:   Tue Jan 31 11:47:54 2023 -0300

        Add meu nome

    commit 0f420212b147d5f2a60c652b645127f4276624e4 (insper/main)
    Author: Igor Montagner <igordsm@gmail.com>
    Date:   Tue Jan 31 11:46:28 2023 -0300

        Adicionando arquivos iniciais para APS01

    commit 73293182f28cd7f11796c5c01ccb7231d817554a
    Author: Igor Montagner <igordsm@gmail.com>
    Date:   Tue Jan 31 11:32:50 2023 -0300

        Add readme.md
    ``` 
