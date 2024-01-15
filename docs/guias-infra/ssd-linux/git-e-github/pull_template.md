# Pull changes from a template repository


**Siga este tutorial apenas quando o professor alertar alguma modificação de uma tarefa, durante a aula, ou em um aviso no blackboard.**


Uma atividade do classroom se baseia em um repositório base, denominado de template repository, se o aluno aceitou a atividade no classroom antes do professor fazer alguma modificação no template, o aluno não terá acesso às modificações. Neste tutorial vamos aprender como pegar (fetch) modificações da atividade do classroom.


## Passo 1 - URL do template repo


Nas atividades de robótica, a URL estará disponível no README da atividade.
Em outros casos, abra a página do seu repositório no seu navegador. Em baixo do nome do repositório terá um texto do tipo: *generated from {link}*, clique no link.
Pegue o link do repositório no botão **<> Code**, esse é o link do repositório template, se estiver usando autenticação por SSH, pegue esse link.


## Passo 2 - fetch do template repo
Agora execute os seguinte comandos, lembre-se de colocar o link obtido anteriormente,


```bash
git remote add template [URL of the template repo]
git fetch --all
git merge template/main --allow-unrelated-histories
```


## Passo 3 - Ajustando conflitos
Se voce tiver alguma alteração, receberá um alerta de conflito.
Abra o VSCode, entre nos arquivos que apresentam conflitos e clique no botão **Resolve in Merge Editor**.
Neste EDITOR `Incoming` são as alterações vindas do repositorio templates e `Current` são as alterações feitas no seu repositório
Você pode:
* aceitar um ou ambos.
* Ignorar as modificações, ou seja rejeitar.
* Fazer alguma outra modificação


Quando estiver pronto, clique no botão **Complete Merge** e de commit nas suas modificações,
```bash
git add .
git commit -m "Merge remote-tracking branch 'template/main' into main"
git push
```

