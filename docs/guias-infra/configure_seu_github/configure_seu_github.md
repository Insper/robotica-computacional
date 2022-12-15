# Configure o seu GitHub

Boas noticias, não precisa instalar o GitHub, deixamos isso pronto pra você, porém, algumas configurações só você pode fazer, te mostramos neste guia quais são os passos para fazer as configurações necessárias, bom proveito!

## Se identificando para o GitHub

A primeira coisa que devemos fazer antes de começar a usar o GitHub é configurar o **nome de usuário** e **endereço de e-mail**. Isto é importante porque cada *commit*  usa esta informação, e ela é carimbada de forma imutável nos seus *commits*:

Abra um terminal usando o atalho Ctrl + T, então use os comando a seguir para configurar o seu user name e seu email:

```bash
git config --global user.name "Seu Belo Nome Aqui"
git config --global user.email SeuBeloEmailAqui@exemplo.br
```

💡Não precisa colocar o seu login do GitHub ao configurar o user.name, pode ser o seu nome sem problemas

Você precisará fazer a configuração de usuário e email somente uma vez, por que estamos usado a opção `--global`, o Git usará esta configuração para qualquer coisa que você fizer no Git em qualquer parte do sistema.
Se você precisar substituir os dados de usuário e email para um projeto específico, você pode rodar o mesmo comando **sem a opção** `--global` dentro do projeto específico, então a configuração será local e só vai funcionar naquele espaço.

### Seu Editor

Agora que a sua identidade está configurada, você pode escolher o editor de texto padrão que será chamado quando Git precisar que você entre uma mensagem. Se não for configurado, o Git usará o editor padrão, que normalmente é o nano.
Se você quiser usar um editor de texto diferente, como o Visual Code, você pode fazer o seguinte:

```bash
git config --global core.editor code
```

## Criando um **personal access token**

Atualmente, o GitHub oferece suporte a dois tipos de tokens, nós vamos de clássico. 

para saber detalhes e diferenças entre os tipos de tokens do GitHub e porque eles existem (Spoiler > é por segurança) consulte a [documentação aqui](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) 

Na página do seu GitHub, clique na sua foto e depois encontre a opção “Configuração” ou **“Settings”**

![Untitled](imgs/Untitled.png)

Em seguida, **Na barra lateral esquerda**, lá embaixo, clique em **Developer settings.**

![Untitled](imgs/Untitled1.png)

Clique em **Personal access tokens,**

![Untitled](imgs/Untitled2.png)

Depois, em Tokens (classic)

![Untitled](imgs/Untitled3.png)

Finalmente, clique em **Generate new token (classic)**

![Untitled](imgs/Untitled4.png)

💡Você pode criar quantos tokens você quiser, apenas certifique-se de salvar a chave hash com carinho.

Dê um nome para o seu token, como você pode criar vários, ter um bom nome ajuda com a organização.

![Untitled](imgs/Untitled5.png)

Role a página até o final e encontre o botão Generate token

![Untitled](imgs/Untitled6.png)

Salve com muito carinho o token gerado, ele **não vai aparecer novamente,** se você perder o token, será necessário criar outro.

![Untitled](imgs/Untitled7.png)

É possível salvar o seu token no gerenciador de credenciais do git.
💡Você precisa estar dentro de um repositório git para conseguir armazenar a sua credencial.

Na primeira vez que você fizer um push para um repositório remoto, o git solicitará suas credenciais, como nome de usuário e seu token
Da próxima vez, ele vai usar o mesmo token, que permanecerá armazenado com segurança em seu Gerenciador de Credenciais dentro do repositório, basta abrir um terminal e executar o comando

```bash
git config credential.helper store
```

```bash
git push http://example.com/repo.git
```

`Username: SeuUserNameAqui`

`Password: SeuTokenAqui`

Pronto, após essa configuração, não será mais necessário utlizar o token explicitamente para autenticar os seus commits, ele estará armazenado no gerenciador de credenciais do Git e será usado automaticamente.

Recomendamos fortemente que você se inscreva na atividade complementar de Git para aprender a usar essa ferramenta, vamos utilizar muito daqui pra frente.

Se quiser entender melhor como funciona o sistema de gerenciamento de credenciais do Git, a documentação é essa [aqui](https://git-scm.com/docs/git-credential-store).

Se quiser se começar a entender como funciona o GitHub, comece por [aqui](https://docs.github.com/en/get-started/quickstart).