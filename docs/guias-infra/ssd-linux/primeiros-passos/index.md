# Meu SSD subiu, e agora?

Assumindo que você seguiu o [tutorial anterior](../recebi-meu-ssd/index.md), este guia detalha como: 

**1.** Fazer o login

**2.** Trocar a senha

**3.** Configurar a rede

**4.** Atualizar o Sistema

## Fazendo Login

Se tudo deu certo até agora, você encontrará esta imagem:

![Untitled](imgs/img1.png){width="800"}

Clique no pinguin - Usuário **borg** - e coloque a senha inicial: **fl1pfl0p**

![Untitled](imgs/img2.png){width="800"}

Você encontrará este ambiente de trabalho

![Untitled](imgs/img3.png){width="800"}

## Trocando a Senha

A primeira configuração que recomendamos é trocar esta senha inicial. Temos duas formas de realizar esta tarefa, pelo ambiente gráfico e pelo terminal. Vamos ver as duas formas de realizar, iniciando pelo ambiente gráfico.

Clique no canto superior direito, em seguinda em settings.

![Untitled](imgs/img4.png){width="800"}

Na barra da esquerda, escolha o item Users e clique no botão no canto superior direito chamado Unlock.

![Untitled](imgs/img5.png){width="800"}

Ele pedira a senha de usuário (ainda é fl1pfl0p). Depois basta clicar em Password. Ele pedira a senha atual e a nova senha. Lembre-se que precisa ser uma senha forte, ou seja, ter letras maiúsculas, minúsculas, números e símbolos. Ter seu comprimento superior ou igual a 8 dígitos.

![Untitled](imgs/img6.png){width="800"}

Após preencher a nova senha e confirmar a nova senha, se ela estiver dentro dos padrões de segurança, o botão Change ficará habilitado.

![Untitled](imgs/img7.png){width="800"}
A partir de agora todas as vezes que for pedida a senha de usuário você deve usar a nova senha. É recomendado que reinicie a máquina para que o ambiente gráfico utilize a nova senha.

A outra forma de fazer a mudança de senha é pelo terminal. Para isso, você pode abrir o terminal (veja dois ícones de terminais na barra de atalhos). Você pode clicar no programa com o mouse ou usar as teclas de atalho SUPER+T **(Super é a tecla do windows).**

Com o terminal aberto (não importa qual dos dois você escolheu), você digitará o comando: `passwd` em seguida forneca a **senha atual** (fl1pfl0p) e depois a **nova senha**, lembrando que precisa ter no mínimo 8 dígitos, usar letras maíusculas e minúsculas, números e símbolos. Ele pedirá para **confirmar a nova senha** e pronto! Sempre que precisar trocar a senha, refaça este procedimento.

!!! dica
    É normal não aparecer nada no terminal quando você digita a senha, é uma medida de segurança do terminal esconder o display de caracteres.

![Untitled](imgs/img8.png){width="800"}

## Configurando a Rede

Esta etapa não é difícil mas requer atenção em alguns detalhes. Você precisará usar uma rede diferente para esta disciplina, chamada **Robotica**. É nesta rede que nossos robôs se conectam para serem controlados. A credencial desta rede é a mesma que você usa na rede **insper_alunos** e as configurações de conexão são iguais também. Então após este tutorial você estará apto a logar em qualquer rede do INSPER com sua máquina.

O primeiro passo é clicar nos ícones no canto superior direito, depois clicar no item da Wi-fi seguido de “Select Network”, como visto na imagem abaixo.

![Untitled](imgs/img9.png){width="800"}

Na tela que se abrirá, escolha a rede que deseja conectar e clique em “Connect”.

![Untitled](imgs/img10.png){width="800"}

Então você verá a tela de configurações de rede. São três informações que você deve passar:

**1.** Clicar (”check”) na caixa: No CA certificate is required

**2.** Username: seu usuário de rede (e-mail sem @al.insper.edu.br)

**3.** Password: senha do seu e-mail.

Depois clique em “Connect”.

![Untitled](imgs/img11.png){width="800"}

Após a conexão, você verá o ícone do Wi-Fi no canto superior direito da tela:

![Untitled](imgs/img12.png){width="800"}

## Atualizando o Sistema

Vamos abrir o terminal usando as teclas de atalho SUPER+T (lembrando que a tecla super é como a tecla “windows” é chamada no linux).

Em nosso SSD você verá a tela do Terminator após usar este atalho. Na imagem abaixo você verá o terminator aberto e o primeiro comando que iremos utilizar. 

![Untitled](imgs/img13.png){width="800"}

O primeiro comando que iremos utilizar,  `sudo apt update`,  é dividido em três partes. A primeira palavra **sudo** diz ao sistema que você usará permissão de **root** (administrador do sistema) para executar o comando a seguir, o comando **apt** chama o gerenciador de pacotes do ubuntu, que espera uma sequência de comandos, e por fim a palavra **update** que diz ao apt que precisamos atualizar as bases dos repositórios para saber se temos alguma atualização. Como estamos usando permissão de root, pedirá a senha de usuário a seguir. 

Com a mesma premissa, vamos atualizar os pacotes que ele encontrou com novas versões. para isto utilize o comando `sudo apt upgrade`.  Este comando perguntará, a cada pacote, se você quer ou não instalar, basta teclar ENTER se quiser instalar. Se você não quer confirmar pacote a pacote, pode acrescentar o “-y” no comando, já enviando antecipadamente o “yes”, então comando ficaria assim: **sudo apt upgrade -y**.

Podemos concatenar os dois comandos utilizando `&&` no terminal. Desta forma podemos executar o update e o upgrade na mesma linha, como segue: `sudo apt update && sudo apt upgrade -y`.

!!! dica
    Use o TAB para autocompletar os comandos no terminal

## Finalizando

Se você conseguiu seguir todos os passos até agora, o seu SSD já está pronto para ser utilizado. Como você utilizará linux em outras etapas do curso, a dica é usar o SSD para tudo neste semestre.
