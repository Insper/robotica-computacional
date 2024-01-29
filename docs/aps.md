# Configuração da APS

Todas as entregas da disciplina serão feita via Github Classroom. Para cada APS será enviado um link com o convite do Github Classroom.
As APSs são em duplas, então um aluno vai criar a equipe para cada APS e o outro deve entrar na equipe já existente **- NÃO SE ESQUEÇA DE ENTRAR NA SUA EQUIPE! -**.

Ao clicar no link de convite você será direcionado para a página do Github Classroom, com as opções de criar uma equipe ou entrar em uma já existente, como na imagem abaixo.

![](figs/github-classroom.png)

Podem escolher qualquer nome para a equipe, mas é importante que os dois membros da dupla estejam na mesma equipe. Após criar a equipe, espere um momento que você será direcionado para um repositório privado, onde será feita a entrega da APS.

Se você já completou o tutorial de configuração do git e gerou sua chave SSH, pode clonar o repositório lembrande de trocar para SSH no link do repositório. Se ainda não fez, siga o tutorial abaixo.
 
[Configure o seu Git](modulos/01-intro/atividades/guias-infra/ssd-linux/git-e-github/index.md)

Uma vez que você tenha clonado o repositório, entre no arquivo `README.md` e adicione o nome dos dois membros da dupla no local indicado. Após isso, faça um commit e um push para o repositório. A partir desse momento, vocês já podem começar a trabalhar na APS.

!!! warning
    Pode acontecer do professor pedir para que vocês atualizem o repositório com as últimas alterações. Para isso, basta seguir o tutorial [pull_template](modulos/01-intro/atividades/guias-infra/ssd-linux/git-e-github/pull_template.md)

## Configuração do Pacote (ROS 2)

Para as APSs que utilizam o ROS 2, você deve primeiramente aceitar o convite para o Github Classroom e clona-lo **dentro da pasta `colcon_ws/src/`** do seu SSD. E então, **dentro do diretório do seu repositório**, crie um novo pacote chamado `entregavel_#`, onde `#` é o número da APS. Você vai aprender a criar um pacote no módulo 2.