# Configuração da APS

Todas as entregas da disciplina serão feitas via GitHub.

## Entregas via GitHub

* **Arquivos da APS:** Para cada APS, um repositório com os arquivos iniciais será disponibilizado.

  * Acesse o repositório informado no Blackboard, clique em **Code → Download ZIP** e extraia os arquivos no seu computador.
  * Depois, crie um **novo repositório privado** na sua conta do GitHub.
  
  * No caso de atividades em grupo, o repositório deve ser criado por **apenas um dos integrantes**.
  * No caso de atividades em grupo, os demais integrantes devem ser adicionados como colaboradores no repositório.

* **Escolha do Nome do Repositório:** O nome do repositório deve seguir o padrão `robcomp_aps#_XXXX`, substituindo `#` pelo número da APS e `XXXX` pelo nome do grupo. Use apenas letras minúsculas, números, hífens ou sublinhados, sem espaços e sem acentos. Exemplo: `robcomp_aps1_wall-e`.

* **Configuração do Repositório:**

  * No campo **Visibility**, escolha **Private**.
  * Marque a opção **Add a README file**.
  * Em **Add .gitignore**, escolha o modelo **Python**.
  * Não escolha uma licença.
  * Depois, clique em **Create repository**.

### Acesso ao Repositório

Após a criação do repositório, adicione o usuário GitHub do professor `dsoldev` como colaborador. Caso seja uma atividade em grupo, adicione também os demais integrantes.

Para isso, acesse **Settings → Collaborators**, clique em **Add people** e adicione os usuários GitHub correspondentes.

## Configuração do Repositório

* **Clonagem do Repositório:** Se já completou o tutorial de configuração do git e gerou sua chave SSH, clone o novo repositório optando por SSH. Caso contrário, siga o [tutorial de configuração do Git](modulos/01-intro/atividades/guias-infra/ssd-linux/git-e-github/index.md).
* **Cópia dos Arquivos:** Copie para dentro do novo repositório todos os arquivos e diretórios extraídos do ZIP da APS. Certifique-se de copiar também os arquivos ocultos e, caso seja solicitado, substitua os arquivos existentes.
* **Primeiro envio:** Adicione o nome de todos os integrantes do grupo no arquivo `README.md`. Em seguida, adicione os arquivos copiados ao Git, faça um commit e envie-o para o GitHub com `git push`. Após essas etapas, podem iniciar o trabalho na APS.
* **Inclusão de Colaboradores:** Em **Settings → Collaborators**, adicione:

  * os usuários GitHub dos demais integrantes do grupo;
  * o usuário GitHub do professor: `dsoldev`.

## Configuração do Pacote (ROS 2)

* **Preparação Inicial:** Clone o novo repositório **dentro da pasta** `colcon_ws/src/` no seu SSD e copie para ele os arquivos baixados do repositório da APS.
* **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_#`, substituindo `#` pelo número da APS correspondente. A criação de um pacote será ensinada no módulo 2.

## Entrega

No Blackboard, envie:

* o link do repositório;
* o SHA completo do commit que deve ser avaliado.

Para obter o SHA do commit mais recente, execute:

```bash
git rev-parse HEAD
```

A correção será realizada com base no commit informado. O horário oficial da entrega será o registrado no Blackboard, e não o horário do commit. 

!!! warning
    A correção será feita considerando o histórico até o commit informado, e não necessariamente o último commit do repositório. Commits posteriores não serão considerados. Portanto, é importante informar no Blackboard o SHA do último commit que deseja entregar antes do fechamento da atividade.
