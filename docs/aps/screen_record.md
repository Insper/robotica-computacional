# Como eu gravo a tela no linux?

Em diversas situações será necessário gravar a sua tela do linux. Existem inúmeros aplicativos para esta função, mas se você for gravar a tela inteira, não precisa instalar nenhum aplicativo extra, basta usar um recurso de gravaçãod e tela do próprio Ubuntu. 

Vamos agora entender como fazer:

## Alterando o tempo máximo de gravação

Antes de iniciar a gravação do vídeo, será preciso alterar o **tempo máximo** de gravação padrão do sistema que é de **30s**.

Você deve usar o seguinte comando: 
```
gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 0
```

Se você quer limitar o vídeo em um tempo específico, substitua o **0** pelo tempo que deseja.

## Iniciando a gravação

Para iniciar a gravação, basta pressionar as teclas **Ctrl + Alt + Shift + R** simultaneamente. Aparecerá o ícone da gravação no canto superior direito, como mostra a figura a seguir.

![Untitled](imgs/record_icon.png)


## Terminando a gravação

Para terminar sua gravação, você deve utilizar as mesmas teclas de atalho: **Ctrl + Alt + Shift + R**. Você perceberá que o ícone de gravação irá sumir. 

![Untitled](imgs/without_icon.png)


## Em que pasta está a gravação

Todas as gravações de tela ficam armazenadas na pasta padrão de vídeos do sistema, você a encontra no Nautilus, o navegador de arquivos. E a nomenclatura conterá a data e horário da gravação.

![Untitled](imgs/folder_Videos.png)


Equipe 404