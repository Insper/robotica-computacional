# Como mudar a Raspberry Pi de Rede

## No laptop
No seu laptop, que deve estar na rede antiga, conecte-se à Rasperry Pi
	ssh pi@$IPBerry

## Na Raspberry Pi
Uma vez no terminal da Raspberry Pi, digite o seguinte comando para editar a configuração de rede:

	sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

O editor nano deverá se abrir. Você deve editar a configuração de rede para dar mais prioridade à sua rede desejada. 

Caso seu wpa_supplicant.conf só tenha uma rede, basta trocá-la para sua rede desejada sem mexer em prioridade.

Para sair do nano aperte `Crtl` junto com `X`. Ele perguntará se você deseja salvar, escolha a opção de salvar.

Ainda na Raspberry Pi, para reiniciar digite:

	sudo reboot


## No laptop novamente

Agora mude seu laptop para a mesma rede da Raspberry Pi e  prossiga novamente.





