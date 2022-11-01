

# Como desabilitar os prints do SMACH

Frequentemente queremos ver algum `print` feito pelo nosso programa que está perdido em meio a dezenas de saídas do SMACH.

Este guia explica como desabilitar a saída do SMACH.
 

Edite o arquivo:

No `nano` faça:

	sudo nano /opt/ros/kinetic/lib/python2.7/dist-packages/smach_ros/__init__.py

Ou no `sublime`:

	sudo subl /opt/ros/kinetic/lib/python2.7/dist-packages/smach_ros/__init__.py


Localize as seguintes linhas (no screen `Ctrl` `C` mostra a linha atual):

42	# Setup smach-ros interface
43	smach.set_loggers(
44	        rospy.loginfo,
45	        rospy.logwarn,
46	        rospy.logdebug,
47	        rospy.logerr)


Altere na linha 44 de `rospy.loginfo` para `rospy.logdebug`, ficando portanto:

42	# Setup smach-ros interface
43	smach.set_loggers(
44	        rospy.logdebug,
45	        rospy.logwarn,
46	        rospy.logdebug,
47	        rospy.logerr)

Dê `Ctrl` `X`, em seguida escolha a opção de salvar.

Para voltar a ver as impressões faça a mudança reversa


