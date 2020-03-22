# happyfeet
O Happy é um robô móvel dotado de sensores e controlado por um Arduino! Com este robô, temos a possibilidade de realizar tarefas em um ambiente inteligente! O objetivo é a criação de um espaço inteligente no IFES Guarapari e o primeiro passo é ter uma plataforma robótica que consiga executar tarefas móveis neste espaço.

# Estrutura física
A estrutura física do robô é composta por dois andares feitos de acrílico. Os arquivos de CAD com os cortes e furos estão na pasta CAD deste projeto.
As peças projetadas em 3D foram impressas utilizando PLA e podem ser encontradas na pasta Pecas_3d deste projeto.

<img src="https://github.com/Penguin-Lab/happyfeet/blob/master/images/happy0.jpg" width="300"> <img src="https://github.com/Penguin-Lab/happyfeet/blob/master/images/happy1.jpg" width="300"> <img src="https://github.com/Penguin-Lab/happyfeet/blob/master/images/happy2.jpg" width="300"> <img src="https://github.com/Penguin-Lab/happyfeet/blob/master/images/happy3.jpg" width="300">

## Placas de circuito impresso
No projeto, foram desenvolvidas duas placas de circuito:
* Shield para o Arduino Mega e sensores
* Placa para a ESP8266 12

<img src="https://github.com/Penguin-Lab/happyfeet/blob/master/images/shield_arduino_mega_cut.png" width="300"> <img src="https://github.com/Penguin-Lab/happyfeet/blob/master/images/esp_cut.png" width="220">

## Componentes eletrônicos
Os componentes eletrônicos embarcados no robô e equipamentos utilizados são:
* Controlador
  * Arduino Mega
* Módulo Wi-fi
  * ESP8266 12E
  * Conversor step down 5V/3,3V
  * Display OLED
* Motores
  * Motores 6V com encoder
  * Módulo Ponte H L298n
* Sensores
  * FPV Camera System 3-IN-1 antenna Cloverleaf
  * Módulo ultrassom HC-SR04
* Bateria
  * Bateria Zippy 1100mAh 2s 6,6v
  * Carregador de bateria iMax B6 80W

# Software
Na pasta de códigos, há o código de programação do Arduino Mega, da ESP8266 12 (podendo-se utilizar o NodeMCU no lugar), e um exemplo em Python de controlador de posição final com offset. Esse código em Python que se comunica com o robô por meio do protocolo MQTT, utilizando como broker o Mosquitto.

# Exemplos
## Exemplo para conferir a odometria vinda do robô (terminal do ubuntu):
`mosquitto_sub -t happy/odometry`

## Exemplo para conferir as velocidades publicadas no tópico que o robô consome (terminal do ubuntu):
`mosquitto_sub -t happy/velocities`

## Exemplo para publicar velocidades no tópico que o robô consome (terminal do ubuntu):
`mosquitto_pub -t happy/velocities -m <1;100.00;-20.00>`

## Exemplo para executar o controlador de posição final com offset (terminal do ubuntu):
`python controller.py`
