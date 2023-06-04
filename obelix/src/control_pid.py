
import RPi.GPIO as GPIO     #biblioteca para a raspybarry  
from time import sleep      #biblioteca para trabalhar com o tempo

#Variáveis globais

contador = 0

#definição de pinos

PIN_AMARELO = 5    #fio amarelo, encoder A phase > pino GPIO 6
#PIN_VERDE = 5      #fio verde, encoder B phase > pino GPIO 5

def conta(channel):
    '''Função para contar'''
    global contador
    contador +=1
    #print("leitura do encoder- contador: ",contador)

#Setup
GPIO.setmode(GPIO.BCM)      # Numeração nomes GPIO 

GPIO.setup(PIN_AMARELO,GPIO.IN)

GPIO.add_event_detect(PIN_AMARELO,GPIO.RISING,callback=conta)

try:

    while True:
        
        sleep(1)
        print("pulsos/seg: ", contador)
        contador = 0

except KeyboardInterrupt:

    GPIO.cleanup()