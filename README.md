Sistemas de Control en Tiempo Real de Célula Peltier
-

El objetivo del proyecto es realizar un control de la temperatura de la superficie de una célula Peltier mediante un PID digital dentro de una placa STM32F429I-DISC1.

Además desde la pantalla de la placa se podrá cambiar los valores de las 3 constantes del PID y la temperatura objetivo, y desde una conexión USB se enviarán los datos a un sofware en PC para graficar el historial de la temperatura.

Este proyecto ha sido realizado para la asigantura de Sistemas de control en tiempo real de 4º curso del Grado de Ingeniería en Electrónica Industrial y Automática en colaboración con mi compañero Marcos.

Materiales
-

- Placa STM32F429I-DISC1
- Sensor LM35
- 2 Relés 
- Transistor MOSFET IRL540
- Programa STM32CubeIDE
- Programa QT Creator

Esquema conexión Peltier
-

![conexión Peltier](https://github.com/Hemonel/STM32F429I-control-Peltier/assets/153218898/8f217b4b-80a3-4710-83bd-a1dd885b2843)

Software en placa STM32
-

La función de la placa es leer la temperatura de la placa (mediante un conversor analógico-digital incluido en el microcontrolador), realizar un cálculo PID, según el valor calculado la célula peltier debe
calentar, enfriar o no hacer nada y finalmente mostrar en la pantalla el valor de temperatura actual y las constantes del PID.

- Control de temperatura

  El programa aquí descrito se encuentra dentro de la función temporizadorTick (líneas 1020 a 1095 en main.c).

  Lo primero es un if-else que activa o desactiva la célula peltier según en que parte del ciclo se esté, de esta forma la célula no esta activa siempre cuando se quiere calientar o enfriar, generandose
  así una PWM que permite tener un control total del calor que transporta la célula.

  Lo siguiente es la lectura de la temperatura y cálculo de la actuación, se realiza cada 3 ciclos.
  
    1º Solicita la conversión de analógico a digital.
  
    2º Cuando tiene el valor realiza el calculo de cual es la temperatura real (valor medido x 3,3Vmax / 10mV/ºC /2^12), en el programa está como medida*330/4096.

  
