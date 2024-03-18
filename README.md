# Sistemas de Control en Tiempo Real de Célula Peltier

El objetivo del proyecto es realizar un control de la temperatura de la superficie de una célula Peltier mediante un PID digital en una placa STM32F429I-DISC1.

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

Este programa se encuentra en practica2\programaSTM.

### Programa principal (main.c)

Este archivo contiene las funciones para atender la pantalla táctil y para el control de la célula peltier.

- Inicialización
  
  Desde la línea 759 hasta la línea 909 se definen los valores iniciales de las variables y lo que se debe hacer cuando se pulse cada botón.

- Control de temperatura
  
  El programa aquí descrito se encuentra dentro de la función temporizadorTick (líneas 1020 a 1095 en main.c).
  
  Lo primero es un if-else que activa o desactiva la célula peltier según en que parte del ciclo esté, de esta forma la célula no esta activa siempre cuando se quiere calientar o enfriar, generandose
  así una PWM que permite tener un control total del calor que transporta la célula.
  
  Lo siguiente es la lectura de la temperatura y cálculo de la actuación, se realiza cada 3 ciclos.
    
  - 1º Solicita la conversión de analógico a digital.
    
  - 2º Cuando tiene el valor realiza el calculo de cual es la temperatura real (valor medido x 3,3Vmax / 10mV/ºC /2^12), en el programa está como medida*330/4096.
  
  - 3º Calcula el error y realiza el cálculo de PID como (Kp*error)+(Ki*Ts*0.1*sumaError)+(Kd*Ts*0.1*(error-errorAnt)).
  
  - 4º Si el valor calculado es mayor que 1 enfría, si es menor que -1 calienta y si está entre esos valores no hacce nada.
  
  - 5º Ultiliza el valor calculado en el PID para establecer el tiempo que debe estar activada la célula peltier (entre 0 y 100%).
  
  Finalmente envía por el puerto USB los valores de las constantes del PID, la temperatura medida y la temperatura objetivo.

### Gestión de pantalla (pantalla.c)

Este archivo contiene todas las funciones necesarias para gestionar la pantalla, por ejemplo dibujar una imagen o comprobar si la pantalla fue tocada.

Funciones destacables:

- Inicializado de la pantalla (inicializaPantalla2Buffers)
 
  Algo importante a destacar es que el sistema de funcionamiento es con 2 frame buffers, de forma que cada ciclo máquina todo lo nuevo que se quiera dibujar se dibuja en el buffer que no se está mostrando
  y al terminar el ciclo, de esta forma se evita que se vea como se van dibujando los nuevos componentes sino que aparecen de golpe.

  La función crea los 2 buffers y dibuja el fondo de pantalla (un color sólido o una imagen).

- Comprobar si la pantalla fue tocada (pantallaPulsada)
    
  Devuelve un 1 si la pantalla fue pulsada y da las coordenadas.

- Inicializado de botón (inicializaBoton)
 
  Guarda los datos del botón (coordenadas, ancho, alto, la imagen a mostrar y la función que debe activar) dentro de las variables de la estructura generada para ese botón y lo dibuja.

- Inicializado de juego de caracteres (inicializaJuegoCaracteres y inicializaJuegoCaracteresAlpha)
 
  Algo muy importante es que los carácteres deben ser dibujados píxel a píxel como una imagen y para eso se han creado los archivos juegoCaracteres11x16.c y juegoCaracteres8x11.c donde se definen
  las dimensiones y las caracteristicas de cada carácter. Esta función registra esos juegos de caracteres, la separación que debe haber entre ellos y el color.

Sofware para graficado
-

A traves del puerto USB se reciben los datos de de las constantes del PID, la temperatura medida y la temperatura objetivo. Estos datos se muestran en una intefaz de QT, además la temperatura real y la objetivo se muestran en una gráfica respecto al tiempo.

Este programa se encuentra en practica2\receptor.

- Inicialización y recepción de datos (mainwindow.cpp)

  Inicialmente establece cual es el puerto serie con el que se debe comunicar y define la interfaz.

  Tras esto lee los datos recibidos como una cadena, separa los distintos valores de la cadena y los manda dibujar.

- Dibujado de la grafica (variasgraficas.cpp)

  La función de este programa es establecer las caracteristicas de las gráficas e ir actualizándolas.

Anotaciones
-

Este proyecto se realizó en 2 fases, todo lo descrito anteriormente es parte de la práctica 2, sin embargo está disponible la práctica 1 para su curiosidad y tambien se incluyen los pdfs con los objetivos de la asigantura para ambas fases.

Se incluyen manuales de la placa STM32F429I-DISC1 y del sensor LM35.
