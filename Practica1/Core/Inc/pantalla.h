
#ifndef INC_PANTALLA_H_
#define INC_PANTALLA_H_


#include <stdint.h>  // Para uint8_t, uint16_t y uint32_t
#include <string.h>  // Para strlen()
#include "stm32f429i_discovery.h"  // Para recursos de manejo de la placa
#include "stm32f429i_discovery_lcd.h"  // Para recursos de manejo de la pantalla
#include "stm32f429i_discovery_ts.h"  // Para interfaz táctil


#define LCD_FRAME_BUFFER_LAYER_0 (LCD_FRAME_BUFFER)
#define LCD_FRAME_BUFFER_LAYER_1 (LCD_FRAME_BUFFER+0x130000)
// Direcciones de memoria donde residen los dos frame buffers de la pantalla


typedef struct {
	uint16_t ancho, alto;  // Ancho y alto de la pantalla en puntos
	uint32_t* buffers[2];  // Direcciones de memoria donde residen los frame buffers
	int bufferDibujo;  // Buffer oculto donde se dibuja, el 0 o el 1
} PantallaLCD;
// Estructura con información sobre la pantalla LCD.


typedef struct {
	uint8_t ancho, alto;  // Ancho y alto en puntos de cada caracter
	uint8_t separacion;  // Puntos de separación en horizontal entre caracteres
	uint8_t nBytesPorCaracter;  // Número de bytes utilizados para describir cada carácter
	uint32_t color;  // Color de escritura de caracteres
	const uint8_t* pCaracteres;  // Dirección de memoria donde se describen los caracteres
} JuegoCaracteres;
// Estructura que representa a un juego de caracteres


void inicializaPantalla2Buffers(uint32_t colorFond,const uint8_t * imagen, PantallaLCD* pantallaLCD);
// Inicializa la pantalla para trabajar con 2 frame buffers. Pone el color de todos los puntos
// según el parámetro 'colorFondo'. En 'pantallaLCD' se indica la dirección de una estructura
// donde se guarda información sobre la pantalla.


void intercambiaBuffersLCD(PantallaLCD* pantallaLCD);
// Cambia en qué frame buffer se dibuja y qué frame buffer se visualiza


void inicializaJuegoCaracteres(uint8_t ancho, uint8_t alto, uint8_t separacion, uint32_t color,
	const uint8_t* pCaracteres, JuegoCaracteres * pJuegoCaracteres);
// Inicializa la estructura apuntada por 'pJuegoCaracteres' para indicar que se están utilizando
// caracteres de 'ancho' puntos de ancho, 'alto' puntos de alto, con una separación horizontal
// entre ellos de 'separacion' puntos y que se dibujan con un color. En 'pCaracteres' se indica
// dónde se guardan los bytes que describen cómo hay que dibujar los caracteres.


void dibujaCaracter(uint16_t x, uint16_t y, char caracter, JuegoCaracteres * pJuegoCaracteres,
	PantallaLCD * pPantalla);
// Dibuja el carácter indicado en 'caracter' de forma que su esquina superior izquierda
// corresponda a las coordenadas ('xCaracter', 'yCaracter') utilizando el juego de caracteres
// descrito en la estructura apuntada por 'pJuegoCaracteres'.


void dibujaCadenaCaracteres(uint32_t xCadena, uint32_t yCadena, char* cadena,
	JuegoCaracteres * pJuegoCaracteres, PantallaLCD * pPantalla);
// Dibuja la cadena de caracteres apuntada por 'cadena' de forma que la esquina superior izquierda
// del primer carácter se indica en 'xCadena' e 'yCadena'. Los caracteres se visualizan según
// el juego de caracteres expresado en la estructura apuntada por 'pJuegoCaracteres'.


void dibujaPunto(uint16_t x, uint16_t y, uint32_t color, PantallaLCD * pPantalla);
// Dibuja un punto en coordenadas ('x', 'y') con el color indicado en 'color'


void dibujaLinea(int x0, int y0, int x1, int y1, uint32_t color, PantallaLCD *pPantalla);
// Dibuja una línea que une el punto indicado en 'x0', 'y0' con el punto indicado en 'x1', 'y1'
// en el color 'color' utilizando el algoritmo de Bressenham
// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm


void dibujaCirculo(int xc, int yc, int r, uint32_t color, PantallaLCD * pPantalla);
// Dibuja un círculo cuyo centro se indica en 'xc', 'yc', de radio 'r' y con el color
// indicado en 'color' utilizando el algoritmo de Bressenham
// https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/


void dibujaRectangulo(uint16_t xInicial, uint16_t yInicial, uint16_t ancho, uint16_t alto,
	uint32_t color, PantallaLCD * pPantalla);
// Dibuja un rectángulo cuya esquina superior izquierda está situada en (xInicial, yInicial) y
// donde se indica el alto y ancho el color de sus puntos


void dibujaImagen(uint16_t xImagen, uint16_t yImagen, uint16_t ancho, uint16_t alto,uint16_t transparencia,
	const uint8_t * imagen, PantallaLCD * pPantalla);
// Dibuja una imagen situando su esquina superior izquierda en 'xImagen', 'yImagen', que tiene
// una resolución de 'ancho' puntos en horizontal y 'alto' puntos en vertical. Los bytes que
// expresan el color de todos sus puntos (cada punto en 4 bytes en formato ARGB)
// se encuentran en 'imagen'.


#endif /* INC_PANTALLA_H_ */
