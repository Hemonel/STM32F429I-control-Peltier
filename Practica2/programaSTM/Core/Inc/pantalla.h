
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
	uint8_t solido;
	uint32_t fondoColor;
	uint32_t * fondoImagen;

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

typedef struct {
	uint8_t ancho, alto;  // Ancho y alto en puntos de cada caracter
	uint8_t separacion;  // Puntos de separación en horizontal entre caracteres
	uint16_t nBytesPorCaracter;  // Número de bytes utilizados para describir cada carácter
	const uint8_t* pCaracteres;  // Dirección de memoria donde se describen los caracteres
} JuegoCaracteresAlpha;

typedef struct {
	uint8_t ancho, alto;  // Ancho y alto en puntos de cada caracter
	uint16_t x,y;
	uint8_t habilitado;
	uint8_t visible;
	uint8_t *imagen;
	void (* pFuncion)(void);  // Dirección de memoria donde se describen los caracteres
	PantallaLCD *pantallaLCD;
} TsBoton;
void setvisiblebBoton(uint8_t En, TsBoton * botonReset);
void setHabilitacionBoton(uint8_t En, TsBoton * botonReset);
uint32_t rgb2gray(uint32_t color);
void atiendeBoton(uint16_t x,uint16_t y, uint16_t pulsada, TsBoton * botonReset);
void inicializaBoton(uint16_t xBoton,uint16_t yBoton, uint8_t ancho, uint8_t alto, const uint8_t * imagen ,void (* pFunc)(void), uint8_t vis,uint8_t En,TsBoton * botonReset, PantallaLCD *pantallaLCD);

void inicializaPantalla2Buffers(uint8_t solido,uint32_t colorFondo,const uint8_t * imagen, PantallaLCD* pantallaLCD);
// Inicializa la pantalla para trabajar con 2 frame buffers. Pone el color de todos los puntos
// según el parámetro 'colorFondo'. En 'pantallaLCD' se indica la dirección de una estructura
// donde se guarda información sobre la pantalla.

uint32_t mezclaColores(uint32_t fondo,uint32_t frente);

void intercambiaBuffersLCD(PantallaLCD* pantallaLCD);
// Cambia en qué frame buffer se dibuja y qué frame buffer se visualiza


void inicializaJuegoCaracteres(uint8_t ancho, uint8_t alto, uint8_t separacion, uint32_t color,
	const uint8_t* pCaracteres, JuegoCaracteres * pJuegoCaracteres);
// Inicializa la estructura apuntada por 'pJuegoCaracteres' para indicar que se están utilizando
// caracteres de 'ancho' puntos de ancho, 'alto' puntos de alto, con una separación horizontal
// entre ellos de 'separacion' puntos y que se dibujan con un color. En 'pCaracteres' se indica
// dónde se guardan los bytes que describen cómo hay que dibujar los caracteres.
void inicializaJuegoCaracteresAlpha(uint8_t ancho, uint8_t alto, uint8_t separacion,const uint8_t* pCaracteres, JuegoCaracteresAlpha * pJuegoCaracteres);
void dibujaCaracter(uint16_t xCaracter, uint16_t yCaracter, char caracter,
		JuegoCaracteres * pJuegoCaracteres, uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla);
// Dibuja el carácter indicado en 'caracter' de forma que su esquina superior izquierda
// corresponda a las coordenadas ('xCaracter', 'yCaracter') utilizando el juego de caracteres
// descrito en la estructura apuntada por 'pJuegoCaracteres'.

void dibujaCadenaAlpha(uint32_t xCaracter, uint32_t yCaracter, char *cadena,
		JuegoCaracteres * pJuegoCaracteresAlpha, uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla);

void dibujaCadenaCaracteresAlpha(uint32_t xCadena, uint32_t yCadena, char *cadena,
		JuegoCaracteresAlpha * pJuegoCaracteresAlpha, uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla);
void dibujaCadenaCaracteres(uint32_t xCadena, uint32_t yCadena, char* cadena,
		JuegoCaracteres * pJuegoCaracteres,uint8_t gray,uint8_t alfa, uint8_t fondo,  PantallaLCD * pPantalla);


// Dibuja la cadena de caracteres apuntada por 'cadena' de forma que la esquina superior izquierda
// del primer carácter se indica en 'xCadena' e 'yCadena'. Los caracteres se visualizan según
// el juego de caracteres expresado en la estructura apuntada por 'pJuegoCaracteres'.

void dibujaPunto(uint16_t x, uint16_t y, uint32_t color,uint8_t gray, uint8_t alfa, uint8_t frame,PantallaLCD * pPantalla);

// Dibuja un punto en coordenadas ('x', 'y') con el color indicado en 'color'


void dibujaLinea(int x0, int y0, int x1, int y1, uint32_t color,  uint8_t gray,uint8_t alfa,uint8_t fondo,PantallaLCD *pPantalla);
// Dibuja una línea que une el punto indicado en 'x0', 'y0' con el punto indicado en 'x1', 'y1'
// en el color 'color' utilizando el algoritmo de Bressenham
// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

uint16_t pantallaPulsada(uint16_t * x, uint16_t *y);
void dibujaCirculo(int xc, int yc, int r, uint32_t color, uint8_t gray,uint8_t alfa,uint8_t fondo, PantallaLCD * pPantalla);
// Dibuja un círculo cuyo centro se indica en 'xc', 'yc', de radio 'r' y con el color
// indicado en 'color' utilizando el algoritmo de Bressenham
// https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/


void dibujaRectangulo(uint16_t xInicial, uint16_t yInicial, uint16_t ancho, uint16_t alto,
	uint32_t color, PantallaLCD * pPantalla);
// Dibuja un rectángulo cuya esquina superior izquierda está situada en (xInicial, yInicial) y
// donde se indica el alto y ancho el color de sus puntos
void dibujaRectangulo2(uint16_t xInicial, uint16_t yInicial, uint16_t ancho, uint16_t alto,
	uint32_t color ,uint8_t gray,uint8_t alfa,uint8_t fondo ,PantallaLCD * pPantalla);

void dibujaImagen(uint16_t xImagen, uint16_t yImagen, uint16_t ancho, uint16_t alto,
	const uint8_t * imagen, uint8_t gray,uint8_t alfa,uint8_t fondo,PantallaLCD * pPantalla);
// Dibuja una imagen situando su esquina superior izquierda en 'xImagen', 'yImagen', que tiene
// una resolución de 'ancho' puntos en horizontal y 'alto' puntos en vertical. Los bytes que
// expresan el color de todos sus puntos (cada punto en 4 bytes en formato ARGB)
// se encuentran en 'imagen'.


#endif /* INC_PANTALLA_H_ */
