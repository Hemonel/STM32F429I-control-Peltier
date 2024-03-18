
#include "pantalla.h"
#include <stdio.h>  // Para sprintf()
#include <stdlib.h>  // para abs()
#include <cmsis_os2.h>  // para osDelay()


void inicializaPantalla2Buffers(uint32_t colorFondo,const uint8_t * imagen, PantallaLCD* pantallaLCD) {
	// Inicializa la pantalla para trabajar con 2 frame buffers. Pone el color de todos los puntos
	// según el parámetro 'colorFondo'. En 'pantallaLCD' se indica la direcciónd e una estructura
	// donde se guarda información sobre la pantalla.

	BSP_LCD_Init();
	BSP_LCD_Init();  // Inicializa la pantalla LCD

	pantallaLCD->ancho = BSP_LCD_GetXSize();  // Obtiene el ancho de la pantalla en número de puntos
	pantallaLCD->alto = BSP_LCD_GetYSize();  // Obtiene el alto de la pantalla
	pantallaLCD->buffers[0] = (uint32_t*)LCD_FRAME_BUFFER_LAYER_0;  // Dirección del frame buffer número 0
	pantallaLCD->buffers[1] = (uint32_t*)LCD_FRAME_BUFFER_LAYER_1;  // Dirección del frame buffer número 1

    BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER_0);  // Inicializa frame buffer 0
	BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_LAYER_1);  // Inicializa frame buffer 1

	pantallaLCD->bufferDibujo = 0;  // Selecciona el frame buffer 0 para dibujar
	dibujaImagen(0, 0, pantallaLCD->ancho,pantallaLCD->alto,0,imagen,pantallaLCD);
	pantallaLCD->bufferDibujo = 1;  // Selecciona el frame buffer 0 para dibujar
	dibujaImagen(0, 0, pantallaLCD->ancho,pantallaLCD->alto,0,imagen,pantallaLCD);
	//dibujaRectangulo(0, 0, 240, 320, colorFondo, pantallaLCD);  // Pone todos los puntos al color de fondo

	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));  // Espera a que la GPU finalice sincronismo con pantalla
	BSP_LCD_SetLayerVisible(0, ENABLE);  // Frame buffer 0 visible
	BSP_LCD_SetLayerVisible(1, DISABLE);  // Frame buffer 1 invisible

	BSP_TS_Init(pantallaLCD->ancho, pantallaLCD->alto);
	// Indica a la interfaz táctil el ancho y alto de la pantalla en puntos
}


void intercambiaBuffersLCD(PantallaLCD* pantallaLCD) {
	// Cambia en qué frame buffer se dibuja y qué frame buffer se visualiza

	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));  // Espera a que la GPU finalice sincronismo con pantalla
	if (pantallaLCD->bufferDibujo == 0) {  // Si se dibujó en el buffer 0
		BSP_LCD_SetLayerVisible(1, DISABLE);  // Buffer 1 invisible
		BSP_LCD_SetLayerVisible(0, ENABLE);  // Buffer 0 visible
		pantallaLCD->bufferDibujo = 1;  // Buffer donde se va a dibujar
	} else {
		BSP_LCD_SetLayerVisible(0, DISABLE);  // Buffer 0 invisible
		BSP_LCD_SetLayerVisible(1, ENABLE);  // Buffer 1 ivisible
		pantallaLCD->bufferDibujo = 0;  // Buffer donde se dibuja
	}
}


void dibujaRectangulo(uint16_t xInicial, uint16_t yInicial, uint16_t ancho, uint16_t alto,
	uint32_t color, PantallaLCD * pPantalla) {
	// Dibuja un rectángulo cuya esquina superior izquierda está situada en (xInicial, yInicial) y
	// donde se indica el alto y ancho el color de sus puntos

    uint32_t * p = pPantalla->buffers[pPantalla->bufferDibujo] + yInicial * pPantalla->ancho + xInicial;
    // Puntero que apunta al primer punto de la primera fila de puntos del rectángulo

    uint32_t incremento = pPantalla->ancho - ancho;
    // Número de puntos a saltar en el frame buffer para ir desde el final de una fila al comienzo de la siguiente

    for(int fila = 0; fila < alto; fila++) {  // Recorriendo las filas del rectángulo
    	for(int columna = 0; columna < ancho; columna++) {  // y las columnas
    	    *(p++) = color;  // colorea cada punto del rectángulo e incrementa el puntero para ir al siguiente
    	}
    	p += incremento;  // Mueve el puntero al comienzo de la siguiente fila
    }
}


void inicializaJuegoCaracteres(uint8_t ancho, uint8_t alto, uint8_t separacion, uint32_t color,
	const uint8_t* pCaracteres, JuegoCaracteres * pJuegoCaracteres) {
	// Inicializa la estructura apuntada por 'pJuegoCaracteres' para indicar que se están utilizando
	// caracteres de 'ancho' puntos de ancho, 'alto' puntos de alto, con una separación horizontal
	// entre ellos de 'separacion' puntos y que se dibujan con un color. En 'pCaracteres' se indica
	// dónde se guardan los bytes que describen cómo hay que dibujar los caracteres.

	pJuegoCaracteres->ancho = ancho;
	pJuegoCaracteres->alto = alto;
	pJuegoCaracteres->separacion = separacion;
	pJuegoCaracteres->pCaracteres = pCaracteres;
	pJuegoCaracteres->color = color;
	// Guarda los parámetros en campos de la estructura apuntada por 'pJuegoCaracteres'

	pJuegoCaracteres->nBytesPorCaracter = ((ancho - 1) / 8 + 1) * alto;
	// Guarda el número de bytes utilizado para describir cómo se dibuja cada carácter.
}


void dibujaCaracter(uint16_t xCaracter, uint16_t yCaracter, char caracter,
	JuegoCaracteres * pJuegoCaracteres, PantallaLCD * pPantalla) {
	// Dibuja el carácter indicado en 'caracter' de forma que su esquina superior izquierda
	// corresponda a las coordenadas ('xCaracter', 'yCaracter') utilizando el juego de caracteres
	// descrito en la estructura apuntada por 'pJuegoCaracteres'.

	const uint8_t * pBytes = pJuegoCaracteres->pCaracteres +
		(caracter - ' ') * pJuegoCaracteres->nBytesPorCaracter;
	// Obtiene la dirección de memoria donde se encuentra el primer byte donde se describe cómo hay
	// que dibujar el carácter

	uint8_t nBytesAnchoCaracter = (pJuegoCaracteres->ancho - 1) / 8 + 1;
	// Número de bytes necesarios para representar los puntos de una fila

	uint32_t * pBuffer = pPantalla->buffers[pPantalla->bufferDibujo] + yCaracter * pPantalla->ancho +
		xCaracter;
	// Dirección de memoria en el frame buffer donde se representan los puntos de pantalla para la
	// visualización del carácter

	for(int y = 0; y < pJuegoCaracteres->alto; y++) {  // Recorriendo las filas
	    for(int x = 0; x < pJuegoCaracteres->ancho; x++) {  // y las columnas de puntos

		    if (pBytes[x / 8 + y * nBytesAnchoCaracter] & (1 << (7 - x % 8))) {
		    	// Si hay que dibujar un punto del carácter

		        pBuffer[y * pPantalla->ancho + x] = pJuegoCaracteres->color;
		        // Pone el punto al color establecido en el juego de caracteres
		    }
	    }
	}
}


void dibujaCadenaCaracteres(uint32_t xCadena, uint32_t yCadena, char* cadena,
	JuegoCaracteres * pJuegoCaracteres, PantallaLCD * pPantalla) {
	// Dibuja la cadena de caracteres apuntada por 'cadena' de forma que la esquina superior izquierda
	// del primer carácter se indica en 'xCadena' e 'yCadena'. Los caracteres se visualizan según
	// el juego de caracteres expresado en la estructura apuntada por 'pJuegoCaracteres'.

	int i = 0, x = 0;
	while(cadena[i]) {  // Para cada carácter de la cadena

		dibujaCaracter(xCadena + x * (pJuegoCaracteres->separacion + pJuegoCaracteres->ancho), yCadena,
			cadena[i], pJuegoCaracteres, pPantalla);
		// Dibuja el carácter en posiciones sucesivas hacia la derecha

		if (cadena[i] != '\'' && cadena[i] != '~')
			x++;
		// La comilla se utiliza para componer letras acentuadas, ej. "presi\'on"
		// El ~ se utiliza para componer las eñes, ej. "ca~na"

		i++;  // Para acceder al siguiente carácter de la cadena
	}
}

/*void inicializaJuegoCaracteresAlpha(uint8_t ancho, uint8_t alto, uint8_t separacion,
	const uint8_t* pCaracteres, JuegoCaracteres * pJuegoCaracteres) {
	// Inicializa la estructura apuntada por 'pJuegoCaracteres' para indicar que se están utilizando
	// caracteres de 'ancho' puntos de ancho, 'alto' puntos de alto, con una separación horizontal
	// entre ellos de 'separacion' puntos y que se dibujan con un color. En 'pCaracteres' se indica
	// dónde se guardan los bytes que describen cómo hay que dibujar los caracteres.

	pJuegoCaracteres->ancho = ancho;
	pJuegoCaracteres->alto = alto;
	pJuegoCaracteres->separacion = separacion;
	pJuegoCaracteres->pCaracteres = pCaracteres;
	// Guarda los parámetros en campos de la estructura apuntada por 'pJuegoCaracteres'

	pJuegoCaracteres->nBytesPorCaracter = ((ancho - 1) / 8 + 1) * alto;
	// Guarda el número de bytes utilizado para describir cómo se dibuja cada carácter.
}
void dibujaCadenaCaracteresAlpha(uint32_t xCadena, uint32_t yCadena, char* cadena,
	JuegoCaracteres * pJuegoCaracteres, PantallaLCD * pPantalla) {
	// Dibuja la cadena de caracteres apuntada por 'cadena' de forma que la esquina superior izquierda
	// del primer carácter se indica en 'xCadena' e 'yCadena'. Los caracteres se visualizan según
	// el juego de caracteres expresado en la estructura apuntada por 'pJuegoCaracteres'.

	int i = 0, x = 0;
	while(cadena[i]) {  // Para cada carácter de la cadena

		dibujaCaracterAlfa(xCadena + x * (pJuegoCaracteres->separacion + pJuegoCaracteres->ancho), yCadena,
			cadena[i], pJuegoCaracteres, pPantalla);
		// Dibuja el carácter en posiciones sucesivas hacia la derecha

		if (cadena[i] != '\'' && cadena[i] != '~')
			x++;
		// La comilla se utiliza para componer letras acentuadas, ej. "presi\'on"
		// El ~ se utiliza para componer las eñes, ej. "ca~na"

		i++;  // Para acceder al siguiente carácter de la cadena
	}
}

void dibujaCaracterAlfa(uint16_t xCaracter, uint16_t yCaracter, char caracter,
	JuegoCaracteres * pJuegoCaracteres, PantallaLCD * pPantalla) {
	// Dibuja el carácter indicado en 'caracter' de forma que su esquina superior izquierda
	// corresponda a las coordenadas ('xCaracter', 'yCaracter') utilizando el juego de caracteres
	// descrito en la estructura apuntada por 'pJuegoCaracteres'.

	const uint8_t pBytes = pJuegoCaracteres->pCaracteres +
		(caracter - ' ') * pJuegoCaracteres->nBytesPorCaracter;
	// Obtiene la dirección de memoria donde se encuentra el primer byte donde se describe cómo hay
	// que dibujar el carácter
	int lista[404];
	for (int i=pBytes-1;i<pBytes+404;i++) lista[i-Bytes]=pJuegoCaracteres[pBytes];
	dibujaImagen(xCaracter, yCaracter,pJuegoCaracteres->ancho, pJuegoCaracteres->alto,0,
			lista,pPantalla);

}*/

void dibujaPunto(uint16_t x, uint16_t y, uint32_t color, PantallaLCD * pPantalla) {
	// Dibuja un punto en coordenadas ('x', 'y') con el color indicado en 'color'

	    uint32_t * pPuntos = pPantalla->buffers[pPantalla->bufferDibujo];
	    // Puntero al frame buffer donde se va a dibujar el punto

	    pPuntos[y * pPantalla->ancho + x] = color;
	    // Guarda los 4 bytes del color en el frame buffer
}


void dibujaLinea(int x0, int y0, int x1, int y1, uint32_t color, PantallaLCD *pPantalla) {
	// Dibuja una línea que une el punto indicado en 'x0', 'y0' con el punto indicado en 'x1', 'y1'
    // en el color 'color' utilizando el algoritmo de Bressenham
	// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

	int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
	int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
	int err = (dx>dy ? dx : -dy)/2, e2;

	while(1) {
	    dibujaPunto(x0, y0, color, pPantalla);
	    if (x0==x1 && y0==y1) break;
	    e2 = err;
	    if (e2 >-dx) { err -= dy; x0 += sx; }
	    if (e2 < dy) { err += dx; y0 += sy; }
	}
}


void dibujaPuntosCirculo(int xc, int yc, int x, int y, uint32_t color, PantallaLCD * pPantalla) {
	// Utilizada por la función dibujaCirculo

    dibujaPunto(xc+x, yc+y, color, pPantalla);
    dibujaPunto(xc-x, yc+y, color, pPantalla);
    dibujaPunto(xc+x, yc-y, color, pPantalla);
    dibujaPunto(xc-x, yc-y, color, pPantalla);
    dibujaPunto(xc+y, yc+x, color, pPantalla);
    dibujaPunto(xc-y, yc+x, color, pPantalla);
    dibujaPunto(xc+y, yc-x, color, pPantalla);
    dibujaPunto(xc-y, yc-x, color, pPantalla);
}


void dibujaCirculo(int xc, int yc, int r, uint32_t color, PantallaLCD * pPantalla) {
	// Dibuja un círculo cuyo centro se indica en 'xc', 'yc', de radio 'r' y con el color
	// indicado en 'color' utilizando el algoritmo de Bressenham
	// https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/

    int x = 0, y = r;
    int d = 3 - 2 * r;
    dibujaPuntosCirculo(xc, yc, x, y, color, pPantalla);
    while (y >= x) {
        x++;
        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else d = d + 4 * x + 6;
        dibujaPuntosCirculo(xc, yc, x, y, color, pPantalla);
    }
}


void dibujaImagen(uint16_t xImagen, uint16_t yImagen, uint16_t ancho, uint16_t alto,uint16_t transparencia,
	const uint8_t * imagen, PantallaLCD * pPantalla) {
	// Dibuja una imagen situando su esquina superior izquierda en 'xImagen', 'yImagen', que tiene
	// una resolución de 'ancho' puntos en horizontal y 'alto' puntos en vertical. Los bytes que
	// expresan el color de todos sus puntos (cada punto en 4 bytes en formato ARGB)
	// se encuentran en 'imagen'.

	uint32_t * p = (uint32_t*) imagen;  // Para acceder a cada punto por separado
	uint32_t tr;
	tr=(0xFF-(transparencia/100*0xFF))<<24;
	for(uint16_t y = yImagen; y < yImagen + alto; y++)  // Recorriendo filas
		for (uint16_t x = xImagen; x < xImagen + ancho; x++) {  // y columnas
			if (*p>>24 > tr>>24)dibujaPunto(x, y, *p-tr, pPantalla);  // dibuja cada punto
			else dibujaPunto(x, y, *p, pPantalla);
			p++;  // Para acceder al color del siguiente punto
		}
}



