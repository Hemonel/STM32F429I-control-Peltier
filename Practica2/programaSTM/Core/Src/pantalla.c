
#include "pantalla.h"
#include <stdio.h>  // Para sprintf()
#include <stdlib.h>  // para abs()
#include <cmsis_os2.h>  // para osDelay()


uint16_t pantallaPulsada(uint16_t * x, uint16_t *y)
{
	TS_StateTypeDef TsState;
	BSP_TS_GetState(& TsState);
	*x=TsState.X;
	*y=320-TsState.Y;
	return TsState.TouchDetected;

}
void setvisiblebBoton(uint8_t Vis, TsBoton * botonReset)
{
	botonReset->visible=Vis;
	if (botonReset->visible)
	{
		if(botonReset->habilitado)
		{
			intercambiaBuffersLCD(botonReset->pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,100,0,botonReset->pantallaLCD);
			intercambiaBuffersLCD(botonReset->pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,100,0,botonReset->pantallaLCD);
		}

		else
		{
		intercambiaBuffersLCD(botonReset->pantallaLCD);
		dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,1,100,0,botonReset->pantallaLCD);
		intercambiaBuffersLCD(botonReset->pantallaLCD);
		dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,1,100,0,botonReset->pantallaLCD);

		}

	}
	else
	{
		intercambiaBuffersLCD(botonReset->pantallaLCD);
		dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,0,0,botonReset->pantallaLCD);
		intercambiaBuffersLCD(botonReset->pantallaLCD);
		dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,0,0,botonReset->pantallaLCD);
	}

}

void setHabilitacionBoton(uint8_t En, TsBoton * botonReset)
{
	botonReset->habilitado=En;
	if (botonReset->visible)
		{
			if(botonReset->habilitado)
			{
				intercambiaBuffersLCD(botonReset->pantallaLCD);
				dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,100,0,botonReset->pantallaLCD);
				intercambiaBuffersLCD(botonReset->pantallaLCD);
				dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,100,0,botonReset->pantallaLCD);
			}

			else
			{
			intercambiaBuffersLCD(botonReset->pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,1,100,0,botonReset->pantallaLCD);
			intercambiaBuffersLCD(botonReset->pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,1,100,0,botonReset->pantallaLCD);

			}

		}
		else
		{
			intercambiaBuffersLCD(botonReset->pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,0,0,botonReset->pantallaLCD);
			intercambiaBuffersLCD(botonReset->pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,0,0,botonReset->pantallaLCD);
		}
}
void atiendeBoton(uint16_t x,uint16_t y, uint16_t pulsada, TsBoton * botonReset)
{
	if (pulsada && botonReset->habilitado  && botonReset->visible)
		if (x>botonReset->x && x<botonReset->x+botonReset->ancho && y>botonReset->y && y<botonReset->y+botonReset->alto )
			botonReset->pFuncion();
}

void inicializaBoton(uint16_t xBoton,uint16_t yBoton, uint8_t ancho, uint8_t alto, const uint8_t * imagen ,void (* pFunc)(void), uint8_t vis,uint8_t En,TsBoton * botonReset, PantallaLCD *pantallaLCD)
{
	botonReset->x=xBoton;
	botonReset->y=yBoton;
	botonReset->ancho=ancho;
	botonReset->alto=alto;
	botonReset->habilitado=En;
	botonReset->visible=vis;
	botonReset->pFuncion=pFunc;
	botonReset->imagen=imagen;
	botonReset->pantallaLCD=pantallaLCD;
	if (botonReset->visible)
	{
		if(botonReset->habilitado)
		{   intercambiaBuffersLCD(pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,100,0,pantallaLCD);
			intercambiaBuffersLCD(pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,0,100,0,pantallaLCD);
		}
		else
		{   intercambiaBuffersLCD(pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,1,100,0,pantallaLCD);
			intercambiaBuffersLCD(pantallaLCD);
			dibujaImagen(botonReset->x, botonReset->y, 	botonReset->ancho,botonReset->alto,botonReset->imagen,1,100,0,pantallaLCD);
		}
	}
}

void inicializaPantalla2Buffers(uint8_t solido,uint32_t colorFondo,const uint8_t * imagen, PantallaLCD* pantallaLCD) {
	// Inicializa la pantalla para trabajar con 2 frame buffers. Pone el color de todos los puntos
	// segÃºn el parÃ¡metro 'colorFondo'. En 'pantallaLCD' se indica la direcciÃ³nd e una estructura
	// donde se guarda informaciÃ³n sobre la pantalla.

	BSP_LCD_Init();
	BSP_LCD_Init();  // Inicializa la pantalla LCD

	pantallaLCD->ancho = BSP_LCD_GetXSize();  // Obtiene el ancho de la pantalla en nÃºmero de puntos
	pantallaLCD->alto = BSP_LCD_GetYSize();  // Obtiene el alto de la pantalla
	pantallaLCD->buffers[0] = (uint32_t*)LCD_FRAME_BUFFER_LAYER_0;  // DirecciÃ³n del frame buffer nÃºmero 0
	pantallaLCD->buffers[1] = (uint32_t*)LCD_FRAME_BUFFER_LAYER_1;  // DirecciÃ³n del frame buffer nÃºmero 1

    BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER_0);  // Inicializa frame buffer 0
	BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_LAYER_1);  // Inicializa frame buffer 1
	pantallaLCD->solido=solido;
	pantallaLCD->fondoColor=colorFondo;
	pantallaLCD->fondoImagen=imagen;
	pantallaLCD->bufferDibujo = 0;  // Selecciona el frame buffer 0 para dibujar
	dibujaRectangulo(0, 0, 240, 320, colorFondo,pantallaLCD);  // Pone todos los puntos al color de fondo
	pantallaLCD->bufferDibujo = 1;  // Selecciona el frame buffer 0 para dibujar
	dibujaRectangulo(0, 0, 240, 320, colorFondo,pantallaLCD);  // Pone todos los puntos al color de fondo
	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));  // Espera a que la GPU finalice sincronismo con pantalla
	BSP_LCD_SetLayerVisible(0, DISABLE);  // Frame buffer 0 visible
	BSP_LCD_SetLayerVisible(1,ENABLE);  // Frame buffer 1 invisible
	//BSP_LCD_SetTransparency(1,0x10);
	BSP_TS_Init(pantallaLCD->ancho, pantallaLCD->alto);
	if (!solido)
			dibujaImagen(0,0,240,320,imagen,0,100,0,pantallaLCD);
	intercambiaBuffersLCD(pantallaLCD);
	if (!solido)
				dibujaImagen(0,0,240,320,imagen,0,100,0,pantallaLCD);


}


void intercambiaBuffersLCD(PantallaLCD* pantallaLCD) {
	// Cambia en quÃ© frame buffer se dibuja y quÃ© frame buffer se visualiza

	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));  // Espera a que la GPU finalice sincronismo con pantalla
	if (pantallaLCD->bufferDibujo == 0) {  // Si se dibujÃ³ en el buffer 0
		BSP_LCD_SetLayerVisible(1, DISABLE);  // Buffer 1 invisible
		BSP_LCD_SetLayerVisible(0, ENABLE);  // Buffer 0 visible
		pantallaLCD->bufferDibujo = 1;  // Buffer donde se va a dibujar
	} else {
		BSP_LCD_SetLayerVisible(0, DISABLE);  // Buffer 0 invisible
		BSP_LCD_SetLayerVisible(1, ENABLE);  // Buffer 1 ivisible
		pantallaLCD->bufferDibujo = 0;  // Buffer donde se dibuja
	}
}

uint32_t rgb2gray(uint32_t color)
{
	uint8_t r,g,b,gy,alfa=color>>24;
	uint32_t salida=0;
	b=color;
	g=color>>8;
	r=color>>16;
	gy=(r+g+b)/3;
	salida|=gy;
	salida|=gy<<8;
	salida|=gy<<16;
	salida|=alfa<<24;
	return salida;
}
void dibujaRectangulo(uint16_t xInicial, uint16_t yInicial, uint16_t ancho, uint16_t alto,
	uint32_t color, PantallaLCD * pPantalla) {
	// Dibuja un rectÃ¡ngulo cuya esquina superior izquierda estÃ¡ situada en (xInicial, yInicial) y
	// donde se indica el alto y ancho el color de sus puntos

    uint32_t * p = pPantalla->buffers[pPantalla->bufferDibujo] + yInicial * pPantalla->ancho + xInicial;
    // Puntero que apunta al primer punto de la primera fila de puntos del rectÃ¡ngulo

    uint32_t incremento = pPantalla->ancho - ancho;
    // NÃºmero de puntos a saltar en el frame buffer para ir desde el final de una fila al comienzo de la siguiente

    for(int fila = 0; fila < alto; fila++) {  // Recorriendo las filas del rectÃ¡ngulo
    	for(int columna = 0; columna < ancho; columna++) {  // y las columnas
    		//dibujaPunto(fila,columna, color,0,0xFF, fondo, pPantalla);
    		*(p++) = color;  // colorea cada punto del rectÃ¡ngulo e incrementa el puntero para ir al siguiente
    	}
    	p += incremento;  // Mueve el puntero al comienzo de la siguiente fila
    }
}

void dibujaRectangulo2(uint16_t xInicial, uint16_t yInicial, uint16_t ancho, uint16_t alto,
	uint32_t color,uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla) {
	// Dibuja un rectÃ¡ngulo cuya esquina superior izquierda estÃ¡ situada en (xInicial, yInicial) y
	// donde se indica el alto y ancho el color de sus puntos

    uint32_t * p = pPantalla->buffers[pPantalla->bufferDibujo] + yInicial * pPantalla->ancho + xInicial;
    // Puntero que apunta al primer punto de la primera fila de puntos del rectÃ¡ngulo

    uint32_t incremento = pPantalla->ancho - ancho;
    // NÃºmero de puntos a saltar en el frame buffer para ir desde el final de una fila al comienzo de la siguiente

    for(int fila = 0; fila < alto; fila++) {  // Recorriendo las filas del rectÃ¡ngulo
    	for(int columna = 0; columna < ancho; columna++) {  // y las columnas
    		//dibujaPunto(fila,columna, color,0,0xFF, fondo, pPantalla);
    		dibujaPunto(columna, fila, color,gray,alfa, fondo, pPantalla);
    	}
    	p += incremento;  // Mueve el puntero al comienzo de la siguiente fila
    }
}

void inicializaJuegoCaracteres(uint8_t ancho, uint8_t alto, uint8_t separacion, uint32_t color,
	const uint8_t* pCaracteres, JuegoCaracteres * pJuegoCaracteres) {
	// Inicializa la estructura apuntada por 'pJuegoCaracteres' para indicar que se estÃ¡n utilizando
	// caracteres de 'ancho' puntos de ancho, 'alto' puntos de alto, con una separaciÃ³n horizontal
	// entre ellos de 'separacion' puntos y que se dibujan con un color. En 'pCaracteres' se indica
	// dÃ³nde se guardan los bytes que describen cÃ³mo hay que dibujar los caracteres.

	pJuegoCaracteres->ancho = ancho;
	pJuegoCaracteres->alto = alto;
	pJuegoCaracteres->separacion = separacion;
	pJuegoCaracteres->pCaracteres = pCaracteres;
	pJuegoCaracteres->color = color;
	// Guarda los parÃ¡metros en campos de la estructura apuntada por 'pJuegoCaracteres'

	pJuegoCaracteres->nBytesPorCaracter = ((ancho - 1) / 8 + 1) * alto;
	// Guarda el nÃºmero de bytes utilizado para describir cÃ³mo se dibuja cada carÃ¡cter.
}

void inicializaJuegoCaracteresAlpha(uint8_t ancho, uint8_t alto, uint8_t separacion,const uint8_t* pCaracteres, JuegoCaracteresAlpha * pJuegoCaracteres)
{
		pJuegoCaracteres->ancho = ancho;
		pJuegoCaracteres->alto = alto;
		pJuegoCaracteres->separacion = separacion;
		pJuegoCaracteres->pCaracteres = pCaracteres;
		pJuegoCaracteres->nBytesPorCaracter = ancho * alto*4;
		return;
}



void dibujaCaracterAlpha(uint16_t xCaracter, uint16_t yCaracter, char caracter,
		JuegoCaracteresAlpha * pJuegoCaracteresAlpha, uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla)
		{
		const uint8_t * pBytes = pJuegoCaracteresAlpha->pCaracteres +(caracter - ' ') * pJuegoCaracteresAlpha->nBytesPorCaracter;

		dibujaImagen(xCaracter, yCaracter, pJuegoCaracteresAlpha->ancho, pJuegoCaracteresAlpha->alto,pBytes, gray,alfa,fondo,pPantalla);


}
void dibujaCaracter(uint16_t xCaracter, uint16_t yCaracter, char caracter,
	JuegoCaracteres * pJuegoCaracteres, uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla) {
	// Dibuja el carÃ¡cter indicado en 'caracter' de forma que su esquina superior izquierda
	// corresponda a las coordenadas ('xCaracter', 'yCaracter') utilizando el juego de caracteres
	// descrito en la estructura apuntada por 'pJuegoCaracteres'.
	const uint8_t * pBytes = pJuegoCaracteres->pCaracteres +
		(caracter - ' ') * pJuegoCaracteres->nBytesPorCaracter;
	// Obtiene la direcciÃ³n de memoria donde se encuentra el primer byte donde se describe cÃ³mo hay
	// que dibujar el carÃ¡cter

	uint8_t nBytesAnchoCaracter = (pJuegoCaracteres->ancho - 1) / 8 + 1;
	// NÃºmero de bytes necesarios para representar los puntos de una fila
	uint32_t color;
	//uint32_t * pBuffer;

	// DirecciÃ³n de memoria en el frame buffer donde se representan los puntos de pantalla para la
	// visualizaciÃ³n del carÃ¡cter
	color=pJuegoCaracteres->color;
	for(int y = 0; y < pJuegoCaracteres->alto; y++) {  // Recorriendo las filas
	    for(int x = 0; x < pJuegoCaracteres->ancho; x++) {  // y las columnas de puntos
	    	{
	    		if (pBytes[x / 8 + y * nBytesAnchoCaracter] & (1 << (7 - x % 8)))
	    				dibujaPunto(x+xCaracter, y+yCaracter, color,gray,alfa,fondo, pPantalla);
	    		else
	    			    dibujaPunto(x+xCaracter, y+yCaracter, color,gray,0,fondo, pPantalla);
	    	}
	    }
	}
}
void dibujaCadenaCaracteres(uint32_t xCadena, uint32_t yCadena, char* cadena,
	JuegoCaracteres * pJuegoCaracteres,uint8_t gray,uint8_t alfa, uint8_t fondo,  PantallaLCD * pPantalla) {
	// Dibuja la cadena de caracteres apuntada por 'cadena' de forma que la esquina superior izquierda
	// del primer carÃ¡cter se indica en 'xCadena' e 'yCadena'. Los caracteres se visualizan segÃºn
	// el juego de caracteres expresado en la estructura apuntada por 'pJuegoCaracteres'.

	int i = 0, x = 0;
	while(cadena[i]) {  // Para cada carÃ¡cter de la cadena

		dibujaCaracter(xCadena + x * (pJuegoCaracteres->separacion + pJuegoCaracteres->ancho), yCadena,
			cadena[i], pJuegoCaracteres,gray,alfa,fondo, pPantalla);
		// Dibuja el carÃ¡cter en posiciones sucesivas hacia la derecha

		if (cadena[i] != '\'' && cadena[i] != '~')
			x++;
		// La comilla se utiliza para componer letras acentuadas, ej. "presi\'on"
		// El ~ se utiliza para componer las eÃ±es, ej. "ca~na"

		i++;  // Para acceder al siguiente carÃ¡cter de la cadena
	}
}


void dibujaCadenaCaracteresAlpha(uint32_t xCadena, uint32_t yCadena, char *cadena,
		JuegoCaracteresAlpha * pJuegoCaracteresAlpha, uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla)
{
	int i = 0, x = 0;
		while(cadena[i]) {  // Para cada carÃ¡cter de la cadena

			dibujaCaracterAlpha(xCadena + x * (pJuegoCaracteresAlpha->separacion + pJuegoCaracteresAlpha->ancho), yCadena,
				cadena[i], pJuegoCaracteresAlpha,gray,alfa,fondo, pPantalla);
			// Dibuja el carÃ¡cter en posiciones sucesivas hacia la derecha

			if (cadena[i] != '\'' && cadena[i] != '~')
				x++;
			// La comilla se utiliza para componer letras acentuadas, ej. "presi\'on"
			// El ~ se utiliza para componer las eÃ±es, ej. "ca~na"

			i++;  // Para acceder al siguiente carÃ¡cter de la cadena
		}
}

uint32_t mezclaColores(uint32_t back,uint32_t front)
{
	uint8_t rb,gb,bb,rf,gf,bf,r,g,b,transparencia;
	uint32_t salida;
	transparencia=front>>24;
	rb=back>>16;
	gb=back>>8;
	bb=back;
	rf=front>>16;
	gf=front>>8;
	bf=front;
	r=((rf-rb)*(transparencia*100/255)/100)+rb;
	g=((gf-gb)*(transparencia*100/255)/100)+gb;
	b=((bf-bb)*(transparencia*100/255)/100)+bb;
	salida=0x000000FF&b;
	salida|=g<<8;
	salida|=r<<16;
	salida|=0xFF000000;
	return salida;
}

void dibujaPunto(uint16_t x, uint16_t y, uint32_t color,uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla)
{
	// Dibuja un punto en coordenadas ('x', 'y') con el color indicado en 'color'
		uint32_t * pPuntos;
		uint32_t coloralfa,back,mezcla;
		coloralfa=color>>24;
		coloralfa=(uint8_t)coloralfa*((double)alfa/100);
		coloralfa=(coloralfa<<24)|(color&0x00FFFFFF);
		/*coloralfa=0x00FFFFFF & color;
		coloralfa|=alfa<<24;*/
		if (gray)
			    	coloralfa=rgb2gray(coloralfa);
		pPuntos= pPantalla->buffers[pPantalla->bufferDibujo]+( y * pPantalla->ancho) + x;

	    if (fondo)
	    {	if(pPantalla->solido)
	    		back=pPantalla->fondoColor;
	    	else
	    		back=*(pPantalla->fondoImagen +( y * pPantalla->ancho) + x);
	    }
	    else
	    	back=*pPuntos;
	    mezcla=mezclaColores(back,coloralfa);
	    *pPuntos= mezcla;

	    // Guarda los 4 bytes del color en el frame buffer

}


void dibujaLinea(int x0, int y0, int x1, int y1, uint32_t color,  uint8_t gray,uint8_t alfa,uint8_t fondo,PantallaLCD *pPantalla) {
	// Dibuja una lÃ­nea que une el punto indicado en 'x0', 'y0' con el punto indicado en 'x1', 'y1'
    // en el color 'color' utilizando el algoritmo de Bressenham
	// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

	int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
	int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
	int err = (dx>dy ? dx : -dy)/2, e2;

	while(1) {
	    dibujaPunto(x0, y0, color,gray,alfa,fondo, pPantalla);
	    if (x0==x1 && y0==y1) break;
	    e2 = err;
	    if (e2 >-dx) { err -= dy; x0 += sx; }
	    if (e2 < dy) { err += dx; y0 += sy; }
	}
}


void dibujaPuntosCirculo(int xc, int yc, int x, int y, uint32_t color, uint8_t gray,uint8_t alfa, uint8_t fondo, PantallaLCD * pPantalla) {
	// Utilizada por la funciÃ³n dibujaCirculo

    dibujaPunto(xc+x, yc+y, color,gray,alfa,fondo, pPantalla);
    dibujaPunto(xc-x, yc+y, color,gray,alfa,fondo, pPantalla);
    dibujaPunto(xc+x, yc-y, color,gray,alfa,fondo, pPantalla);
    dibujaPunto(xc-x, yc-y, color,gray,alfa,fondo, pPantalla);
    dibujaPunto(xc+y, yc+x, color,gray,alfa,fondo, pPantalla);
    dibujaPunto(xc-y, yc+x, color,gray,alfa,fondo, pPantalla);
    dibujaPunto(xc+y, yc-x, color,gray,alfa,fondo, pPantalla);
    dibujaPunto(xc-y, yc-x, color, gray,alfa,fondo,pPantalla);
}


void dibujaCirculo(int xc, int yc, int r, uint32_t color, uint8_t gray,uint8_t alfa,uint8_t fondo, PantallaLCD * pPantalla) {
	// Dibuja un cÃ­rculo cuyo centro se indica en 'xc', 'yc', de radio 'r' y con el color
	// indicado en 'color' utilizando el algoritmo de Bressenham
	// https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/
    int x = 0, y = r;
    int d = 3 - 2 * r;
    dibujaPuntosCirculo(xc, yc, x, y, color,gray,alfa,fondo,pPantalla);
    while (y >= x) {
        x++;
        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else d = d + 4 * x + 6;
        dibujaPuntosCirculo(xc, yc, x, y, color,gray,alfa,fondo, pPantalla);
    }
}

void dibujaImagen(uint16_t xImagen, uint16_t yImagen, uint16_t ancho, uint16_t alto,
	const uint8_t * imagen, uint8_t gray,uint8_t alfa,uint8_t fondo,PantallaLCD * pPantalla) {
	// Dibuja una imagen situando su esquina superior izquierda en 'xImagen', 'yImagen', que tiene
	// una resoluciÃ³n de 'ancho' puntos en horizontal y 'alto' puntos en vertical. Los bytes que
	// expresan el color de todos sus puntos (cada punto en 4 bytes en formato ARGB)
	// se encuentran en 'imagen'.

	uint32_t * p = (uint32_t*) imagen;  // Para acceder a cada punto por separado
	for(uint16_t y = yImagen; y < yImagen + alto; y++)  // Recorriendo filas
		for (uint16_t x = xImagen; x < xImagen + ancho; x++) {  // y columnas
			dibujaPunto(x, y, *p, gray,alfa,fondo,pPantalla);  // dibuja cada punto
			p++;  // Para acceder al color del siguiente punto
		}
}


