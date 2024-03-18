#include "variasgraficas.h"

VariasGraficas::VariasGraficas(QWidget *parent) : QWidget(parent) {

        // Constructor al que se le pasa la dirección del objeto que representa al widget que contiene a
        // este componente
        grafica = new QChart();  // Crea el objeto que maneja las gráficas
        grafica->legend()->hide();  // No muestra el nombre
        grafica->setTitle("");  // No muestra el nombre de la gráfica
        grafica->setMargins(QMargins(0, 0, 0, 0));  // Elimina márgenes
        ejeX = new QValueAxis();
        grafica->addAxis(ejeX, Qt::AlignBottom);
        // Eje X debajo
        ejeYIzquierdo = new QValueAxis();
        grafica->addAxis(ejeYIzquierdo, Qt::AlignLeft);
        // Eje Y a la izquierda
        ejeYDerecho = new QValueAxis();
        grafica->addAxis(ejeYDerecho, Qt::AlignRight);
        // Eje Y a la derecha
        visor = new QChartView(grafica);  // Añade la gráfica al visor
        visor->setRenderHint(QPainter::Antialiasing);  // Establece suavizado de la gráfica
        visor->setParent(parent);  // Sitúa el visor dentro del widget contenedor
        nGraficasIzquierda = 0;  // Por ahora no se añadieron gráficas
        nGraficasDerecha = 0;
    }
    void VariasGraficas::setNGraficas(int nGraficasIzquierda, int nGraficasDerecha) {
        // Para establecer cuántas gráficas están asociadas al eje izquierdo y cuántas al derecho
        puntosIzquierda = new QLineSeries* [nGraficasIzquierda];
        this->nGraficasIzquierda = nGraficasIzquierda;
        for (int i = 0; i < nGraficasIzquierda; i++) {
            puntosIzquierda[i] = new QLineSeries();
            grafica->addSeries(puntosIzquierda[i]);
            puntosIzquierda[i]->attachAxis(ejeX);
            puntosIzquierda[i]->attachAxis(ejeYIzquierdo);
        }
        puntosDerecha = new QLineSeries* [nGraficasDerecha];
        this->nGraficasDerecha = nGraficasDerecha;
        for (int i = 0; i < nGraficasDerecha; i++) {
            puntosDerecha[i] = new QLineSeries();
            grafica->addSeries(puntosDerecha[i]);
            puntosDerecha[i]->attachAxis(ejeX);
            puntosDerecha[i]->attachAxis(ejeYDerecho);
        }
    }
    void VariasGraficas::setColorGraficaIzquierda(int nGrafica, QColor * pColor) {
        // Establece el color de cada gráfica. Se numeran a partir de 0.
        puntosIzquierda[nGrafica]->setColor(*pColor);
    }
    void VariasGraficas::setColorGraficaDerecha(int nGrafica, QColor * pColor) {
        // Establece el color de cada gráfica. Se numeran a partir de 0.
        puntosDerecha[nGrafica]->setColor(*pColor);
    }
    void VariasGraficas::setColorEjeIzquierdo(QColor * pColor) {
        // Establece color de eje
        ejeYIzquierdo->setLinePenColor(*pColor);
    }
    void VariasGraficas::setColorEjeDerecho(QColor * pColor) {
        ejeYDerecho->setLinePenColor(*pColor);
    }
    void VariasGraficas::anadePuntos(double x, double * valoresIzquierda, double * valoresDerecha) {
        // Añade un nuevos puntos, todos con la misma coordenada X. En valoresIzquierda se pasa la
        // dirección de memoria donde están todos los valores para las gráficas asociadas al eje izquierdo.
        // En valoresDerecha para las del eje derecho.
        for (int i = 0; i < nGraficasIzquierda; i ++)
            puntosIzquierda[i]->append(x, valoresIzquierda[i]);
        for (int i = 0; i < nGraficasDerecha; i ++)
            puntosDerecha[i]->append(x, valoresDerecha[i]);
    }
    void VariasGraficas::setRangoX(double xMin, double xMax) {
        // Establece el rango de valores en el eje horizontal
        ejeX->setRange(xMin, xMax);
    }
    void VariasGraficas::setRangoYIzquierda(double yMin, double yMax) {
        // Establece el rango de valores en el eje vertical de la izquierda
        ejeYIzquierdo->setRange(yMin, yMax);
    }
    void VariasGraficas::setRangoYDerecha(double yMin, double yMax) {
        // Establece el rango de valores en el eje vertical de la derecha
        ejeYDerecho->setRange(yMin, yMax);
    }
    void VariasGraficas::setGeometry(const QRect & rectangulo) {
        // Redefine este método recibido mediante herencia de la clase base QWidget.
        // Establece la posición de la gráfica y sus dimensiones para que corresponda
        // con el rectángulo representado por el objeto pasado por referencia.
        visor->setGeometry(rectangulo);
    }
    void VariasGraficas::remove()
    {
        for (int i = 0; i < nGraficasIzquierda; i ++)
            puntosIzquierda[i]->remove(0);
        for (int i = 0; i < nGraficasDerecha; i ++)
            puntosDerecha[i]->remove(0);
    }
    double VariasGraficas::getTime(int x)
    {
        return puntosIzquierda[0]->at(x).x();
    }

