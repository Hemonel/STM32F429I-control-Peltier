#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <cmath>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    time.start();
    canal.setPortName("COM6");
    // Se utiliza el canal serie identificado en el sistema operativo como "COM3"
    canal.setBaudRate(QSerialPort::Baud115200);
    canal.setParity(QSerialPort::NoParity);
    canal.setStopBits(QSerialPort::OneStop);
    canal.setDataBits(QSerialPort::Data8);
    if (canal.open(QIODevice::ReadWrite)) { // Abre el canal y si no hay problemas ...
        canal.clear(); // Borra los buffers de recepción y transmisión del canal
        connect(&canal, &QSerialPort::readyRead, this, &MainWindow::handlerRecepcion);
        // Asocia el método handlerRecepcion de esta clase VentanaPrincipal al evento que
        // se produce cuando se ha recibido algo por el canal
    }
    ui->grafica->setRangoX(0, 20);  // En el eje X de 0 a 360 grados
    ui->grafica->setRangoYIzquierda(10,80);  // Rango en el eje izquierdo vertical
    ui->grafica->setRangoYDerecha(-100, 100);  // Rango en el eje derecho vertical
    QColor* pAzul = new QColor("blue");
    QColor* pVioleta = new QColor("purple");
    QColor* pRojo = new QColor("red");
    ui->grafica->setNGraficas(2, 1);  // Dos gráficas para el eje izquierdo y una para el derecho
    ui->grafica->setColorGraficaDerecha(0, pVioleta);  // Gráfica del eje derecho en rojo
    ui->grafica->setColorGraficaIzquierda(0, pAzul);  // Primera del eje izquierdo en azul
    ui->grafica->setColorGraficaIzquierda(1, pRojo);  // Segunda del eje izquierdo en auzl claro
    ui->grafica->setColorEjeDerecho(pVioleta);  // Eje derecho en rojo
    ui->grafica->setColorEjeIzquierdo(pAzul);  // Eje izquierdo en azul
}


MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::handlerRecepcion() {
double izq[2],dch;
float consigna,kp,ki,kd;
QString str;
// Se ejecuta cuando se recibe algo en el canal
bytesRecibidos.append(canal.readAll());
// Añade todos los bytes recibidos en el canal a la colección bytesRecibidos
if (bytesRecibidos.contains('\n')) { // Si se ha recibido un salto de línea '\n' ...
    QString cadena(bytesRecibidos);
    // Convierte los bytes recibidos a una cadena de caracteres
    cadena = cadena.trimmed();
    // Elimina posibles separadores al comienzo y al final de la cadena. Por ejemplo,
    // espacios en blanco y caracteres retorno de carro '\r' y alimentación de línea '\n'
    // Muestra el texto recibido en un QLabel de la ventana
    //QStringList lista = cadena.split(" ");
    QStringList list1 =cadena.split(' ');
    dch = list1[1].toInt();
    izq[0]=list1[0].toInt();
    izq[1]=list1[2].toInt();

    consigna=list1[0].toFloat();
    kp=list1[3].toFloat();
    kd=list1[4].toFloat();
    ki=list1[5].toFloat();
    str=QString::number(consigna);
    ui->consigna->setText(str);
    str=QString::number(kp);
    ui->kp->setText(str);
    str=QString::number(kd);
    ui->kd->setText(str);
    str=QString::number(ki);
    ui->ki->setText(str);

    int difference = time.elapsed()/1000;        //mseg
    if (difference<20)
        ui->grafica->anadePuntos(difference, izq, &dch);  // Añade los valores
    else
    {
    ui->grafica->remove();
    ui->grafica->anadePuntos(difference, izq, &dch);  // Añade los valores
    ui->grafica->setRangoX(ui->grafica->getTime(0),difference );
    }
bytesRecibidos.clear(); // Borra la colección de bytes para preparar otra recepción
}
}

