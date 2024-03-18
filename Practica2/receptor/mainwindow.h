#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QElapsedTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
void handlerRecepcion();

private:
    Ui::MainWindow *ui;
    QSerialPort canal; // Objeto para manejar el canal serie
    QByteArray bytesRecibidos; // Colección donde se almacenan los bytes recibidos por el canal
    QElapsedTimer time;

};
#endif // MAINWINDOW_H
