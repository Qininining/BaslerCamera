#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTransform>
#include "BaslerCameraControl.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_Start_clicked(bool checked);

private:
    Ui::MainWindow *ui;
    BaslerCameraControl* m_Camera = Q_NULLPTR;
    QTransform  m_matrix;
};
#endif // MAINWINDOW_H
