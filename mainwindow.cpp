#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_control = new BaslerCameraControl(this);
    m_control->initSome();
    connect(m_control, &BaslerCameraControl::sigCurrentImage, [=](QImage img){
        QPixmap pix = QPixmap::fromImage(img).transformed(m_matrix);
        ui->label->setPixmap(pix);
        ui->widget_pic->setFixedSize(pix.size());
    });
    connect(m_control, &BaslerCameraControl::sigSizeChange, [=](QSize size){
        // 默认大小641,494
        // ui->label_size->setText(QString("\345\260\272\345\257\270:%0*%1").arg(QString::number(size.width())).arg(QString::number(size.height()))); // 尺寸
        ui->widget_pic->setFixedSize(size);
    });
    m_control->OpenCamera(m_control->cameras().first());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_Start_clicked(bool checked)
{
    if(checked) {// 开始采集
        m_control->StartAcquire();
        ui->pushButton_Start->setText("\347\273\223\346\235\237\351\207\207\351\233\206");// 结束采集
    } else {
        m_control->StopAcquire();
        ui->pushButton_Start->setText("\345\274\200\345\247\213\351\207\207\351\233\206");// 开始采集
    }
}

