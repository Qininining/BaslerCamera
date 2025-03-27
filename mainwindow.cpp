#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_Camera = new BaslerCameraControl(this);
    m_Camera->initSome();


    connect(m_Camera, &BaslerCameraControl::sigCurrentImage, [=](QImage img){
        QPixmap pix = QPixmap::fromImage(img).transformed(m_matrix);
        ui->label_pic->setPixmap(pix);
        ui->widget_pic->setFixedSize(pix.size());
    });
    connect(m_Camera, &BaslerCameraControl::sigSizeChange, [=](QSize size) {
        // ui->label_size->setText(QString("尺寸: %0*%1").arg(size.width()).arg(size.height()));
        ui->widget_pic->setFixedSize(size);
    });

    const auto cameras = m_Camera->cameras(); // 获取相机列表
    ui->comboBox_CamList->addItems(cameras);


    if (!cameras.isEmpty()) { // 检查是否至少存在一个相机
        const QString& firstCameraSN = cameras.first(); // 获取第一个相机序列号
        if (!m_Camera->OpenCamera(firstCameraSN)) { // 尝试打开相机
            qDebug() << "成功打开相机: " << firstCameraSN;
            ui->label_pic->setText("成功打开相机！");
        } else {
            qCritical() << "无法打开相机: " << firstCameraSN;
            ui->label_pic->setText("无法打开相机!");
        }
    } else {
        qWarning() << "未检测到任何相机！";
        ui->label_pic->setText("未检测到相机，请连接设备！");
    }
}

MainWindow::~MainWindow() {
    // 安全释放相机资源
    if (m_Camera) {
        m_Camera->deleteAll(); // 关闭相机（假设存在该方法）
        delete m_Camera;
        m_Camera = nullptr;
    }
    delete ui;
}


void MainWindow::on_pushButton_Start_clicked(bool checked)
{
    if(checked) {// 开始采集
        m_Camera->StartAcquire();
        ui->pushButton_Start->setText("结束采集");// 结束采集
    } else {
        m_Camera->StopAcquire();
        ui->pushButton_Start->setText("开始采集");// 开始采集
    }
}

