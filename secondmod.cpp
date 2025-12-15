#include "secondmod.h"
#include "ui_secondmod.h"
#include "secondwindow.h"
#include "secondmod2.h"
#include "secondmod3.h"
#include "flight_model.h"

SecondMod::SecondMod(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::SecondMod)
{
    ui->setupUi(this);
    // Дефолтные значения
    ui->lineEdit_startX->setText("0"); ui->lineEdit_startY->setText("0"); ui->lineEdit_startZ->setText("0");
    ui->lineEdit_p1X->setText("1000"); ui->lineEdit_p1Y->setText("1000"); ui->lineEdit_p1Z->setText("1000");
    ui->lineEdit_p2X->setText("2000"); ui->lineEdit_p2Y->setText("2000"); ui->lineEdit_p2Z->setText("2000");
    ui->lineEdit_V->setText("60");
}

SecondMod::~SecondMod()
{
    delete ui;
}

void SecondMod::on_pushButton_clicked()
{
    hide();
    window = new SecondWindow(this);
    window->show();
}


void SecondMod::on_pushButton_2_clicked()
{
    MissionParamsMode2 params;
    params.startX = ui->lineEdit_startX->text().toDouble();
    params.startY = ui->lineEdit_startY->text().toDouble();
    params.startZ = ui->lineEdit_startZ->text().toDouble();

    params.p1X = ui->lineEdit_p1X->text().toDouble();
    params.p1Y = ui->lineEdit_p1Y->text().toDouble();
    params.p1Z = ui->lineEdit_p1Z->text().toDouble();

    params.p2X = ui->lineEdit_p2X->text().toDouble();
    params.p2Y = ui->lineEdit_p2Y->text().toDouble();
    params.p2Z = ui->lineEdit_p2Z->text().toDouble();

    params.targetV = ui->lineEdit_V->text().toDouble();

    hide();
    window = new SecondMod2(params, this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}


void SecondMod::on_on_btn2D_clicked_clicked()
{
    MissionParamsMode2 params;
    params.startX = ui->lineEdit_startX->text().toDouble();
    params.startY = ui->lineEdit_startY->text().toDouble();
    params.startZ = ui->lineEdit_startZ->text().toDouble();

    params.p1X = ui->lineEdit_p1X->text().toDouble();
    params.p1Y = ui->lineEdit_p1Y->text().toDouble();
    params.p1Z = ui->lineEdit_p1Z->text().toDouble();

    params.p2X = ui->lineEdit_p2X->text().toDouble();
    params.p2Y = ui->lineEdit_p2Y->text().toDouble();
    params.p2Z = ui->lineEdit_p2Z->text().toDouble();

    params.targetV = ui->lineEdit_V->text().toDouble();

    hide();
    window = new SecondMod3(params, this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}

