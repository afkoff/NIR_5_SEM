#include "firstmod.h"
#include "ui_firstmod.h"
#include "firstmod2.h"
#include "mainwindow.h"
#include "flight_model.h"
#include "firstmod3.h"
#include "secondwindow.h"

FirstMod::FirstMod(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::FirstMod)
{
    ui->setupUi(this);

    // Установим дефолтные значения (для удобства тестов, чтобы каждый раз не вводить рукам)
    ui->lineEdit_X_start->setText("0");
    ui->lineEdit_Y_start->setText("0");
    ui->lineEdit_Z_start->setText("0");
    ui->lineEdit_V_target->setText("60");

    ui->lineEdit_X_end->setText("1000");
    ui->lineEdit_Y_end->setText("1000");
    ui->lineEdit_Z_end->setText("1000");
}

FirstMod::~FirstMod()
{
    delete ui;
}

void FirstMod::on_btnStart_clicked()
{
    // 1. Считываем данные из полей ввода
    MissionParams params;

    // Преобразуем текст в числа
    params.startX = ui->lineEdit_X_start->text().toDouble();
    params.startY = ui->lineEdit_Y_start->text().toDouble();
    params.startZ = ui->lineEdit_Z_start->text().toDouble();
    params.targetV = ui->lineEdit_V_target->text().toDouble();

    params.targetX = ui->lineEdit_X_end->text().toDouble();
    params.targetY = ui->lineEdit_Y_end->text().toDouble();
    params.targetZ = ui->lineEdit_Z_end->text().toDouble();

    // 2. Скрываем текущее окно
    hide();

    // 3. Создаем окно симуляции, передаем туда параметры и показываем его
    window = new FirstMod2(params, this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}

void FirstMod::on_btnBack_clicked()
{
    hide();

    // Возвращаемся в Главное Меню
    window = new SecondWindow(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}

void FirstMod::on_btnMod3_clicked()
{
    // 1. Считывание
    MissionParams params;
    params.startX = ui->lineEdit_X_start->text().toDouble();
    params.startY = ui->lineEdit_Y_start->text().toDouble();
    params.startZ = ui->lineEdit_Z_start->text().toDouble();
    params.targetV = ui->lineEdit_V_target->text().toDouble();
    params.targetX = ui->lineEdit_X_end->text().toDouble();
    params.targetY = ui->lineEdit_Y_end->text().toDouble();
    params.targetZ = ui->lineEdit_Z_end->text().toDouble();

    hide();

    // 2. Создание окна с передачей параметров
    window = new FirstMod3(params, this);

    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}
