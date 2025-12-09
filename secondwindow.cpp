#include "secondwindow.h"
#include "ui_secondwindow.h"
#include "mainwindow.h"
#include "firstmod.h"
#include "secondmod.h"

SecondWindow::SecondWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::SecondWindow)
    , window(nullptr)
{
    ui->setupUi(this);
}

SecondWindow::~SecondWindow()
{
    delete ui;
}

// Переход на MainWindow
void SecondWindow::on_pushButton_clicked()
{
    hide();
    window = new MainWindow(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}

// Переход на FirstMod
void SecondWindow::on_pushButton_2_clicked()
{
    hide();
    window = new FirstMod(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}

// Переход на SecondMod
void SecondWindow::on_pushButton_3_clicked()
{
    hide();
    window = new SecondMod(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}
