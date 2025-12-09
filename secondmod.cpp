#include "secondmod.h"
#include "ui_secondmod.h"
#include "secondwindow.h"
#include "secondmod2.h"

SecondMod::SecondMod(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::SecondMod)
{
    ui->setupUi(this);
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
    hide();
    window = new SecondMod2(this);
    window->show();
}

