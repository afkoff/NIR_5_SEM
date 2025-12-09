#include "secondmod2.h"
#include "ui_secondmod2.h"
#include "secondmod.h"

SecondMod2::SecondMod2(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::SecondMod2)
{
    ui->setupUi(this);
}

SecondMod2::~SecondMod2()
{
    delete ui;
}

void SecondMod2::on_pushButton_clicked()
{
    hide();
    window = new SecondMod(this);
    window->show();
}


