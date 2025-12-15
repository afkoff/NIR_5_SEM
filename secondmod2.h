#ifndef SECONDMOD2_H
#define SECONDMOD2_H

#include <QMainWindow>
#include "flight_model.h"

class SecondMod;
namespace Ui {
class SecondMod2;
}

class SecondMod2 : public QMainWindow
{
    Q_OBJECT

public:
    explicit SecondMod2(MissionParamsMode2 params, QWidget *parent = nullptr);
    ~SecondMod2();

private slots:
    void on_pushButton_clicked();

private:
    Ui::SecondMod2 *ui;
    SecondMod *window;

    void runSimulation(MissionParamsMode2 params);
};

#endif // SECONDMOD2_H
