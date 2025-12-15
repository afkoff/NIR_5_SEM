#ifndef SECONDMOD3_H
#define SECONDMOD3_H

#include <QMainWindow>
#include "flight_model.h"

class SecondMod;
namespace Ui {
class SecondMod3;
}

class SecondMod3 : public QMainWindow
{
    Q_OBJECT

public:
    explicit SecondMod3(MissionParamsMode2 params, QWidget *parent = nullptr);
    ~SecondMod3();

private slots:
    void on_pushButton_clicked();

private:
    Ui::SecondMod3 *ui;
    SecondMod *window;

    void runSimulationAndPlot(MissionParamsMode2 params);
};

#endif // SECONDMOD3_H
