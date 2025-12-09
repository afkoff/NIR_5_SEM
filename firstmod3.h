#ifndef FIRSTMOD3_H
#define FIRSTMOD3_H

#include <QMainWindow>
#include "flight_model.h"

class FirstMod; // Forward declaration

namespace Ui {
class FirstMod3;
}

class FirstMod3 : public QMainWindow
{
    Q_OBJECT

public:
    explicit FirstMod3(MissionParams params, QWidget *parent = nullptr);
    ~FirstMod3();

private slots:
    void on_pushButton_clicked();

private:
    Ui::FirstMod3 *ui;
    FirstMod *window;

    void runSimulationAndPlot(MissionParams params);
};

#endif // FIRSTMOD3_H
