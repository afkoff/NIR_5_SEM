#ifndef FIRSTMOD2_H
#define FIRSTMOD2_H

#include <QMainWindow>
#include "flight_model.h"

class FirstMod; // Forward declaration

namespace Ui {
class FirstMod2;
}

class FirstMod2 : public QMainWindow
{
    Q_OBJECT

public:
    explicit FirstMod2(MissionParams params, QWidget *parent = nullptr);
    ~FirstMod2();

private slots:
    void on_pushButton_clicked();

private:
    Ui::FirstMod2 *ui;
    FirstMod *window;

    void runSimulation(MissionParams params);
};

#endif // FIRSTMOD2_H
