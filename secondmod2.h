#ifndef SECONDMOD2_H
#define SECONDMOD2_H

#include <QMainWindow>
class SecondMod;
namespace Ui {
class SecondMod2;
}

class SecondMod2 : public QMainWindow
{
    Q_OBJECT

public:
    explicit SecondMod2(QWidget *parent = nullptr);
    ~SecondMod2();

private slots:
    void on_pushButton_clicked();

private:
    Ui::SecondMod2 *ui;
    SecondMod *window;
};

#endif // SECONDMOD2_H
