#ifndef SECONDMOD_H
#define SECONDMOD_H

#include <QMainWindow>
class MainWindow;
namespace Ui {
class SecondMod;
}

class SecondMod : public QMainWindow
{
    Q_OBJECT

public:
    explicit SecondMod(QWidget *parent = nullptr);
    ~SecondMod();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::SecondMod *ui;
    QMainWindow *window;
};

#endif // SECONDMOD_H
