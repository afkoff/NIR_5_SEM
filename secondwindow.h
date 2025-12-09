#ifndef SECONDWINDOW_H
#define SECONDWINDOW_H

#include <QMainWindow>

class MainWindow;
class FirstMod;
class SecondMod;

namespace Ui {
class SecondWindow;
}

class SecondWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit SecondWindow(QWidget *parent = nullptr);
    ~SecondWindow();

private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();

private:
    Ui::SecondWindow *ui;
    QMainWindow *window;
};

#endif // SECONDWINDOW_H
