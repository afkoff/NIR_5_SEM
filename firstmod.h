#ifndef FIRSTMOD_H
#define FIRSTMOD_H

#include <QMainWindow>

class MainWindow;
namespace Ui {
class FirstMod;
}

class FirstMod : public QMainWindow
{
    Q_OBJECT

public:
    explicit FirstMod(QWidget *parent = nullptr);
    ~FirstMod();

private slots:
    void on_btnStart_clicked();

    void on_btnBack_clicked();


    void on_btnMod3_clicked();

private:
    Ui::FirstMod *ui;
    QMainWindow *window;
};

#endif // FIRSTMOD_H
