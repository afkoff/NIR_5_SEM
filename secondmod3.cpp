#include "secondmod3.h"
#include "ui_secondmod3.h"
#include "secondmod.h"

// --- ПОДКЛЮЧЕНИЕ QT CHARTS ---
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
// -----------------------------

#include <QVBoxLayout>
#include <boost/numeric/odeint.hpp>
#include <cmath>
#include <algorithm>
#include "flight_model.h"

using namespace boost::numeric::odeint;

// Наблюдатель
struct Observer {
    std::vector<double>& m_times;
    std::vector<state_type>& m_states;

    Observer(std::vector<double>& times, std::vector<state_type>& states)
        : m_times(times), m_states(states) { }

    void operator()(const state_type &x, double t) {
        m_times.push_back(t);
        m_states.push_back(x);
    }
};


// Правильный конструктор
SecondMod3::SecondMod3(MissionParamsMode2 params, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::SecondMod3)
{
    ui->setupUi(this);
    runSimulationAndPlot(params);
}

SecondMod3::~SecondMod3()
{
    delete ui;
}

// Функция-помощник для создания графика (копия из FirstMod3)
QChartView* createChartWidget2(const QString& title, const QString& yAxisTitle,
                                         const std::vector<double>& times,
                                         const std::vector<double>& values,
                                         QColor color)
{
    QChart *chart = new QChart();
    QLineSeries *series = new QLineSeries();

    series->setName(title);
    series->setColor(color);
    QPen pen = series->pen();
    pen.setWidth(2);
    series->setPen(pen);

    for (size_t i = 0; i < times.size(); ++i) {
        series->append(times[i], values[i]);
    }

    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle(title);
    chart->legend()->hide();

    if (!chart->axes(Qt::Horizontal).isEmpty()) {
        QValueAxis *axisX = static_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
        axisX->setTitleText("Время t [сек]");
        axisX->setGridLineVisible(true);
    }
    if (!chart->axes(Qt::Vertical).isEmpty()) {
        QValueAxis *axisY = static_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
        axisY->setTitleText(yAxisTitle);
        axisY->setGridLineVisible(true);
    }

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setMinimumHeight(300);
    return chartView;
}

void SecondMod3::runSimulationAndPlot(MissionParamsMode2 params)
{

    std::vector<double> times;
    std::vector<state_type> states;
    runge_kutta_dopri5<state_type> stepper;

    // ЭТАП 1: Полет от СТАРТА к ТОЧКЕ 1

    state_type x(13);
    x[0] = params.startX; x[1] = params.startY; x[2] = params.startZ; x[3] = params.targetV;

    // Вектор на P1
    double dx1 = params.p1X - params.startX;
    double dy1 = params.p1Y - params.startY;
    double dz1 = params.p1Z - params.startZ;
    double dist1 = std::sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1);
    double dist1_h = std::sqrt(dx1*dx1 + dz1*dz1);

    // Расчет идеального математического угла
    double target_theta1 = std::atan2(dy1, dist1_h);

    x[4] = std::atan2(dy1, dist1_h);
    x[5] = std::atan2(-dz1, dx1);
    x[6] = std::sin(target_theta1);
    x[7] = std::cos(target_theta1);
    for(int i=8; i<13; ++i) x[i] = 0;
    x[3] = 0;

    MissionParams leg1;
    leg1.targetX = params.p1X; leg1.targetY = params.p1Y; leg1.targetZ = params.p1Z;
    leg1.targetV = params.targetV;

    double time1 = (dist1 / params.targetV) * 1.5;

    AircraftModel model1(leg1);
    integrate_adaptive(make_controlled(1E-3, 1E-3, stepper), model1, x, 0.0, time1, 0.1, Observer(times, states));

    // Обрезка по Точке 1
    QVector3D p1(params.p1X, params.p1Y, params.p1Z);
    double min_dist = 1e9;
    size_t split_idx = states.size() - 1;
    for(size_t i=0; i<states.size(); ++i) {
        QVector3D cur(states[i][0], states[i][1], states[i][2]);
        double d = cur.distanceToPoint(p1);
        if (d < min_dist) { min_dist = d; split_idx = i; }
    }
    states.resize(split_idx + 1);
    times.resize(split_idx + 1);

    // ЭТАП 2: Полет от ТОЧКИ 1 к ТОЧКЕ 2

    state_type x2 = states.back();
    double currentTime = times.back();

    MissionParams leg2;
    leg2.targetX = params.p2X; leg2.targetY = params.p2Y; leg2.targetZ = params.p2Z;
    leg2.targetV = params.targetV;

    double dx2 = params.p2X - params.p1X;
    double dy2 = params.p2Y - params.p1Y;
    double dz2 = params.p2Z - params.p1Z;
    double dist2 = std::sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2);

    double dist2_h = std::sqrt(dx2*dx2 + dz2*dz2);

    // Расчет идеального математического угла
    double target_theta2 = std::atan2(dy2, dist2_h);

    x2[4] = target_theta2;
    x2[5] = std::atan2(-dz2, dx2);    // Новый курс (psi)
    x2[6] = std::sin(target_theta2);
    x2[7] = std::cos(target_theta2);

    for(int i = 8; i < 13; ++i) {
        x2[i] = 0.0;
    }
    // Восстанавливаем целевую скорость (на всякий случай)
    x2[3] = params.targetV;

    double time2 = (dist2 / params.targetV) * 1.5;

    AircraftModel model2(leg2);
    integrate_adaptive(make_controlled(1E-3, 1E-3, stepper), model2, x2, currentTime, currentTime + time2, 0.1, Observer(times, states));

    // Обрезка по Точке 2
    QVector3D p2(params.p2X, params.p2Y, params.p2Z);
    min_dist = 1e9;
    size_t end_idx = states.size() - 1;
    for(size_t i=split_idx; i<states.size(); ++i) {
        QVector3D cur(states[i][0], states[i][1], states[i][2]);
        double d = cur.distanceToPoint(p2);
        if (d < min_dist) { min_dist = d; end_idx = i; }
    }

    size_t plot_end = end_idx + 20; // +2 секунды запаса
    if (plot_end > states.size()) plot_end = states.size();

    // ПОДГОТОВКА ДАННЫХ И ОТРИСОВКА

    std::vector<double> vals_X, vals_Y, vals_Z, vals_V, plot_times;

    for (size_t i = 0; i < plot_end; ++i) {
        plot_times.push_back(times[i]);
        vals_X.push_back(states[i][0]);
        vals_Y.push_back(states[i][1]);
        vals_Z.push_back(states[i][2]);
        vals_V.push_back(states[i][3] );
    }

    QVBoxLayout *layout = qobject_cast<QVBoxLayout*>(ui->scrollAreaWidgetContents->layout());
    if (!layout) layout = new QVBoxLayout(ui->scrollAreaWidgetContents);

    // Очистка
    QLayoutItem *item;
    while ((item = layout->takeAt(0)) != nullptr) {
        if (item->widget()) delete item->widget();
        delete item;
    }

    // Добавление графиков
    layout->addWidget(createChartWidget2("Скорость (V)", "[м/с]", plot_times, vals_V, Qt::magenta));
    layout->addWidget(createChartWidget2("Координата X", "X [м]", plot_times, vals_X, Qt::red));
    layout->addWidget(createChartWidget2("Координата Y", "Y [м]", plot_times, vals_Y, Qt::green));
    layout->addWidget(createChartWidget2("Координата Z", "Z [м]", plot_times, vals_Z, Qt::blue));

    layout->addStretch();
}

void SecondMod3::on_pushButton_clicked()
{
    hide();
    window = new SecondMod(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}
