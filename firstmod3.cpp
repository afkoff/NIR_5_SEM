#include "firstmod3.h"
#include "ui_firstmod3.h"
#include "firstmod.h"

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

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

FirstMod3::FirstMod3(MissionParams params, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::FirstMod3)
{
    ui->setupUi(this);
    runSimulationAndPlot(params);
}

FirstMod3::~FirstMod3()
{
    delete ui;
}

QChartView* createChartWidget(const QString& title, const QString& yAxisTitle,
                                        const std::vector<double>& times,
                                        const std::vector<double>& values,
                                        QColor color)
{
    QChart *chart = new QChart();
    QLineSeries *series = new QLineSeries();

    series->setName(title);
    series->setColor(color);

    for (size_t i = 0; i < times.size(); ++i) {
        series->append(times[i], values[i]);
    }

    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle(title);

    if (!chart->axes(Qt::Horizontal).isEmpty()) {
        QValueAxis *axisX = static_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
        axisX->setTitleText("Время t [сек]");
    }
    if (!chart->axes(Qt::Vertical).isEmpty()) {
        QValueAxis *axisY = static_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
        axisY->setTitleText(yAxisTitle);
    }

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setMinimumHeight(300);
    return chartView;
}

void FirstMod3::runSimulationAndPlot(MissionParams params)
{
    AircraftModel model(params);
    state_type x(13);
    x[0] = params.startX; x[1] = params.startY; x[2] = params.startZ; x[3] = params.targetV;

    double dx = params.targetX - params.startX;
    double dy = params.targetY - params.startY;
    double dz = params.targetZ - params.startZ;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    double dist_h = std::sqrt(dx*dx + dz*dz);

    x[4] = std::atan2(dy, dist_h);
    x[5] = std::atan2(-dz, dx);
    for(int i=6; i<13; ++i) x[i] = 0;

    double sim_time = (dist / params.targetV) * 1.1;

    std::vector<double> times;
    std::vector<state_type> states;
    runge_kutta_dopri5<state_type> stepper;

    integrate_adaptive(make_controlled(1E-3, 1E-3, stepper), model, x, 0.0, sim_time, 0.1, Observer(times, states));

    std::vector<double> vals_X, vals_Y, vals_Z, vals_V;
    for (const auto& s : states) {
        vals_X.push_back(s[0]);
        vals_Y.push_back(s[1]);
        vals_Z.push_back(s[2]);
        vals_V.push_back(s[3]);
    }

    QVBoxLayout *layout = qobject_cast<QVBoxLayout*>(ui->scrollAreaWidgetContents->layout());
    if (!layout) {
        layout = new QVBoxLayout(ui->scrollAreaWidgetContents);
    }

    QLayoutItem *item;
    while ((item = layout->takeAt(0)) != nullptr) {
        if (item->widget()) delete item->widget();
        delete item;
    }

    layout->addWidget(createChartWidget("Скорость (V)", "м/с", times, vals_V, Qt::magenta));
    layout->addWidget(createChartWidget("Координата X", "м", times, vals_X, Qt::red));
    layout->addWidget(createChartWidget("Координата Y", "м", times, vals_Y, Qt::green));
    layout->addWidget(createChartWidget("Координата Z", "м", times, vals_Z, Qt::blue));

    layout->addStretch();
}

void FirstMod3::on_pushButton_clicked()
{
    hide();
    window = new FirstMod(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}
