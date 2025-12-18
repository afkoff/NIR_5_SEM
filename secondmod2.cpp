#include "secondmod2.h"
#include "ui_secondmod2.h"
#include "secondmod.h"

// --- ПОДКЛЮЧЕНИЕ МОДУЛЕЙ QT 3D ---
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DCore/QEntity>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QExtrudedTextMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/QForwardRenderer>

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

SecondMod2::SecondMod2(MissionParamsMode2 params, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::SecondMod2)
{
    ui->setupUi(this);
    runSimulation(params);
}

SecondMod2::~SecondMod2()
{
    delete ui;
}

// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (Копии из FirstMod2)

void createSphere(Qt3DCore::QEntity *root, QVector3D pos, QColor color, float radius) {
    Qt3DCore::QEntity *entity = new Qt3DCore::QEntity(root);
    Qt3DExtras::QSphereMesh *mesh = new Qt3DExtras::QSphereMesh();
    mesh->setRadius(radius);

    Qt3DExtras::QPhongMaterial *mat = new Qt3DExtras::QPhongMaterial();
    mat->setDiffuse(color);
    mat->setAmbient(color);

    Qt3DCore::QTransform *trans = new Qt3DCore::QTransform();
    trans->setTranslation(pos);

    entity->addComponent(mesh);
    entity->addComponent(mat);
    entity->addComponent(trans);
}

void createCylinderBetweenPoints(Qt3DCore::QEntity *root, QVector3D start, QVector3D end, QColor color, float thickness) {
    Qt3DCore::QEntity *entity = new Qt3DCore::QEntity(root);
    QVector3D diff = end - start;
    float length = diff.length();
    if (length < 0.001f) return;

    Qt3DExtras::QCylinderMesh *mesh = new Qt3DExtras::QCylinderMesh();
    mesh->setRadius(thickness);
    mesh->setLength(length);
    mesh->setRings(2); mesh->setSlices(8);

    Qt3DExtras::QPhongMaterial *mat = new Qt3DExtras::QPhongMaterial();
    mat->setDiffuse(color);
    mat->setAmbient(color);

    Qt3DCore::QTransform *trans = new Qt3DCore::QTransform();
    trans->setTranslation(start + diff / 2.0f);
    QQuaternion rotation = QQuaternion::rotationTo(QVector3D(0, 1, 0), diff.normalized());
    trans->setRotation(rotation);

    entity->addComponent(mesh);
    entity->addComponent(mat);
    entity->addComponent(trans);
}

void createLabel(Qt3DCore::QEntity *root, QVector3D pos, QString text, QColor color, float scale) {
    Qt3DCore::QEntity *textEntity = new Qt3DCore::QEntity(root);
    Qt3DExtras::QExtrudedTextMesh *textMesh = new Qt3DExtras::QExtrudedTextMesh();
    textMesh->setText(text);
    textMesh->setDepth(0.1f);

    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(color);
    material->setAmbient(color);

    Qt3DCore::QTransform *transform = new Qt3DCore::QTransform();

    // Центрирование текста
    float approxWidth = text.length() * 0.55f * scale;
    float approxHeight = scale;
    QVector3D centeredOffset(-approxWidth / 2.0f, -approxHeight / 2.0f, 0.0f);

    transform->setTranslation(pos + centeredOffset);
    transform->setScale(scale);

    textEntity->addComponent(textMesh);
    textEntity->addComponent(material);
    textEntity->addComponent(transform);
}

// АДАПТИВНАЯ ОБЪЕМНАЯ СЕТКА
void createVolumetricGridAndLabels(Qt3DCore::QEntity *root, float maxSize) {
    float targetStep = maxSize / 6.0f;
    float magnitude = std::pow(10.0f, std::floor(std::log10(targetStep)));
    float fraction = targetStep / magnitude;
    float step;
    if (fraction < 1.5f) step = 1.0f * magnitude;
    else if (fraction < 3.0f) step = 2.0f * magnitude;
    else if (fraction < 7.0f) step = 5.0f * magnitude;
    else step = 10.0f * magnitude;
    if (step < 1.0f) step = 1.0f;

    QColor gridColor(60, 60, 60);
    float thickness = maxSize / 2000.0f;
    if (thickness < 0.2f) thickness = 0.2f;

    float gridLimit = std::ceil(maxSize / step) * step;

    for (float i = 0; i <= gridLimit + 0.001f; i += step) {
        for (float j = 0; j <= gridLimit + 0.001f; j += step) {
            createCylinderBetweenPoints(root, QVector3D(i, j, 0), QVector3D(i, j, gridLimit), gridColor, thickness);
            createCylinderBetweenPoints(root, QVector3D(i, 0, j), QVector3D(i, gridLimit, j), gridColor, thickness);
            createCylinderBetweenPoints(root, QVector3D(0, i, j), QVector3D(gridLimit, i, j), gridColor, thickness);
        }
    }

    for (float i = step; i <= gridLimit + 0.001f; i += step) {
        QString num = QString::number((int)i);
        float textScale = step * 0.2f;
        createLabel(root, QVector3D(i, -textScale*1.5f, 0), num, Qt::red, textScale);
        createLabel(root, QVector3D(-textScale*2.5f, i, 0), num, Qt::green, textScale);
        createLabel(root, QVector3D(-textScale*2.5f, 0, i), num, Qt::blue, textScale);
    }
}

// ОСНОВНАЯ ЛОГИКА (2 РЕЖИМ)

void SecondMod2::runSimulation(MissionParamsMode2 params)
{

    std::vector<double> times;
    std::vector<state_type> states;
    runge_kutta_dopri5<state_type> stepper;

    // ЭТАП 1: Старт -> Точка 1 (P1)

    // Инициализация
    state_type x(13);
    x[0] = params.startX; x[1] = params.startY; x[2] = params.startZ; x[3] = params.targetV;

    // Расчет углов для P1
    double dx1 = params.p1X - params.startX;
    double dy1 = params.p1Y - params.startY;
    double dz1 = params.p1Z - params.startZ;
    double dist1 = std::sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1);
    double dist1_h = std::sqrt(dx1*dx1 + dz1*dz1);

    // Расчет идеального математического угла
    double target_theta1 = std::atan2(dy1, dist1_h);

    x[4] = target_theta1;
    x[5] = std::atan2(-dz1, dx1);
    x[6] = std::sin(target_theta1);
    x[7] = std::cos(target_theta1);
    for(int i=8; i<13; ++i) x[i] = 0;

    x[3] = 0;

    // Время с запасом
    double time1 = dist1 / params.targetV * 1.5;

    // Модель для участка 1
    MissionParams leg1;
    leg1.targetX = params.p1X; leg1.targetY = params.p1Y; leg1.targetZ = params.p1Z; leg1.targetV = params.targetV;
    AircraftModel model1(leg1);

    // Интеграция 1
    integrate_adaptive(make_controlled(1E-3, 1E-3, stepper), model1, x, 0.0, time1, 0.2, Observer(times, states));

    // Обрезка по Точке 1
    QVector3D p1(params.p1X, params.p1Y, params.p1Z);
    double min_dist = 1e9;
    size_t split_idx = states.size() - 1;
    for(size_t i=0; i<states.size(); ++i) {
        QVector3D cur(states[i][0], states[i][1], states[i][2]);
        double d = cur.distanceToPoint(p1);
        if (d < min_dist) { min_dist = d; split_idx = i; }
    }
    // Оставляем данные только до P1
    states.resize(split_idx + 1);
    times.resize(split_idx + 1);


    // ЭТАП 2: Точка 1 -> Точка 2 (P2)

    // Берем последнее состояние как стартовое
    state_type x2 = states.back();
    double currentTime = times.back();

    // Расчет расстояния для P2
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

    double time2 = dist2 / params.targetV * 1.5;

    // Модель для участка 2
    MissionParams leg2;
    leg2.targetX = params.p2X; leg2.targetY = params.p2Y; leg2.targetZ = params.p2Z; leg2.targetV = params.targetV;
    AircraftModel model2(leg2);

    // Интеграция 2 (продолжаем во времени)
    integrate_adaptive(make_controlled(1E-3, 1E-3, stepper), model2, x2, currentTime, currentTime + time2, 0.2, Observer(times, states));

    // Обрезка по Точке 2
    QVector3D p2(params.p2X, params.p2Y, params.p2Z);
    min_dist = 1e9;
    size_t end_idx = states.size() - 1;
    // Ищем минимум начиная с конца первого участка
    for(size_t i=split_idx; i<states.size(); ++i) {
        QVector3D cur(states[i][0], states[i][1], states[i][2]);
        double d = cur.distanceToPoint(p2);
        if (d < min_dist) { min_dist = d; end_idx = i; }
    }
    if (end_idx < states.size()) states.resize(end_idx + 1);


    // 3. 3D ВИЗУАЛИЗАЦИЯ

    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x202020)));
    QWidget *container = QWidget::createWindowContainer(view);
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

    // Камера
    Qt3DRender::QCamera *camera = view->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 10000000.0f);

    // Масштаб сцены (учитываем все 3 точки)
    float maxCoord = std::max({
        std::abs(params.startX)*1.1, std::abs(params.startY)*1.1, std::abs(params.startZ)*1.1,
        std::abs(params.p1X)*1.1, std::abs(params.p1Y)*1.1, std::abs(params.p1Z)*1.1,
        std::abs(params.p2X)*1.1, std::abs(params.p2Y)*1.1, std::abs(params.p2Z)*1.1,
        std::abs(params.startX)*1.1, std::abs(params.startY)*1.1, std::abs(params.startZ)*1.1
    });
    if (maxCoord < 10.0f) maxCoord = 100.0f;

    // Сетка и шаг
    float tStep = maxCoord / 6.0f;
    float mag = std::pow(10.0f, std::floor(std::log10(tStep)));
    float fr = tStep / mag;
    float st = (fr < 1.5f) ? 1.0f*mag : (fr < 3.0f ? 2.0f*mag : (fr < 7.0f ? 5.0f*mag : 10.0f*mag));
    float gridEnd = std::ceil(maxCoord / st) * st;

    camera->setPosition(QVector3D(-gridEnd*0.6f, gridEnd*0.8f, gridEnd*1.5f));
    camera->setViewCenter(QVector3D(gridEnd/3.0f, gridEnd/3.0f, gridEnd/3.0f));

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(rootEntity);
    camController->setCamera(camera);
    camController->setLinearSpeed(gridEnd);
    camController->setLookSpeed(180.0f);

    // Свет
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white"); light->setIntensity(2.0f);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTrans = new Qt3DCore::QTransform(lightEntity);
    lightTrans->setTranslation(QVector3D(-gridEnd, gridEnd, gridEnd));
    lightEntity->addComponent(lightTrans);

    Qt3DCore::QEntity *lightEntity2 = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light2 = new Qt3DRender::QPointLight(lightEntity2);
    light2->setColor("white"); light2->setIntensity(1.5f);
    lightEntity2->addComponent(light2);
    Qt3DCore::QTransform *lightTrans2 = new Qt3DCore::QTransform(lightEntity2);
    lightTrans2->setTranslation(QVector3D(gridEnd, gridEnd, -gridEnd));
    lightEntity2->addComponent(lightTrans2);

    // Сетка
    createVolumetricGridAndLabels(rootEntity, maxCoord);

    // Подписи осей
    float labelPos = gridEnd + (gridEnd * 0.05f);
    float labelScale = gridEnd * 0.05f;
    createLabel(rootEntity, QVector3D(labelPos, 0, 0), "X", Qt::red, labelScale);
    createLabel(rootEntity, QVector3D(0, labelPos, 0), "Y", Qt::green, labelScale);
    createLabel(rootEntity, QVector3D(0, 0, labelPos), "Z", Qt::blue, labelScale);

    // Оси
    float objectScale = gridEnd / 150.0f;
    createCylinderBetweenPoints(rootEntity, QVector3D(0,0,0), QVector3D(gridEnd, 0, 0), Qt::red, objectScale);
    createCylinderBetweenPoints(rootEntity, QVector3D(0,0,0), QVector3D(0, gridEnd, 0), Qt::green, objectScale);
    createCylinderBetweenPoints(rootEntity, QVector3D(0,0,0), QVector3D(0, 0, gridEnd), Qt::blue, objectScale);

    // Траектория
    if (states.size() > 1) {
        for (size_t i = 0; i < states.size() - 1; ++i) {
            QVector3D p1(states[i][0], states[i][1], states[i][2]);
            QVector3D p2(states[i+1][0], states[i+1][1], states[i+1][2]);
            createCylinderBetweenPoints(rootEntity, p1, p2, Qt::cyan, objectScale * 0.6f);
        }
    }

    // Точки маршрута: Старт, P1, P2
    QVector3D startPos(params.startX, params.startY, params.startZ);
    createSphere(rootEntity, startPos, Qt::green, objectScale * 3.0f);
    createSphere(rootEntity, p1, Qt::yellow, objectScale * 3.0f); // Промежуточная точка
    createSphere(rootEntity, p2, Qt::red, objectScale * 3.0f);    // Финиш

    view->setRootEntity(rootEntity);

    // Вставка в UI
    if (ui->chartContainer != nullptr) {
        if (ui->chartContainer->layout()) {
            QLayoutItem* item;
            while ((item = ui->chartContainer->layout()->takeAt(0)) != nullptr) {
                delete item->widget(); delete item;
            }
            delete ui->chartContainer->layout();
        }
        QVBoxLayout *layout = new QVBoxLayout(ui->chartContainer);
        layout->addWidget(container);
    }
    else {
        if (ui->centralwidget->layout()) {
            ui->centralwidget->layout()->addWidget(container);
        } else {
            container->setParent(this);
            container->resize(800, 600);
            container->show();
        }
    }
}

void SecondMod2::on_pushButton_clicked()
{
    hide();
    window = new SecondMod(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}
