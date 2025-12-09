#include "firstmod2.h"
#include "ui_firstmod2.h"
#include "firstmod.h"

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

FirstMod2::FirstMod2(MissionParams params, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::FirstMod2)
{
    ui->setupUi(this);
    runSimulation(params);
}

FirstMod2::~FirstMod2()
{
    delete ui;
}

// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ

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
    mesh->setRings(2); mesh->setSlices(6); // Оптимизация (меньше граней для сетки)

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
    transform->setTranslation(pos);
    transform->setScale(scale);

    textEntity->addComponent(textMesh);
    textEntity->addComponent(material);
    textEntity->addComponent(transform);
}

// АДАПТИВНАЯ ОБЪЕМНАЯ СЕТКА (LATTICE)
void createVolumetricGridAndLabels(Qt3DCore::QEntity *root, float maxSize) {
    // 1. АВТОМАТИЧЕСКИЙ РАСЧЕТ ШАГА
    // Целимся в 5-6 делений, чтобы не перегружать сцену тысячами линий
    float targetStep = maxSize / 6.0f;

    float magnitude = std::pow(10.0f, std::floor(std::log10(targetStep)));
    float fraction = targetStep / magnitude;

    float step;
    if (fraction < 1.5f) step = 1.0f * magnitude;
    else if (fraction < 3.0f) step = 2.0f * magnitude;
    else if (fraction < 7.0f) step = 5.0f * magnitude;
    else step = 10.0f * magnitude;

    if (step < 1.0f) step = 1.0f;

    QColor gridColor(60, 60, 60); // Темно-серый

    // Линии сетки делаем тонкими
    float thickness = maxSize / 2000.0f;
    if (thickness < 0.2f) thickness = 0.2f;

    // Округляем границу
    float gridLimit = std::ceil(maxSize / step) * step;

    // 2. РИСУЕМ ОБЪЕМНУЮ РЕШЕТКУ (ВНУТРИ КУБА)
    // Используем вложенные циклы, чтобы линии шли насквозь через весь объем
    for (float i = 0; i <= gridLimit + 0.001f; i += step) {
        for (float j = 0; j <= gridLimit + 0.001f; j += step) {
            // Линии, параллельные Z (на всех пересечениях X=i и Y=j)
            createCylinderBetweenPoints(root, QVector3D(i, j, 0), QVector3D(i, j, gridLimit), gridColor, thickness);

            // Линии, параллельные Y (на всех пересечениях X=i и Z=j)
            createCylinderBetweenPoints(root, QVector3D(i, 0, j), QVector3D(i, gridLimit, j), gridColor, thickness);

            // Линии, параллельные X (на всех пересечениях Y=i и Z=j)
            createCylinderBetweenPoints(root, QVector3D(0, i, j), QVector3D(gridLimit, i, j), gridColor, thickness);
        }
    }

    // 3. ПОДПИСИ (Только по краям, чтобы не мешать обзору)
    for (float i = step; i <= gridLimit + 0.001f; i += step) {
        QString num = QString::number((int)i);
        float textScale = step * 0.2f;

        createLabel(root, QVector3D(i, -textScale*1.5f, 0), num, Qt::red, textScale);
        createLabel(root, QVector3D(-textScale*2.5f, i, 0), num, Qt::green, textScale);
        createLabel(root, QVector3D(-textScale*2.5f, 0, i), num, Qt::blue, textScale);
    }
}

// ОСНОВНАЯ ЛОГИКА

void FirstMod2::runSimulation(MissionParams params)
{

    // 1. РАСЧЕТ
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

    double sim_time = (dist / params.targetV) * 1.3;

    std::vector<double> times;
    std::vector<state_type> states;
    runge_kutta_dopri5<state_type> stepper;

    integrate_adaptive(make_controlled(1E-3, 1E-3, stepper), model, x, 0.0, sim_time, 0.2, Observer(times, states));

    // ОБРЕЗКА ТРАЕКТОРИИ
    QVector3D targetPos(params.targetX, params.targetY, params.targetZ);
    double min_dist = 1e9;
    size_t closest_idx = states.size() - 1;

    for (size_t i = 0; i < states.size(); ++i) {
        QVector3D currentPos(states[i][0], states[i][1], states[i][2]);
        double d = currentPos.distanceToPoint(targetPos);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }
    if (closest_idx < states.size()) {
        states.resize(closest_idx + 1);
    }

    // 2. QT 3D СЦЕНА

    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x202020)));
    QWidget *container = QWidget::createWindowContainer(view);

    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

    // Камера
    Qt3DRender::QCamera *camera = view->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 10000000.0f);

    // Определяем максимальный размер для масштаба
    float maxCoord = std::max({
        std::abs(params.targetX)*1.1, std::abs(params.targetY)*1.1, std::abs(params.targetZ)*1.1,
        std::abs(params.startX)*1.1, std::abs(params.startY)*1.1, std::abs(params.startZ)*1.1
    });
    if (maxCoord < 10.0f) maxCoord = 100.0f;

    // Считаем размер сцены по сетке (для красивой камеры)
    // Повторяем логику шага, чтобы узнать границу gridLimit
    float tStep = maxCoord / 6.0f;
    float mag = std::pow(10.0f, std::floor(std::log10(tStep)));
    float fr = tStep / mag;
    float st = (fr < 1.5f) ? 1.0f*mag : (fr < 3.0f ? 2.0f*mag : (fr < 7.0f ? 5.0f*mag : 10.0f*mag));
    float gridEnd = std::ceil(maxCoord / st) * st;

    // Позиция камеры
    camera->setPosition(QVector3D(-gridEnd*0.6f, gridEnd*0.8f, gridEnd*1.5f));
    camera->setViewCenter(QVector3D(gridEnd/3.0f, gridEnd/3.0f, gridEnd/3.0f));

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(rootEntity);
    camController->setCamera(camera);
    camController->setLinearSpeed(gridEnd);
    camController->setLookSpeed(180.0f);

    // Свет
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(2.0f);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTrans = new Qt3DCore::QTransform(lightEntity);
    lightTrans->setTranslation(QVector3D(-gridEnd, gridEnd, gridEnd));
    lightEntity->addComponent(lightTrans);

    Qt3DCore::QEntity *lightEntity2 = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light2 = new Qt3DRender::QPointLight(lightEntity2);
    light2->setColor("white");
    light2->setIntensity(1.5f);
    lightEntity2->addComponent(light2);
    Qt3DCore::QTransform *lightTrans2 = new Qt3DCore::QTransform(lightEntity2);
    lightTrans2->setTranslation(QVector3D(gridEnd, gridEnd, -gridEnd));
    lightEntity2->addComponent(lightTrans2);

    // РИСУЕМ ОБЪЕМНУЮ СЕТКУ
    createVolumetricGridAndLabels(rootEntity, maxCoord);

    // ПОДПИСИ ОСЕЙ (БУКВЫ)
    float labelPos = gridEnd + (gridEnd * 0.05f);
    float labelScale = gridEnd * 0.05f;
    createLabel(rootEntity, QVector3D(labelPos, 0, 0), "X", Qt::red, labelScale);
    createLabel(rootEntity, QVector3D(0, labelPos, 0), "Y", Qt::green, labelScale);
    createLabel(rootEntity, QVector3D(0, 0, labelPos), "Z", Qt::blue, labelScale);

    // ЖИРНЫЕ ОСИ
    float objectScale = gridEnd / 150.0f;
    createCylinderBetweenPoints(rootEntity, QVector3D(0,0,0), QVector3D(gridEnd, 0, 0), Qt::red, objectScale);
    createCylinderBetweenPoints(rootEntity, QVector3D(0,0,0), QVector3D(0, gridEnd, 0), Qt::green, objectScale);
    createCylinderBetweenPoints(rootEntity, QVector3D(0,0,0), QVector3D(0, 0, gridEnd), Qt::blue, objectScale);

    // ТРАЕКТОРИЯ
    if (states.size() > 1) {
        for (size_t i = 0; i < states.size() - 1; ++i) {
            QVector3D p1(states[i][0], states[i][1], states[i][2]);
            QVector3D p2(states[i+1][0], states[i+1][1], states[i+1][2]);
            createCylinderBetweenPoints(rootEntity, p1, p2, Qt::cyan, objectScale * 0.6f);
        }
    }

    QVector3D startPos(params.startX, params.startY, params.startZ);
    createSphere(rootEntity, startPos, Qt::green, objectScale * 3.0f);
    createSphere(rootEntity, targetPos, Qt::red, objectScale * 3.0f);

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

void FirstMod2::on_pushButton_clicked()
{
    hide();
    window = new FirstMod(this);
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->show();
}
