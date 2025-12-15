#ifndef FLIGHT_MODEL_H
#define FLIGHT_MODEL_H

#include <vector>
#include <cmath>
#include <iostream>

typedef std::vector<double> state_type;

// Структура для передачи параметров запуска
struct MissionParams {
    double startX, startY, startZ;
    double targetX, targetY, targetZ;
    double targetV;
};

struct MissionParamsMode2 {
    double startX, startY, startZ;
    double p1X, p1Y, p1Z; // Первая промежуточная точка
    double p2X, p2Y, p2Z; // Конечная точка
    double targetV;
};

class AircraftModel {
    // Константы ЛА
    double g = 9.81;
    double T_nxa = 0.01;   // Постоянная времени
    double T_nya = 0.01;   // Постоянная времени
    double xi_nya = 0.9;  // Демпфирование
    double T_nza = 0.01;   // Постоянная времени
    double T_gamma = 0.01; // Постоянная времени

    // Параметры ПИД-регуляторов (коэффициенты подобраны эмпирически)
    // Для скорости
    double Kp_v = 5;

    // Для высоты (простой ПД регулятор)
    double Kp_h = 1;
    double Kd_h = 1;

    // Для курса (Боковое движение)
    double Kp_psi = 1;
    double Kp_gamma = 1;

    MissionParams mission;

public:
    // Вектор управлений [u_nxa, u_nya, u_nza, u_gamma]
    // Мы их вычисляем внутри модели на каждом шаге
    std::vector<double> u = {0.0, 0.0, 0.0, 0.0};

    AircraftModel(MissionParams params) : mission(params) {}

    void operator() (const state_type &x, state_type &dxdt, const double /* t */) {
        // Распаковка вектора состояния

        double X = x[0];
        double Y = x[1];
        double Z = x[2];
        double V = x[3];
        double theta = x[4];
        double psi = x[5];
        double n_xa = x[6];
        double n_ya = x[7];
        double n_ya_dot = x[8];
        double n_za = x[9];
        double n_za_dot = x[10];
        double gamma = x[11];
        double gamma_dot = x[12];

        // Защита от деления на ноль и малых скоростей
        if (std::abs(V) < 1.0) V = 1.0;
        double c_th = std::cos(theta);


        // Рассчитываем необходимые управляющие сигналы U

        // 1. Управление скоростью
        // U_nxa = (V_zad - V_tek) * K
        double err_V = mission.targetV - V;
        double u_nxa = Kp_v * err_V + std::sin(theta);

        // Ограничение тяги
        if (u_nxa > 11.0) u_nxa = 11.0;
        if (u_nxa < -11.0) u_nxa = -11.0;


        // 2. Управление высотой -> Выход U_nya
        double err_H = mission.targetY - Y;

        // Лимит вертикальной скорости теперь зависит от полной скорости.
        // Мы разрешаем дрону использовать почти всю свою скорость для набора высоты (до 98%).
        double max_climb_rate = mission.targetV * 0.98;
        double desired_Vy = std::max(-max_climb_rate, std::min(err_H, max_climb_rate));
        double u_nya = Kd_h * (Kp_h * desired_Vy - (V * std::sin(theta))) + 1 + std::cos(theta);

        // Лимиты перегрузки
        u_nya = std::max(-11.0, std::min(u_nya, 11.0));


        // 3. Управление курсом -> Выход U_gamma
        // Рассчитываем целевой курс на точку (Psi_zad)
        double dx = mission.targetX - X;
        double dz = mission.targetZ - Z;
        double target_psi = std::atan2(-dz, dx);

        double err_psi = target_psi - psi;

        // ПИД курса выдает заданный крен (gamma_zad)
        double gamma_zad = Kp_psi * err_psi;

        // Ограничение крена
        if (gamma_zad > 1.5) gamma_zad = 1.5;
        if (gamma_zad < -1.5) gamma_zad = -1.5;

        // Регулятор крена: U_gamma = (gamma_zad - gamma_tek)
        double u_gamma = Kp_gamma * (gamma_zad - gamma);

        // N_za всегда 0 по условию
        double u_nza = 0.0;

        // УРАВНЕНИЯ ДВИЖЕНИЯ

        // Вспомогательные
        double s_th = std::sin(theta);
        double c_ps = std::cos(psi);
        double s_ps = std::sin(psi);
        double c_gm = std::cos(gamma);
        double s_gm = std::sin(gamma);

        if (std::abs(c_th) < 1e-4) c_th = 1e-4; // Защита

        dxdt[0] = V * c_ps * c_th;             // dot_X
        dxdt[1] = V * s_th;                    // dot_Y
        dxdt[2] = -V * s_ps * c_th;            // dot_Z

        dxdt[3] = g * (n_xa - s_th);           // dot_V

        dxdt[4] = (g / V) * (n_ya * c_gm - n_za * s_gm - c_th); // dot_theta
        dxdt[5] = -(g / (V * c_th)) * (n_ya * s_gm + n_za * c_gm); // dot_psi

        dxdt[6] = (u_nxa - n_xa) / T_nxa;      // dot_n_xa
        dxdt[7] = n_ya_dot;                    // dot_n_ya
        dxdt[8] = (u_nya - 2 * xi_nya * T_nya * n_ya_dot - n_ya) / (T_nya * T_nya); // dot_n_ya_dot
        dxdt[9] = n_za_dot;                        // dot_n_za
        dxdt[10] = (u_nza - n_za_dot) / T_nza;     // n_za_dot
        dxdt[11] = gamma_dot;                  // dot_gamma
        dxdt[12] = (u_gamma - gamma_dot) / T_gamma; // dot_gamma_dot
    }
};

#endif // FLIGHT_MODEL_H
