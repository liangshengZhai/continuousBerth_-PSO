#pragma once
#include <vector>
#include "../model/modelParam.h"

double rastrigin(const std::vector<double>& x);

class Particle {
public:
    ModelParams params;
    // const double xmin = -5.12, xmax = 5.12;
    const double w = 0.7;      // inertia weight
    const double c1 = 1.5;     // cognitive coefficient
    const double c2 = 1.5;     // social coefficient
    // double vmax = (xmax - xmin) * 0.2;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> best_position;
    double best_value;

    std::vector<double> code;       // 粒子编码
    double fitness;                 // 适应度值
    std::vector<double> p_best;     // 个体最优编码
    double p_best_fitness;     // 个体最优适应度

    Particle(ModelParams params);
    void Init();
    void evaluate_fitness();
    void init_velocity();
    void update_velocity(const std::vector<double>& global_best_position, double w, double c1, double c2);
    void update_position();
    void printParticle() const;
    void writeToCSV() const;

};

