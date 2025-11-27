#include <vector>
#include <iostream>
#include <iomanip>
#include "particle.h"

// 算法参数
const int POP_SIZE = 4000;        // 种群规模（粒子数量）
const int MAX_ITER = 2000;       // 最大迭代次数

class PSO {
private:
    vector<Particle> swarm;       // 粒子群
    vector<double> g_best;        // 全局最优编码
    double g_best_fitness;        // 全局最优适应度
    ModelParams params;        // 模型参数

public:
    PSO(ModelParams params) : params(params) {
        // 正确初始化粒子群，确保每个粒子独立随机
        swarm.clear();
        for (int i = 0; i < POP_SIZE; ++i) {
            swarm.emplace_back(params);
        }
        // 初始化全局最优（从初始种群中找最优）
        g_best = swarm[0].code;
        g_best_fitness = swarm[0].fitness;
        std::cout << "Initial global best fitness: " << g_best_fitness << std::endl;
        for (auto& particle : swarm) {
            if (particle.fitness < g_best_fitness) {
                g_best = particle.code;
                g_best_fitness = particle.fitness;
            }
        }
    }

    // 执行PSO迭代
    void run() {
        cout << "=== PSO算法开始迭代 ===" << endl;
        cout << "种群规模：" << POP_SIZE << "，最大迭代次数：" << MAX_ITER << endl;
        cout << "初始全局最优适应度：" << fixed << setprecision(2) << g_best_fitness << endl;
        cout << "--------------------------" << endl;

        for (int iter = 0; iter < MAX_ITER; ++iter) {
            // 遍历每个粒子，更新速度、位置
            for (auto& particle : swarm) {
                particle.update_velocity(g_best); // 按全局最优更新速度
                particle.update_position();       // 更新位置
            }

            // 更新全局最优
            for (auto& particle : swarm) {
                if (particle.fitness < g_best_fitness) {
                    g_best = particle.code;
                    g_best_fitness = particle.fitness;
                }
            }

            // 每10次迭代输出进度
            if ((iter + 1) % 10 == 0) {
                cout << "迭代次数：" << iter + 1 << "，当前全局最优适应度：" << fixed << setprecision(2) << g_best_fitness << endl;
            }
        }

        cout << "=== PSO算法迭代结束 ===" << endl;
        // 找到全局最优对应的粒子并打印结果
        for (auto& particle : swarm) {
            if (particle.fitness == g_best_fitness) {
                particle.printParticle();
                break;
            }
        }
    }
};