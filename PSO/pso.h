#include <vector>
#include <iostream>
#include <iomanip>
#include "particle.h"

// 算法参数
const int POP_SIZE = 8000;        // 种群规模（粒子数量）
const int MAX_ITER = 200;       // 最大迭代次数

class PSO {
private:
    vector<Particle> swarm;       // 粒子群
    vector<double> g_best;        // 全局最优编码
    double g_best_fitness;        // 全局最优适应度
    ModelParams params;        // 模型参数
    int g_best_index;
    Particle* g_best_particle;    // 保存最优粒子指针

public:
    PSO(ModelParams params) : params(params) {
        // 正确初始化粒子群，确保每个粒子独立随机
        swarm.clear();
        for (int i = 0; i < POP_SIZE; ++i) {
            swarm.emplace_back(params);
        }
        // 初始化全局最优（从初始种群中找最优）
        g_best_index = 0;
        g_best = swarm[0].code;
        g_best_fitness = swarm[0].fitness;
        g_best_particle = &swarm[0];
        std::cout << "Initial global best fitness: " << g_best_fitness << std::endl;
        for (int i = 0; i < swarm.size(); ++i) {
            if (swarm[i].fitness < g_best_fitness) {
                g_best = swarm[i].code;
                g_best_fitness = swarm[i].fitness;
                g_best_index = i;
                g_best_particle = &swarm[i];
            }
        }
    }

    // 核心：计算每个粒子与其他粒子的平均距离d_i
    vector<double> calculateMeanDistances() {
        vector<double> d_i(swarm.size(), 0.0);
        for (int i = 0; i < swarm.size(); ++i) {
            double total_dist = 0.0;
            for (int j = 0; j < swarm.size(); ++j) {
                if (i == j) continue;
                // 计算粒子i与粒子j的欧氏距离（24维编码）
                double dist = 0.0;
                for (int d = 0; d < 24; ++d) {
                    dist += pow(swarm[i].code[d] - swarm[j].code[d], 2);
                }
                total_dist += sqrt(dist);
            }
            d_i[i] = total_dist / (swarm.size() - 1); // 平均距离
        }
        return d_i;
    }



    // 执行PSO迭代
    void run() {
        cout << "=== PSO算法开始迭代 ===" << endl;
        cout << "种群规模：" << POP_SIZE << "，最大迭代次数：" << MAX_ITER << endl;
        cout << "初始全局最优适应度：" << fixed << setprecision(2) << g_best_fitness << endl;
        cout << "--------------------------" << endl;
        vector<double> d_i = calculateMeanDistances();
        double d_max = *max_element(d_i.begin(), d_i.end());
        double d_min = *min_element(d_i.begin(), d_i.end());
        double d_g = d_i[g_best_index]; // 全局最优粒子的平均距离

        // 计算进化因子f（避免分母为0）
        double f = 0.0;
        if (d_max - d_min > 1e-6) {
            f = (d_g - d_min) / (d_max - d_min);
        }
        f = clamp(f, 0.0, 1.0); // 确保f在[0,1]范围内
        double c1 , c2 =0;
        string state;

        if (f <= 0.25) {          // 收敛状态（Convergence）
            state = "Convergence";
            c1 *= 1.1;  // 增大认知系数
            c2 *= 1.1;  // 增大社会系数
        } else if (f <= 0.5) {     // 开发状态（Exploitation）
            state = "Exploitation";
            c1 *= 1.1;  // 增大认知系数
            c2 *= 0.9;  // 减小社会系数
        } else if (f <= 0.75) {    // 探索状态（Exploration）
            state = "Exploration";
            c1 *= 0.9;  // 减小认知系数
            c2 *= 1.1;  // 增大社会系数
        } else {                   // 跳出状态（Jumping out）
            state = "Jumping out";
            c1 *= 0.9;  // 减小认知系数
            c2 *= 1.1;  // 增大社会系数
        }

        // 限制c1、c2的范围（避免过大或过小）
        c1 = clamp(c1, 0.5, 2.5);
        c2 = clamp(c2, 0.5, 2.5);

        // -------------------------- 步骤3：计算自适应惯性权重ω --------------------------
        double w = 1.0 / (1 + 1.5 * exp(-2.6 * f)); // 公式：ω(f) = 1/(1+1.5e^(-2.6f))

        for (int iter = 0; iter < MAX_ITER; ++iter) {
            // 遍历每个粒子，更新速度、位置
            for (auto& particle : swarm) {
                particle.update_velocity(g_best, w, c1, c2); // 按全局最优更新速度
                particle.update_position();       // 更新位置
            }

            // 更新全局最优
            for (int i = 0; i < swarm.size(); ++i) {
                if (swarm[i].fitness < g_best_fitness) {
                    g_best = swarm[i].code;
                    g_best_fitness = swarm[i].fitness;
                    g_best_index = i;
                    g_best_particle = &swarm[i];
                }
            }

            // 每10次迭代输出进度
            if ((iter + 1) % 10 == 0) {
                cout << "迭代次数：" << iter + 1 << "，当前全局最优适应度：" << fixed << setprecision(2) << g_best_fitness << endl;
            }
        }

        cout << "=== PSO算法迭代结束 ===" << endl;
        // 直接输出保存的最优粒子
        if (g_best_particle) {
            g_best_particle->printParticle();
            g_best_particle->writeToCSV();
        }
    }
     // 辅助函数：数值夹紧
    double clamp(double val, double min_val, double max_val) {
        return max(min_val, min(val, max_val));
    }
};