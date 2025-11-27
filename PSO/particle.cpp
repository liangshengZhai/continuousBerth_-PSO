#include "particle.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <ctime>
#include <fstream>
#include "../model/modelParam.h"



const double V_MAX_POS = 40.0;  // 位置速度上限（米/迭代）
const double V_MAX_TIME = 2.0;  // 时间速度上限（小时/迭代）
const double V_MAX_SLOT = 3.0;  // 槽位速度上限（槽位/迭代）
const double LAMBDA_PEN = 1000000000.0; // 约束惩罚系数

double rastrigin(const std::vector<double>& x) {
    double A = 10.0;
    double sum = A * x.size();
    for (double xi : x) {
    // ===================== 目标函数定义区结束 =====================
        sum += xi * xi - A * std::cos(2 * M_PI * xi);
    }
    return sum;
}

Particle::Particle( ModelParams params){
    //构造函数对参数进行接收
    this->params = params;

    //粒子维度计算
    int dim = params.numShips *2+ params.numShips* params.numShipK *2;
    code.resize(dim,0);
    velocity.resize(dim, 0.0);
    p_best.resize(dim, 0.0);
    p_best_fitness = std::numeric_limits<double>::max();
    Init();
    init_velocity();
    evaluate_fitness();
    p_best = code;
    p_best_fitness = fitness;
}

void Particle::Init() {
    // 船舶停靠位置
    for(int i=0;i<params.numShips;i++){
        code[i] = (rand() / (double)RAND_MAX) * (params.Length - params.shipLength[i]);
    }

    //开始作业时间
    for(int i=0;i<params.numShips;i++){
        double start = params.arrivalTime[i];
        double end = params.planningHorizon;
        if (end < start) end = start; // 防止数据异常
        code[params.numShips + i] = start + (rand() / (double)RAND_MAX) * (end - start);
    }

    //槽位分配

    for(int i=0;i<params.numShips;i++){
        for(int k=0;k<params.numShipK;k++){
            int required_slots = params.requiredSlots[i][k];
            // 计算当前舱的编码起始索引：前2×numShips维是位置和时间，之后每舱占2维
            int base_idx = 2 * params.numShips + i * params.numShipK * 2 + k * 2;
            code[base_idx] =(rand() % params.numRows);
            code[base_idx + 1] = 1 + (rand() % (params.numSlotsPerRow - required_slots +1)); //确保有足够槽位
            
        }
    }
}   


void Particle::init_velocity(){

    // （1）位置速度
    for (int i = 0; i < params.numShips; ++i) {
        velocity[i] = (rand() / (double)RAND_MAX) * 2 * V_MAX_POS - V_MAX_POS;
    }

    // （2）时间速度（
    for (int i = 0; i < params.numShips; ++i) {
        velocity[3 + i] = (rand() / (double)RAND_MAX) * 2 * V_MAX_TIME - V_MAX_TIME;
    }

    // （3）槽位分配速度
    for(int i=0;i<params.numShips;i++){
        for(int k=0;k<params.numShipK;k++){
            int required_slots = params.requiredSlots[i][k];
            // 计算当前舱的编码起始索引：前2×numShips维是位置和时间，之后每舱占2维
            int base_idx = 2 * params.numShips + i * params.numShipK * 2 + k * 2;
            double v_row_min = -0.5;
            double v_row_max = 0.5;
            velocity[base_idx] = v_row_min + (rand() / (double)RAND_MAX) * (v_row_max - v_row_min);

            velocity[base_idx + 1] = -V_MAX_SLOT + (rand() / (double)RAND_MAX) * (V_MAX_SLOT * 2);
        }
    }

        
}
void Particle::evaluate_fitness(){
    vector<double> s = vector<double>(&code[0], &code[params.numShips]); // 船位置
    vector<double> start = vector<double>(&code[params.numShips], &code[2*params.numShips]); // 开始时间
    vector<double> slot_assign(&code[2*params.numShips], &code[2*params.numShips + params.numShips* params.numShipK *2]); // 槽位分配

    double total_time = 0.0;    // 总靠泊时间
    double save_cost = 0.0;    // 总存储成本
    double trans_cost = 0.0;    // 总转运成本
    double berth_cost = 0.0;    // 总靠泊成本
    double space_pen = 0.0;     // 泊位空间惩罚
    double time_pen = 0.0;      // 时间惩罚
    double slot_pen = 0.0;      // 槽位惩罚

    //靠泊时间计算（修正：每条船的靠泊时间为所有舱完成时间的最大值减到港时间）
    for(int i=0;i<params.numShips;i++){
        // double max_finish = 0.0;
        double finish_time =start[i]; //开始时间
        double single_time = 0.0;
        for(int k=0;k<params.numShipK;k++){
            double unload_speed = params.unloadingSpeed[i][k];
            double duration = params.cargoWeight[i]/(unload_speed*params.numShipK);
            finish_time  += duration;
        }
        double berth_time = finish_time - params.arrivalTime[i];
        total_time += berth_time;
    }

    //存储成本计算
    for(int i=0;i<params.numShips;i++){
        for(int k=0;k<params.numShipK;k++){
            int row = static_cast<int>(slot_assign[(i*params.numShipK + k)*2]);
            int slot_start = static_cast<int>(slot_assign[(i*params.numShipK + k)*2 +1]);
            for(int v=slot_start;v< slot_start + params.requiredSlots[i][k];v++){
                save_cost += params.storageCost[i][k][row];
            }
        }
    }

    //转运成本计算
    for(int i=0;i<params.numShips;i++){
    double ship_center = s[i] + params.shipLength[i]/2.0;
    for(int k=0;k<params.numShipK;k++){
        int row = static_cast<int>(slot_assign[(i*params.numShipK + k)*2]);
        int slot_start = static_cast<int>(slot_assign[(i*params.numShipK + k)*2 +1]);
        int req_slots = params.requiredSlots[i][k];
        if (req_slots > 0) {
            double cargo_per_cabin = params.cargoWeight[i] / params.numShipK;
            double slotDiv = 1.0 / req_slots;
            for(int v=slot_start;v< slot_start + req_slots;v++){
                double x_rv = v * params.width + params.width / 2.0;
                double y_rv = row * params.width + params.width / 2.0;
                double y0 = 0.0;
                double dx = fabs(ship_center - x_rv);
                double dy = fabs(y0 - y_rv);
                trans_cost += (dx + dy) * cargo_per_cabin * slotDiv;
            }
        }
    }
}

    for(int i=0;i<params.numShips;i++){
        //=====================
        berth_cost += s[i]; //假设靠泊成本为0

    }

    //约束检查
    //（1）空间约束
    // 3.1 泊位边界约束
    for (int i = 0; i < params.numShips; ++i) {
        double ship_len = params.shipLength[i];
        if (s[i] < 0.0) {
            space_pen += pow(-s[i], 2);
        }
        if (s[i] + ship_len > params.Length) {
            space_pen += pow(s[i] + ship_len - params.Length, 2);
        }
    }

    // 3.2 泊位不重叠约束
    for (int i = 0; i < params.numShips; ++i) {
        for (int j = i+1; j < params.numShips; ++j) {
            double len_i = params.shipLength[i];
            double len_j = params.shipLength[j];
            bool no_overlap = (s[i] + len_i + params.safe_distance <= s[j]) || 
                                (s[j] + len_j + params.safe_distance <= s[i]);
            if (!no_overlap) {
                double overlap = std::min(s[i]+len_i, s[j]+len_j) - std::max(s[i], s[j]);
                space_pen += pow(overlap + params.safe_distance, 2);
            }
        }
    }

    // 3.3 作业时间不早于到港
    for (int i = 0; i < params.numShips; ++i) {
        if (start[i] < params.arrivalTime[i]) {
            time_pen += pow(params.arrivalTime[i] - start[i], 2);
        }
    }

    // 3.4 槽位唯一性（每个槽最多分配一次）
    for (int r = 0; r < params.numRows; ++r) {
        for (int v = 0; v < params.numSlotsPerRow; ++v) {
            int count = 0;
            for (int i = 0; i < params.numShips; ++i) {
                for (int k = 0; k < params.numShipK; ++k) {
                    int row = static_cast<int>(slot_assign[(i*params.numShipK + k)*2]);
                    int slot_start = static_cast<int>(slot_assign[(i*params.numShipK + k)*2 +1]);
                    int required_slots = params.requiredSlots[i][k];
                    if (row == r && v >= slot_start && v < slot_start + required_slots) {
                        count++;
                    }
                }
            }
            if (count > 1) slot_pen += pow(count - 1, 2);
        }
    }

    // 计算最终适应度（加权目标+惩罚）
    double fitness_obj = params.alpha * trans_cost + params.gamma * save_cost + params.beta * total_time + 2000 * berth_cost;
    double total_penalty = LAMBDA_PEN * (space_pen + time_pen + slot_pen);
    fitness = fitness_obj + total_penalty;
    // std::cout << fitness << std::endl;
}

void Particle::update_velocity(const std::vector<double>& global_best_position, double w, double c1, double c) {
    for (size_t i = 0; i < velocity.size(); ++i) {
        double r1 = rand() / (double)RAND_MAX;
        double r2 = rand() / (double)RAND_MAX;

        velocity[i] = w * velocity[i]
                      + c1 * r1 * (p_best[i] - code[i])
                      + c2 * r2 * (global_best_position[i] - code[i]);

        // 限制速度在最大范围内
        if (i < params.numShips) { // 位置速度限制
            if (velocity[i] > V_MAX_POS) velocity[i] = V_MAX_POS;
            if (velocity[i] < -V_MAX_POS) velocity[i] = -V_MAX_POS;
        } else if (i < 2 * params.numShips) { // 时间速度限制
            if (velocity[i] > V_MAX_TIME) velocity[i] = V_MAX_TIME;
            if (velocity[i] < -V_MAX_TIME) velocity[i] = -V_MAX_TIME;
        } else { // 槽位速度限制
            if (velocity[i] > V_MAX_SLOT) velocity[i] = V_MAX_SLOT;
            if (velocity[i] < -V_MAX_SLOT) velocity[i] = -V_MAX_SLOT;
        }
    }
}

void Particle::update_position() {
    for (size_t i = 0; i < code.size(); ++i) {
        code[i] += velocity[i];

        // 位置边界检查
        if (i < params.numShips) {
            if (code[i] < 0.0) code[i] = 0.0;
            if (code[i] > params.Length - params.shipLength[i]) 
                code[i] = params.Length - params.shipLength[i];
        }
        // 时间边界检查
        else if (i < 2 * params.numShips) {
            if (code[i] < params.arrivalTime[i - params.numShips]) 
                code[i] = params.arrivalTime[i - params.numShips];
            if (code[i] > params.planningHorizon) 
                code[i] = params.planningHorizon;
        }
        // 槽位边界检查
        else {
            int idx = i - 2 * params.numShips;
            int ship_idx = idx / (params.numShipK * 2);
            int k = (idx / 2) % params.numShipK;
            int required_slots = params.requiredSlots[ship_idx][k];
            if (i % 2 == 0) { // 行号
                if (code[i] < 0) code[i] = 0;
                if (code[i] > params.numRows - 1) code[i] = params.numRows - 1;
            } else { // 槽位起始号
                if (code[i] < 0) code[i] = 0;
                if (code[i] > params.numSlotsPerRow - required_slots) 
                    code[i] = params.numSlotsPerRow - required_slots;
            }
        }
    }

    // 更新位置后重新计算适应度
    evaluate_fitness();
    //  std::cout << "Updated fitness: " << fitness << std::endl;

    // 更新个体最优
    if (fitness < p_best_fitness) {
        p_best = code;
        p_best_fitness = fitness;
    }
}


void Particle::printParticle() const {
    cout << "=== PSO最优解 ===" << endl;
    cout << "1. 船舶调度方案：" << endl;
    for (int i = 0; i < params.numShips; ++i) {
        double ship_len = params.shipLength[i];
        double arrive = params.arrivalTime[i];
        double berth_start = code[i];
        double start_time = code[params.numShips + i];
        double single_finish = start_time;
        // std::cout<<start_time<<std::endl;
        double max_finish = 0.0;
        for (int k = 0; k < params.numShipK; ++k) {
            double unload_speed = params.unloadingSpeed[i][k];
            double duration = params.cargoWeight[i]/(unload_speed*params.numShipK); //假设各舱同时作业，作业时间为最长舱的作业时间
            single_finish += duration;
            cout << "船" << i << ": 船首位置=" << fixed << setprecision(1) << berth_start << "米（船尾=" << berth_start+ship_len << "米）"
            << "，开始时间=" << start_time << "h（到达时间=" << arrive << "h）"
            << "，完成时间=" << fixed << setprecision(2) << single_finish << "h" << endl;
            start_time = single_finish;
        }
        
    }

    cout << "\n2. 槽位分配方案：" << endl;
    for (int i = 0; i < params.numShips; ++i) {
        for (int k = 0; k < params.numShipK; ++k) {
            int base_idx = 2 * params.numShips + i * params.numShipK * 2 + k * 2;
            int row = (int)code[base_idx];
            int start_slot = (int)code[base_idx + 1];
            int need_slots = params.requiredSlots[i][k];
            int end_slot = start_slot + need_slots - 1;
            cout << "船" << i << "-舱" << k << ": 存储行" << row << "，槽位" << start_slot << "-" << end_slot;
            // 计算该货舱的转运成本
            double ship_center = code[i] + params.shipLength[i]/2.0;
            double transfer_cost = 0.0;
            for (int v = start_slot; v < start_slot + need_slots; ++v) {
                double x_rv = v * params.width + params.width / 2.0;
                double y_rv = row * params.width + params.width / 2.0;
                double y0 = 0.0;
                double dx = fabs(ship_center - x_rv);
                double dy = fabs(y0 - y_rv);
                double slotDiv = 1.0;
                if (need_slots > 0 && params.numShipK > 0) {
                    slotDiv = 1.0 / (need_slots * params.numShipK);
                    transfer_cost += (dx + dy) * params.cargoWeight[i] * slotDiv;
                }
            }
            cout << "，转运成本: " << fixed << setprecision(2) << transfer_cost << endl;
        }
    }

    // 重新计算未加惩罚项的目标值
    // 1. 船位置
    std::vector<double> s(&code[0], &code[params.numShips]);
    // 2. 开始时间
    std::vector<double> start(&code[params.numShips], &code[2*params.numShips]);
    // 3. 槽位分配
    std::vector<double> slot_assign(&code[2*params.numShips], &code[2*params.numShips + params.numShips* params.numShipK *2]);

    double total_time = 0.0;
    double save_cost = 0.0;
    double trans_cost = 0.0;
    double berth_cost = 0.0;

    //靠泊时间计算（修正：每条船的靠泊时间为所有舱完成时间的最大值减到港时间）
    for(int i=0;i<params.numShips;i++){
        // double max_finish = 0.0;
        double finish_time =start[i]; //开始时间
        double single_time = 0.0;
        for(int k=0;k<params.numShipK;k++){
            double unload_speed = params.unloadingSpeed[i][k];
            double duration = params.cargoWeight[i]/(unload_speed*params.numShipK);
            finish_time  += duration;
        }
        double berth_time = finish_time - params.arrivalTime[i];
        total_time += berth_time;
    }

    //存储成本计算
    for(int i=0;i<params.numShips;i++){
        for(int k=0;k<params.numShipK;k++){
            int row = static_cast<int>(slot_assign[(i*params.numShipK + k)*2]);
            int slot_start = static_cast<int>(slot_assign[(i*params.numShipK + k)*2 +1]);
            for(int v=slot_start;v< slot_start + params.requiredSlots[i][k];v++){
                save_cost += params.storageCost[i][k][row];
            }
        }
    }

    //转运成本计算
    for(int i=0;i<params.numShips;i++){
    double ship_center = s[i] + params.shipLength[i]/2.0;
    for(int k=0;k<params.numShipK;k++){
        int row = static_cast<int>(slot_assign[(i*params.numShipK + k)*2]);
        int slot_start = static_cast<int>(slot_assign[(i*params.numShipK + k)*2 +1]);
        int req_slots = params.requiredSlots[i][k];
        if (req_slots > 0) {
            double cargo_per_cabin = params.cargoWeight[i] / params.numShipK;
            double slotDiv = 1.0 / req_slots;
            for(int v=slot_start;v< slot_start + req_slots;v++){
                double x_rv = v * params.width + params.width / 2.0;
                double y_rv = row * params.width + params.width / 2.0;
                double y0 = 0.0;
                double dx = fabs(ship_center - x_rv);
                double dy = fabs(y0 - y_rv);
                trans_cost += (dx + dy) * cargo_per_cabin * slotDiv;
            }
        }
    }
}

    for(int i=0;i<params.numShips;i++){
        berth_cost += s[i]; //假设靠泊成本为0
    }

    std::cout<<"转运成本=" << trans_cost << 
    ", 存储成本=" << params.gamma *save_cost<< 
    ", 总完成时间=" << params.beta *total_time << 
    ", 靠泊成本=" << berth_cost << std::endl;
    double fitness_obj = params.alpha * trans_cost + params.gamma * save_cost + params.beta * total_time + 2000 * berth_cost;
    cout << "未加惩罚项的目标值：" << fixed << setprecision(2) << fitness_obj << endl;
    cout << "--------------------------" << endl;
}
