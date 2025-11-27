import matplotlib.pyplot as plt
import numpy as np

def read_log(filename):
    iterations = []
    values = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('Iteration'):
                parts = line.strip().split(',')
                iter_num = int(parts[0].split()[1])
                best_val = float(parts[1].split(':')[1])
                iterations.append(iter_num)
                values.append(best_val)
    return iterations, values

if __name__ == '__main__':
    # 假设C++输出日志到 result.log
    log_file = 'result.log'
    iterations, values = read_log(log_file)
    plt.figure(figsize=(8,5))
    plt.plot(iterations, values, marker='o')
    plt.xlabel('Iteration')
    plt.ylabel('Best Value')
    plt.title('PSO Optimization Progress on Rastrigin Function')
    plt.grid(True)
    plt.tight_layout()
    plt.show()
