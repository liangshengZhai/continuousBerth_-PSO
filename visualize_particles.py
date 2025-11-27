import matplotlib.pyplot as plt
import numpy as np
import os

def parse_particles_log(filename, dim=30):
    """
    解析粒子分布日志，返回一个字典，key为迭代次数，value为粒子位置的numpy数组
    """
    result = {}
    with open(filename, 'r') as f:
        lines = f.readlines()
    iter_num = None
    particles = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith('# Iteration'):
            if iter_num is not None and particles:
                result[iter_num] = np.array(particles)
            iter_num = int(line.split()[2])
            particles = []
        else:
            pos = [float(x) for x in line.split(',') if x]
            if len(pos) == dim:
                particles.append(pos)
    if iter_num is not None and particles:
        result[iter_num] = np.array(particles)
    return result

def plot_particles(particles_dict, dims=(0,1), outdir='particle_plots'):
    """
    dims: tuple, 只画前两个维度
    """
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    for iter_num, positions in particles_dict.items():
        plt.figure(figsize=(6,6))
        plt.scatter(positions[:,dims[0]], positions[:,dims[1]], c='blue', alpha=0.6, label='Particles')
        plt.xlabel(f'Dimension {dims[0]+1}')
        plt.ylabel(f'Dimension {dims[1]+1}')
        plt.title(f'Particle Distribution at Iteration {iter_num}')
        plt.xlim(-5.2, 5.2)
        plt.ylim(-5.2, 5.2)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(f'{outdir}/particles_iter_{iter_num}.png')
        plt.close()
    print(f'All particle distribution plots saved in "{outdir}" folder.')

if __name__ == '__main__':
    # 读取并绘制粒子分布
    particles_log = 'particles.log'
    particles_dict = parse_particles_log(particles_log, dim=30)
    plot_particles(particles_dict, dims=(0,1))
