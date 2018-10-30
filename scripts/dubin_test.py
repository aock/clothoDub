import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import dubins

matplotlib.rcParams['figure.figsize'] = 12, 9

def expand_axis(ax, scale, name):
    getter = getattr(ax, 'get_' + name)
    setter = getattr(ax, 'set_' + name)
    a, b = getter()
    mid = (a+b)/2.0
    diff = (b - mid)
    setter(mid - scale*diff, mid + scale*diff)

def expand_plot(ax, scale = 1.1):
    expand_axis(ax, scale, 'xlim')
    expand_axis(ax, scale, 'ylim')

def plot_dubins_path(qs):
    # path = dubins.shortest_path(q0, q1, r)
    # qs, _ = path.sample_many(step_size)
    # # qs, _ = dubins.path_sample(q0, q1, r, step_size)
    # qs = np.array(qs)
    xs = qs[:, 0]
    ys = qs[:, 1]
    us = xs + np.cos(qs[:, 2])
    vs = ys + np.sin(qs[:, 2])
    plt.plot(xs, ys, 'b-')
    plt.plot(xs, ys, 'r.')
    for i in xrange(qs.shape[0]):
        plt.plot([xs[i], us[i]], [ys[i], vs[i]],'r-')
    ax = plt.gca()
    expand_plot(ax)
    ax.set_aspect('equal')

if __name__ == '__main__':

    print('Dubin test path')

    # src pose
    x0 = 0.0
    y0 = 0.0
    theta0 = np.pi / 2.0

    # target pose
    x1 = 3.0
    y1 = 2.0
    theta1 = np.pi / 2.0

    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    turning_radius = 5.0
    step_size = 6.0

    path = dubins.shortest_path(q0, q1, turning_radius)
    configurations, _ = path.sample_many(step_size)


    qs = np.array(configurations)
    print(qs)

    # calculate angle differences
    # print(qs)
    delta_qs = np.diff(qs, axis=0)
    # print(delta_qs)

    plot_dubins_path(qs)

    plt.show()

    # for pose in configurations:
    #     print(pose)
