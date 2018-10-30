import numpy as np
from g1fitting import build_clothoid, points_on_clothoid
import matplotlib
import matplotlib.pyplot as plt
import dubins
# from ipdb import set_trace

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

    # DUBIN PARAMETER
    turning_radius = 5.0
    step_size = 5.0

    # CLOTHOID PARAMETER

    # SRC POSE
    x0 = 0.0
    y0 = 0.0
    theta0 = np.pi / 2.0

    # TARGET POSE
    x1 = 0.0
    y1 = 10.0
    theta1 = 0.0

    # DUBIN

    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    
    path = dubins.shortest_path(q0, q1, turning_radius)
    configurations, _ = path.sample_many(step_size)

    qs = np.array(configurations)

    print(qs[-1])
    

    # G1FITTING

    pathX = np.array([])
    pathY = np.array([])

    print(qs.shape)

    for i in range(1,qs.shape[0], 1):
        x0 = qs[i-1, 0]
        y0 = qs[i-1, 1]
        theta0 = qs[i-1, 2]
        x1 = qs[i, 0]
        y1 = qs[i, 1]
        theta1 = qs[i, 2]

        (k, dk, L, res1) = build_clothoid(x0, y0, theta0, x1, y1, theta1)
        (X, Y, res2) = points_on_clothoid(x0, y0, theta0, k, dk, L, 100)
        pathX = np.concatenate( (pathX, X) )
        pathY = np.concatenate( (pathY, Y) )

    # VISUALIZE

    plot_dubins_path(qs)
    plt.plot(pathX,pathY,'g-')
    plt.show()