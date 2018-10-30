import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from scipy.special import fresnel
import scipy.integrate as integrate
import dubins

def clothoid_to_point(p1, p2, samples=1000):
    
    Rc = p2[1]
    # regression
    Rc2 = Rc**2.0 / 1.21646743

    s0 = 1.0

    # clothoid algorithm

    a  = 1.0 / np.sqrt( 2.0 * Rc2 * s0 )

    # fresnel

    L = 1.0
    t = np.linspace(0.0,L,samples)
    ssa, csa = fresnel(t)

    x_corr_fac = 1.12389500

    x = 1.0/a * ssa * p2[0] / x_corr_fac
    y = 1.0/a * csa

    return x,y

class Clothoid:

    def __init__(self, A=1):
        self.A = A

    def tangent_by_length(self, L):

        A = self.A

        tx = np.cos(L**2 / (A**2 * 2.0) ) 
        ty = np.sin(L**2 / (A**2 * 2.0) )

        return tx, ty

    def length_by_tangent(self, tx, ty, n=int(0) ):

        # X ROOT
        A = self.A

        Lx1 = - 2.0*np.arccos(tx)
        
        if Lx1 < 0:
            # shift to first positiv root
            num_l_under_zero = np.ceil( -Lx1 / (4.0*np.pi) )
            Lx1 += 4.0 * np.pi * num_l_under_zero

        # shift to next roots
        Lx1 += 4.0 * np.pi * n
        Lx1 = A * np.sqrt(Lx1)

        
        Lx2 = np.arccos(tx)
        if Lx2 < 0:
            # shift to first positiv root
            num_l_under_zero = np.ceil(- Lx2 / (2.0 * np.pi) )
            Lx2 += 2.0 * np.pi * num_l_under_zero

        Lx2 += 2.0 * np.pi * n
        Lx2 = A * np.sqrt(2.0) * np.sqrt(Lx2)

        # Y ROOT

        Ly1 = np.arcsin(ty)
        if Ly1 < 0:
            num_l_under_zero = np.ceil(- Ly1 / (2.0 * np.pi) )
            Ly1 += 2.0 * np.pi * num_l_under_zero

        Ly1 += 2.0 * np.pi * n
        Ly1 = A * np.sqrt(2.0) * np.sqrt(Ly1)

        Ly2 = -np.arcsin(ty)+np.pi
        if Ly2 < 0:
            # shift to first positive root
            num_l_under_zero = np.ceil(- Ly2 / (2.0 * np.pi) )
            Ly2 += 2.0 * np.pi * num_l_under_zero
        Ly2 += 2.0 * np.pi * n
        Ly2 = A * np.sqrt(2) * np.sqrt(Ly2)

        print("Roots:")
        print(Lx1)
        print(Lx2)
        print(Ly1)
        print(Ly2)

        if ty < 0 and tx > 0:
            return Ly1
        elif ty < 0:
            return Ly2
        else:
            return Lx2

    def point_by_length(self, L):
        # L -> lenge von start bis ziel ueber die clothoid bahn
        # R -> kruemmungsradius

        A = self.A

        R = L * 1.0/A**2.0

        # WIKIPEDIA: https://de.wikipedia.org/wiki/Klothoide
        l = L / (A * np.sqrt(np.pi))
        
        ssa, csa = fresnel(l)

        px = A * np.sqrt(np.pi) * csa
        py = A * np.sqrt(np.pi) * ssa

        return px, py

    def tangent_by_angle(self, angle):
        tx = np.cos(angle)
        ty = np.sin(angle)
        return tx,ty

    def transform_to_local(self, local_pose, pose):
        out_pose = np.array([0.0,0.0,0.0])

        # translate
        out_pose[0] = pose[0] - local_pose[0]
        out_pose[1] = pose[1] - local_pose[1]

        # rotate

        phi = -local_pose[2]
        x = out_pose[0] * np.cos(phi) - out_pose[1] * np.sin(phi)
        y = out_pose[1] * np.cos(phi) + out_pose[0] * np.sin(phi)
        out_pose[0] = x
        out_pose[1] = y

        print(x)
        print(y)

        # pose

        # atan2(from.x*to.y-from.y*to.x, from.x*to.x+from.y*to.y)
        out_pose[2] = pose[2] - local_pose[2]
        
        if out_pose[2] < -np.pi:
            out_pose[2] += 2.0 * np.pi
        elif out_pose[2] > np.pi:
            out_pose[2] -= 2.0 * np.pi

        return out_pose

    def transform_to_global(self, global_pose, pose):
        out_pose = np.array([0.0,0.0,0.0])

        # rotate
        out_pose[2] = global_pose[2] + pose[2]

        if out_pose[2] < -np.pi:
            out_pose[2] += 2.0 * np.pi
        elif out_pose[2] > np.pi:
            out_pose[2] -= 2.0 * np.pi

        # translate

        out_pose[0] = pose[0] + global_pose[0]
        out_pose[1] = pose[1] + global_pose[1]

        return out_pose


    def path_by_poses(self, p0, p1, p2, samples=1000):

        # determine state
        # p0 left side of clothoid
        # p1 origin of clothoid (tansform other points to this frame) (TODO: better orientation)
        # p2 right side of clothoid


        origin = p1
        # origin[2] = (p2[2] - p0[2]) / 2.0 + p0[2]

        p0_local = self.transform_to_local(origin, p0)

        p2_local = self.transform_to_local(origin, p2)

        t0x, t0y = self.tangent_by_angle(p0_local[2])
        t2x, t2y = self.tangent_by_angle(p2_local[2])


        L1 = -self.length_by_tangent(t0x, t0y)
        L2 = self.length_by_tangent(t2x, t2y)

        
        Ls = np.linspace(L1, L2, samples)

        px, py = self.point_by_length(Ls)



        # normalize
        # px *= p2[0] / px[-1]
        # py *= p2[1] / py[-1]


        # plt.subplot(221)

        # xs = np.array([p0[0], p1[0], p2[0]])
        # ys = np.array([p0[1], p1[1], p2[1]])

        # plt.xlim((-5,5))
        # plt.ylim((-5,5))
        # plt.plot(xs[0], ys[0], 'o')
        # plt.plot(xs, ys, '-x')

        # plt.subplot(222)
        xs = np.array([p0_local[0], 0, p2_local[0]])
        ys = np.array([p0_local[1], 0, p2_local[1]])
        
        plt.xlim((-5,5))
        plt.ylim((-5,5))
        plt.plot(xs[0], ys[0], 'o')
        plt.plot(t2x, t2y, 'o')
        plt.plot(xs, ys, '-x')
        
        plt.plot(px[-1], py[-1],'x')


        plt.plot(px, py)
        plt.show()

        
        
        return px, py


    def smooth_dubin_path(self, path):

        # example first three points
        q0 = path[0]
        q1 = path[2]
        q2 = path[4]

        t0x, t0y = self.tangent_by_angle(q0[2])
        t1x, t1y = self.tangent_by_angle(q1[2])
        t2x, t2y = self.tangent_by_angle(q2[2])

        # transform to q1 origin

        pathx, pathy = self.path_by_poses(q0, q1, q2)

        


        
def test1():
    print('Clothoid test')

    Ls = np.linspace(0.0, 10.0, 1000)
    A = 3.0

    clo = Clothoid(A=A)
    

    asympx = np.array([ A*np.sqrt(np.pi) / 2.0, - A * np.sqrt(np.pi) / 2.0])
    asympy = np.array([ A*np.sqrt(np.pi) / 2.0, - A * np.sqrt(np.pi) / 2.0])

    print(asympx)

    px, py = clo.point_by_length(Ls)

    tx, ty = clo.tangent_by_length(Ls)
    

    ex_orient_x = 0
    ex_orient_y = 1

    real_length = Ls[-1]

    L = clo.length_by_tangent(ex_orient_x, ex_orient_y)
    p_tangent_x, p_tangent_y = clo.point_by_length(L)

    print('%f = %f' % (real_length, L) )


    p1 = np.array([0.0, 0.0, 1.57079633])
    p2 = np.array([1.0, 1.0, np.pi])

    pathx, pathy = clo.path_by_poses(p1,p2)

    plt.plot(pathx, pathy)
    plt.show()

def test2():
    print('Clothoid test')

    A = 3.0
    clo = Clothoid(A=A)

    # source pose
    x0 = 0.0
    y0 = 0.0
    theta0 = np.pi / 2.0

    # target pose
    x1 = 3.0
    y1 = 2.0
    theta1 = - np.pi / 2.0

    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    turning_radius = 3.0
    step_size = 2.0

    path = dubins.shortest_path(q0, q1, turning_radius)
    configurations, _ = path.sample_many(step_size)

    qs = np.array(configurations)
    print(qs)

    qs = clo.smooth_dubin_path(qs)



if __name__ == '__main__':
    test2()

    # print(p)

    # https://en.wikipedia.org/wiki/Euler_spiral
    # Radius of curvature
    # R = 3.0 
    # Radius of Circular curve at the end of the spiral

    # p1 = np.array([0.0,0.0])
    # p2 = np.array([10.0,20.0])

    # x,y = clothoid_to_point(p1,p2)
    
    # plt.plot(px, py)
    # plt.plot(asympx, asympy, 'x')#
    # plt.plot(tx, ty)
    # plt.plot(p_tangent_x, p_tangent_y, 'o')
    # plt.show()
