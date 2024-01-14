# Author: ivantregear
# Date Created: 22/02/2023
# Description:

# When I wrote this code only god I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)


def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


class SolidBody:
    def __init__(self, points):
        self.loads = []
        self.unknowns = []
        self.indexes = []
        self.n_loads = 0
        self.n_unknowns = 0
        self.points = points

    def add_load(self, load_state):
        self.loads += [load_state]
        self.n_loads += 1

    def add_unknown(self, load_state, index):
        self.unknowns += [load_state]
        self.indexes += [index]
        self.n_unknowns += 1


class CoordinateSpace:
    def __init__(self):
        self.n_bodies = 0
        self.bodies = []
        self.unknown_labels = []
        self.n_unknowns = 0
        self.mat_a = None
        self.mat_b = None
        self.dofs = None
        self.n_dofs = 0
        self.solution = None

    def add_body(self, points):
        self.n_bodies += 1
        self.bodies += [SolidBody(points)]

    def solve(self):

        def construct_mat_b():
            for i in range(self.n_bodies):
                mat_b_temp = np.zeros(6)
                for j in range(self.bodies[i].n_loads):
                    mat_b_temp = np.subtract(mat_b_temp, self.bodies[i].loads[j][1:, :].flatten())
                    mat_b_temp[3:] = np.subtract(mat_b_temp[3:], np.cross(self.bodies[i].loads[j][0, :], self.bodies[i].loads[j][1, :]))

                mat_b_temp = np.delete(mat_b_temp, ~self.dofs)
                self.mat_b[i * self.n_dofs:(i + 1) * self.n_dofs] = mat_b_temp

        def construct_mat_a():

            mat_a_full = np.zeros([6 * self.n_bodies, self.n_unknowns])

            for i in range(self.n_bodies):
                for j in range(self.bodies[i].n_unknowns):
                    mat_a_full[i * 6:(i * 6) + 6, self.bodies[i].indexes[j]] += self.bodies[i].unknowns[j][1:, :].flatten()
                    moment_coefficients = -np.cross(self.bodies[i].unknowns[j][1, :], self.bodies[i].unknowns[j][0, :])

                    mat_a_full[(i * 6) + 3:(i * 6) + 6, self.bodies[i].indexes[j]] += moment_coefficients.T

            self.mat_a = np.delete(mat_a_full, np.tile(~self.dofs, self.n_bodies), axis=0)

        self.n_unknowns = len(self.unknown_labels)
        self.n_dofs = np.count_nonzero(self.dofs)

        n_eqs = self.n_dofs * self.n_bodies

        self.mat_a = np.zeros([n_eqs, n_eqs])
        self.mat_b = np.zeros(n_eqs)

        construct_mat_b()
        construct_mat_a()

        self.solution, _, _, _ = np.linalg.lstsq(self.mat_a, self.mat_b, rcond=None)

    def print_solution(self):
        print("Unknowns are")
        for i in range(len(self.unknown_labels)):
            print(self.unknown_labels[i], " : ", np.round(self.solution[i], 1))

    def plot(self, scale, vertical):
        ax = plt.figure().add_subplot(projection='3d')

        def draw_moment_circle(c, r, v):

            a = np.array([1, 1, 1])
            if v[2] != 0:
                a[2] = -(v[0] * a[0] + v[1] * a[1]) / v[2]
            elif v[1] != 0:
                a[1] = -(v[0] * a[0] + v[2] * a[2]) / v[1]
            elif v[0] != 0:
                a[0] = -(v[1] * a[1] + v[2] * a[2]) / v[0]

            a = a / np.linalg.norm(a)
            b = np.cross(a, v)

            theta = np.linspace(0, 3 * np.pi / 2, 100)
            x = c[0] + r * np.cos(theta) * a[0] + r * np.sin(theta) * b[0]
            y = c[1] + r * np.cos(theta) * a[1] + r * np.sin(theta) * b[1]
            z = c[2] + r * np.cos(theta) * a[2] + r * np.sin(theta) * b[2]

            plt.quiver(x[-1], y[-1], z[-1], x[-1] - x[-2], y[-1] - y[-2], z[-1] - z[-2], arrow_length_ratio=15,
                       color='green')
            ax.plot(x, y, z, color='green')

        for i in range(self.n_bodies):
            body_origin = self.bodies[i].points[0]

            for j in range(self.bodies[i].n_unknowns):
                unknown = self.bodies[i].unknowns[j]
                if unknown[1, :].any() == 1:
                    ax.quiver(body_origin[0] + unknown[0, 0], body_origin[1] + unknown[0, 1],
                              body_origin[2] + unknown[0, 2],
                              unknown[1, 0], unknown[1, 1], unknown[1, 2], length=scale, color='red')
                    ax.text(body_origin[0] + unknown[0, 0] + scale * unknown[1, 0] + scale / 10,
                            body_origin[1] + unknown[0, 1] +
                            scale * unknown[1, 1] + scale / 10,
                            body_origin[2] + unknown[0, 2] + scale * unknown[1, 2] + scale / 10,
                            self.unknown_labels[i * self.bodies[i].n_unknowns + j])
                elif unknown[2, :].any() == 1:
                    draw_moment_circle(body_origin + unknown[0, :] + 0.5 * unknown[2, :], 0.25 * scale, unknown[2, :])
                    ax.text(body_origin[0] + unknown[0, 0] + 0.5 * scale * unknown[2, 0],
                            body_origin[1] + unknown[0, 1] + 0.5 * scale * unknown[2, 1],
                            body_origin[2] + unknown[0, 2] + 0.5 * scale * unknown[2, 2],
                            self.unknown_labels[i * self.bodies[i].n_unknowns + j])

            for j in range(self.bodies[i].n_loads):
                load = self.bodies[i].loads[j]
                load_norm = np.linalg.norm(load)

                if np.any(load[1, :]):
                    ax.quiver(body_origin[0] + load[0, 0], body_origin[1] + load[0, 1], body_origin[2] + load[0, 2],
                              load[1, 0] / load_norm, load[1, 1] / load_norm, load[1, 2] / load_norm, length=scale,
                              color='blue')
                if np.any(load[2, :]):
                    draw_moment_circle(body_origin + load[0, :] + 0.1 * load[2, :] / load_norm, 0.25 * scale,
                                       load[2, :] / load_norm)

            ax.plot(*zip(*self.bodies[i].points), linewidth=3, color='black')
        ax.view_init(vertical_axis=vertical)

        ax.margins(0.15, 0.15, 0.15)
        ax.autoscale()
        ax.set_box_aspect([1, 1, 1])
        set_axes_equal(ax)
        plt.subplots_adjust(bottom=-0.1, top=1.2)
        ax.zaxis.set_ticklabels([])
        ax.view_init(azim=0, elev=0)
        ax.set_proj_type('ortho')
        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis")
        ax.set_zlabel("Z Axis")
        plt.show()


def basic_example():
    transmission = CoordinateSpace()
    transmission.unknown_labels = ['Ra', 'Ry1', 'Rb', 'Ry2']
    transmission.dofs = np.array([1, 1, 0, 0, 0, 1], dtype=bool)

    transmission.add_body(np.array([[0, 0, 0],
                                    [0.5, 0, 0]]))
    transmission.add_body(np.array([[0.5, 0.1, 0],
                                    [1, 0.1, 0]]))
    transmission.bodies[0].add_load(np.array([[0, 0, 0],
                                              [0, 200, 0],
                                              [0, 0, 0]]))
    transmission.bodies[0].add_unknown(np.array([[0.25, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=0)
    transmission.bodies[0].add_unknown(np.array([[0.5, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=1)

    transmission.bodies[1].add_unknown(np.array([[0, 0, 0],
                                                 [0, -1, 0],
                                                 [0, 0, 0]]), index=1)
    transmission.bodies[1].add_unknown(np.array([[0.25, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=2)
    transmission.bodies[1].add_unknown(np.array([[0.5, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=3)

    transmission.solve()
    transmission.print_solution()
    transmission.plot(0.25, 'y')


def example_3d():
    rod = CoordinateSpace()
    rod.unknown_labels = ['Cx', 'Cy', 'Cz', 'Mx', 'My', 'Mz']
    rod.dofs = np.array([1, 1, 1, 1, 1, 1], dtype=bool)

    rod.add_body(np.array([[0, 4, 0], [6, 4, 0], [4, 4, 5]]))

    rod.bodies[0].add_load(np.array([[4, 4, 5],
                                     [2 * 500 / np.sqrt(21), -4 * 500 / np.sqrt(21), -1 * 500 / np.sqrt(21)],
                                     [0, 0, 0]]))
    rod.bodies[0].add_unknown(np.array([[0, 4, 0],
                                        [1, 0, 0],
                                        [0, 0, 0]]), index=0)
    rod.bodies[0].add_unknown(np.array([[0, 4, 0],
                                        [0, 1, 0],
                                        [0, 0, 0]]), index=1)
    rod.bodies[0].add_unknown(np.array([[0, 4, 0],
                                        [0, 0, 1],
                                        [0, 0, 0]]), index=2)
    rod.bodies[0].add_unknown(np.array([[0, 4, 0],
                                        [0, 0, 0],
                                        [1, 0, 0]]), index=3)
    rod.bodies[0].add_unknown(np.array([[0, 4, 0],
                                        [0, 0, 0],
                                        [0, 1, 0]]), index=4)
    rod.bodies[0].add_unknown(np.array([[0, 4, 0],
                                        [0, 0, 0],
                                        [0, 0, 1]]), index=5)

    rod.solve()
    rod.print_solution()
    rod.plot(2, vertical='y')


def shaft_example():
    shaft = CoordinateSpace()
    shaft.unknown_labels = ['Ax', 'Ay', 'Az', 'Bx', 'By', 'F']
    shaft.dofs = np.array([1, 1, 1, 1, 1, 1], dtype=bool)

    shaft.add_body(np.array([[0, 0, -1.7],
                             [0, 0, 0],
                             [0.25, 0, 0],
                             [0.25, 0, 0.4]]))

    shaft.bodies[0].add_load(np.array([[-0.06, 0, -1.1],
                                       [0, -500, 0],
                                       [0, 0, 0]]))
    shaft.bodies[0].add_unknown(np.array([[0, 0, -1.7],
                                          [1, 0, 0],
                                          [0, 0, 0]]), index=0)
    shaft.bodies[0].add_unknown(np.array([[0, 0, -1.7],
                                          [0, 1, 0],
                                          [0, 0, 0]]), index=1)
    shaft.bodies[0].add_unknown(np.array([[0, 0, -1.7],
                                          [0, 0, 1],
                                          [0, 0, 0]]), index=2)
    shaft.bodies[0].add_unknown(np.array([[0, 0, -0.3],
                                          [1, 0, 0],
                                          [0, 0, 0]]), index=3)
    shaft.bodies[0].add_unknown(np.array([[0, 0, -0.3],
                                          [0, 1, 0],
                                          [0, 0, 0]]), index=4)
    shaft.bodies[0].add_unknown(np.array([[0.25, 0, 0.4],
                                          [0, -1, 0],
                                          [0, 0, 0]]), index=5)

    shaft.solve()
    shaft.print_solution()
    shaft.plot(0.4, vertical='y')


if __name__ == "__main__":
    np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})
    shaft_example()
