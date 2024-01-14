# Author: ivantregear
# Date Created: 23/02/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import MultiBodyStaticEquilibriumSolver
import numpy as np
import matplotlib.pyplot as plt


def rotate_gear_force(fr, ft, theta):
    fx_r = fr * np.cos(theta) - ft * np.sin(theta)
    fy_r = fr * np.sin(theta) + ft * np.cos(theta)
    fz_r = 0

    return fx_r, fy_r, fz_r


def rotate_gear_pos(d_ref, theta, gear_pos):
    pos_arr = np.array([0, 0, gear_pos]) + d_ref / 2 * np.array([np.cos(theta), np.sin(theta), 0])

    return pos_arr[0], pos_arr[1], pos_arr[2]


def main(t1, t2):
    np.set_printoptions(edgeitems=30, linewidth=100000,
                        formatter=dict(float=lambda x: "%.3f" % x))
    transmission = MultiBodyStaticEquilibriumSolver.CoordinateSpace()

    transmission.unknown_labels = ['R1ax', 'R1ay', 'R1bx', 'R1by', 'R1bz',
                                   'R2ax', 'R2ay', 'R2bx', 'R2by', 'R2bz',
                                   'R3ax', 'R3ay', 'R3bx', 'R3by', 'R3bz',
                                   'R4ax', 'R4ay', 'R4bx', 'R4by', 'R4bz',
                                   'R5ax', 'R5ay', 'R5bx', 'R5by', 'R5bz']

    transmission.dofs = np.array([1, 1, 1, 1, 1, 1], dtype=bool)

    state = "right_off"  # full, left_off, right_off

    t_max = 3410.46
    sf = 1.3
    ratio = 9.25

    t_in = t_max * sf
    t_out = t_max / ratio * 2 * sf

    theta = np.radians(np.array([t1, t2]))  # middle stage, edge stage

    drefs = np.array([0.5727, 0.1847, 0.4157, 0.1386])  # 4 sizes of gear
    force_r = np.array([447.2, 111.8])
    force_t = np.array([1228.7, 307.2])

    gear_widths = np.array([0.1059, 0.1059, 0.0816, 0.0816])  # 4 sizes of gear
    shaft_diameters = np.array([0.138, 0.095, 0.052])  # 3 different shaft sizes
    shaft_lengths = np.array([0.1727, 0.2709, 0.1416])  # 3 different shaft sizes
    gear_pos = np.array([shaft_lengths[0]/2, 0.0915, 0.1903, shaft_lengths[2]/2])  # 4 positions

    steel_density = 7950  # kg/m^3

    gear_weights = np.pi / 4 * steel_density * np.multiply(drefs ** 2, gear_widths) * 9.81
    shaft_weights = np.pi / 4 * steel_density * np.multiply(shaft_diameters ** 2, shaft_lengths) * 9.81

    offset_1 = -5.81e-3
    offset_2 = 113.7e-3

    transmission.add_body(np.array([[0, 0, 0],
                                    [0, 0, shaft_lengths[0]]]))

    transmission.add_body(
        np.array([[-(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]), (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]), offset_1],
                  [-(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]), (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]),
                   offset_1 + shaft_lengths[1]]]))

    transmission.add_body(
        np.array([[(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]), (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]), offset_1],
                  [(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]), (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]),
                   offset_1 + shaft_lengths[1]]]))

    transmission.add_body(np.array([[-(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]) - (drefs[2] + drefs[3]) / 2 * np.cos(
        theta[1]), (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]) - (drefs[2] + drefs[3]) / 2 * np.sin(theta[1]), offset_2],
                                    [-(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]) - (drefs[2] + drefs[3]) / 2 * np.cos(
                                        theta[1]),
                                     (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]) - (drefs[2] + drefs[3]) / 2 * np.sin(
                                         theta[1]), offset_2 + shaft_lengths[2]]]))

    transmission.add_body(np.array([[(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]) + (drefs[2] + drefs[3]) / 2 * np.cos(
        theta[1]), (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]) - (drefs[2] + drefs[3]) / 2 * np.sin(theta[1]), offset_2],
                                    [(drefs[0] + drefs[1]) / 2 * np.cos(theta[0]) + (drefs[2] + drefs[3]) / 2 * np.cos(
                                        theta[1]),
                                     (drefs[0] + drefs[1]) / 2 * np.sin(theta[0]) - (drefs[2] + drefs[3]) / 2 * np.sin(
                                         theta[1]), offset_2 + shaft_lengths[2]]]))

    transmission.bodies[0].add_unknown(np.array([[0, 0, 0],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=0)
    transmission.bodies[0].add_unknown(np.array([[0, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=1)
    transmission.bodies[0].add_unknown(np.array([[0, 0, shaft_lengths[0]],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=2)
    transmission.bodies[0].add_unknown(np.array([[0, 0, shaft_lengths[0]],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=3)
    transmission.bodies[0].add_unknown(np.array([[0, 0, shaft_lengths[0]],
                                                 [0, 0, 1],
                                                 [0, 0, 0]]), index=4)

    transmission.bodies[1].add_unknown(np.array([[0, 0, 0],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=5)
    transmission.bodies[1].add_unknown(np.array([[0, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=6)
    transmission.bodies[1].add_unknown(np.array([[0, 0, shaft_lengths[1]],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=7)
    transmission.bodies[1].add_unknown(np.array([[0, 0, shaft_lengths[1]],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=8)
    transmission.bodies[1].add_unknown(np.array([[0, 0, shaft_lengths[1]],
                                                 [0, 0, 1],
                                                 [0, 0, 0]]), index=9)

    transmission.bodies[2].add_unknown(np.array([[0, 0, 0],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=10)
    transmission.bodies[2].add_unknown(np.array([[0, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=11)
    transmission.bodies[2].add_unknown(np.array([[0, 0, shaft_lengths[1]],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=12)
    transmission.bodies[2].add_unknown(np.array([[0, 0, shaft_lengths[1]],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=13)
    transmission.bodies[2].add_unknown(np.array([[0, 0, shaft_lengths[1]],
                                                 [0, 0, 1],
                                                 [0, 0, 0]]), index=14)

    transmission.bodies[3].add_unknown(np.array([[0, 0, 0],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=15)
    transmission.bodies[3].add_unknown(np.array([[0, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=16)
    transmission.bodies[3].add_unknown(np.array([[0, 0, shaft_lengths[2]],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=17)
    transmission.bodies[3].add_unknown(np.array([[0, 0, shaft_lengths[2]],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=18)
    transmission.bodies[3].add_unknown(np.array([[0, 0, shaft_lengths[2]],
                                                 [0, 0, 1],
                                                 [0, 0, 0]]), index=19)

    transmission.bodies[4].add_unknown(np.array([[0, 0, 0],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=20)
    transmission.bodies[4].add_unknown(np.array([[0, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=21)
    transmission.bodies[4].add_unknown(np.array([[0, 0, shaft_lengths[2]],
                                                 [1, 0, 0],
                                                 [0, 0, 0]]), index=22)
    transmission.bodies[4].add_unknown(np.array([[0, 0, shaft_lengths[2]],
                                                 [0, 1, 0],
                                                 [0, 0, 0]]), index=23)
    transmission.bodies[4].add_unknown(np.array([[0, 0, shaft_lengths[2]],
                                                 [0, 0, 1],
                                                 [0, 0, 0]]), index=24)
    # Propeller and engine torques
    transmission.bodies[0].add_load(np.array([[0, 0, 0],
                                              [0, 0, 0],
                                              [0, 0, t_in]]))
    if state != "left_off" or state == "full":
        transmission.bodies[3].add_load(np.array([[0, 0, shaft_lengths[2]],
                                                  [0, 0, 0],
                                                  [0, 0, t_out]]))
    if state != "right_off" or state == "full":
        transmission.bodies[4].add_load(np.array([[0, 0, shaft_lengths[2]],
                                                  [0, 0, 0],
                                                  [0, 0, t_out]]))
    # Shaft weights
    transmission.bodies[0].add_load(np.array([[0, 0, shaft_lengths[0]/2],
                                              [0, -shaft_weights[0], 0],
                                              [0, 0, 0]]))
    transmission.bodies[1].add_load(np.array([[0, 0, shaft_lengths[1] / 2],
                                              [0, -shaft_weights[1], 0],
                                              [0, 0, 0]]))
    transmission.bodies[2].add_load(np.array([[0, 0, shaft_lengths[1] / 2],
                                              [0, -shaft_weights[1], 0],
                                              [0, 0, 0]]))
    transmission.bodies[3].add_load(np.array([[0, 0, shaft_lengths[2] / 2],
                                              [0, -shaft_weights[2], 0],
                                              [0, 0, 0]]))
    transmission.bodies[4].add_load(np.array([[0, 0, shaft_lengths[2] / 2],
                                              [0, -shaft_weights[2], 0],
                                              [0, 0, 0]]))

    # Gear weights
    transmission.bodies[0].add_load(np.array([[0, 0, gear_pos[0]],
                                              [0, -gear_weights[0], 0],
                                              [0, 0, 0]]))
    transmission.bodies[1].add_load(np.array([[0, 0, gear_pos[1]],
                                              [0, -gear_weights[1], 0],
                                              [0, 0, 0]]))
    transmission.bodies[1].add_load(np.array([[0, 0, gear_pos[2]],
                                              [0, -gear_weights[2], 0],
                                              [0, 0, 0]]))
    transmission.bodies[2].add_load(np.array([[0, 0, gear_pos[1]],
                                              [0, -gear_weights[1], 0],
                                              [0, 0, 0]]))
    transmission.bodies[2].add_load(np.array([[0, 0, gear_pos[2]],
                                              [0, -gear_weights[2], 0],
                                              [0, 0, 0]]))
    transmission.bodies[3].add_load(np.array([[0, 0, gear_pos[3]],
                                              [0, -gear_weights[3], 0],
                                              [0, 0, 0]]))
    transmission.bodies[4].add_load(np.array([[0, 0, gear_pos[3]],
                                              [0, -gear_weights[3], 0],
                                              [0, 0, 0]]))

    # Gear forces
    f0x, f0y, f0z = rotate_gear_force(force_r[0], force_t[0], theta[0])
    p0x, p0y, p0z = rotate_gear_pos(drefs[0], theta[0], gear_pos[0])

    f0x_, f0y_, f0z_ = rotate_gear_force(force_r[0], force_t[0], -theta[0])

    if state != "left_off" or state == "full":
        transmission.bodies[0].add_load(np.array([[-p0x, p0y, p0z],
                                                  [f0x, -f0y, f0z],
                                                  [0, 0, 0]]))
    if state != "right_off" or state == "full":
        transmission.bodies[0].add_load(np.array([[p0x, p0y, p0z],
                                                  [-f0x_, f0y_, f0z_],
                                                  [0, 0, 0]]))

    f1x, f1y, f1z = rotate_gear_force(force_r[0], force_t[0], theta[0])
    p1x, p1y, p1z = rotate_gear_pos(drefs[1], theta[0], gear_pos[1])

    f1x_, f1y_, f1z_ = rotate_gear_force(force_r[0], force_t[0], -theta[0])

    if state != "left_off" or state == "full":
        transmission.bodies[1].add_load(np.array([[p1x, -p1y, p1z],
                                                  [-f1x, f1y, f1z],
                                                  [0, 0, 0]]))
    if state != "right_off" or state == "full":
        transmission.bodies[2].add_load(np.array([[-p1x, -p1y, p1z],
                                                  [f1x_, -f1y_, f1z_],
                                                  [0, 0, 0]]))

    f2x, f2y, f2z = rotate_gear_force(force_r[1], force_t[1], theta[1])
    p2x, p2y, p2z = rotate_gear_pos(drefs[2], theta[1], gear_pos[2])

    f2x_, f2y_, f2z_ = rotate_gear_force(force_r[1], force_t[1], -theta[1])

    if state != "left_off" or state == "full":
        transmission.bodies[1].add_load(np.array([[-p2x, -p2y, p2z],
                                                  [f2x_, -f2y_, f2z_],
                                                  [0, 0, 0]]))
    if state != "right_off" or state == "full":
        transmission.bodies[2].add_load(np.array([[p2x, -p2y, p2z],
                                                  [-f2x, f2y, f2z],
                                                  [0, 0, 0]]))

    f3x, f3y, f3z = rotate_gear_force(force_r[1], force_t[1], theta[1])
    p3x, p3y, p3z = rotate_gear_pos(drefs[3], theta[1], gear_pos[3])

    f3x_, f3y_, f3z_ = rotate_gear_force(force_r[1], force_t[1], -theta[1])

    if state != "left_off" or state == "full":
        transmission.bodies[3].add_load(np.array([[p3x, p3y, p3z],
                                                  [-f3x_, f3y_, f3z_],
                                                  [0, 0, 0]]))
    if state != "right_off" or state == "full":
        transmission.bodies[4].add_load(np.array([[-p3x, p3y, p3z],
                                                  [f3x, -f3y, f3z],
                                                  [0, 0, 0]]))

    transmission.solve()
    transmission.print_solution()
    transmission.plot(0.25, 'y')

    return transmission.solution


if __name__ == "__main__":

    theta_parameter_study = False  # for single plot, set to false

    if theta_parameter_study:
        fig, ax = plt.subplots(nrows=5, ncols=5)

        load_labels = ['R1ax', 'R1ay', 'R1bx', 'R1by', 'R1bz',
                                       'R2ax', 'R2ay', 'R2bx', 'R2by', 'R2bz',
                                       'R3ax', 'R3ay', 'R3bx', 'R3by', 'R3bz',
                                       'R4ax', 'R4ay', 'R4bx', 'R4by', 'R4bz',
                                       'R5ax', 'R5ay', 'R5bx', 'R5by', 'R5bz']

        n = 30

        t1 = np.linspace(-90, 90, n)
        t2 = np.linspace(-90, 90, n)

        sol_list = []

        for theta1 in t1:
            for theta2 in t2:
                sol = main(theta1, theta2)
                sol_list += [sol]

        sol_array = np.array(sol_list)
        max_load = np.max(sol_array, axis=0)

        for idx, axis in enumerate(ax.flatten()):
            axis.set_title("{} @ {}Nm".format(load_labels[idx], round(max_load[idx]), 2))
            axis.set_xlabel("Theta 1 [deg]")
            axis.set_ylabel("Theta 2 [deg]")
            sol_grid = np.array(sol_array[:, idx]).reshape(n, n)
            axis.contourf(t1, t2, sol_grid)

        plt.subplots_adjust(left=0.05,
                            bottom=0.08,
                            right=0.98,
                            top=0.96,
                            wspace=0.4,
                            hspace=0.85)
        plt.show()

    else:
        main(t1=-45, t2=-60)
