import numpy as np

# DO NOT USE


class SolidBody:
    def __init__(self):
        self.n_known = 0
        self.n_unknown = 0
        self.loads = []
        self.unknown_mask = []
        self.solution = None
        self.unknowns = []

    def add_load(self, load_state, load_type):

        if load_type == "known":
            self.loads += [load_state]
            self.n_known += 1
        elif load_type == "unknown":

            self.unknown_mask += list(load_state[1:, :].flatten())

            self.loads += [np.array([load_state[0, :],
                                    [0, 0, 0],
                                    [0, 0, 0]])]
            self.n_unknown += 1

    def solve(self):
        # Assembling the known array
        known_sums = np.zeros(6)
        for i in range(self.n_known):
            moment = np.cross(self.loads[i][1, :], self.loads[i][0, :])

            known_sums[0] -= self.loads[i][1, 0]
            known_sums[1] -= self.loads[i][1, 1]
            known_sums[2] -= self.loads[i][1, 2]
            known_sums[3] -= (self.loads[i][2, 0] + moment[0])
            known_sums[4] -= (self.loads[i][2, 1] + moment[1])
            known_sums[5] -= (self.loads[i][2, 2] + moment[2])

        known_vect = np.array(known_sums)

        # Assembling the array of coefficients
        coefficients_mat = np.zeros([6, 6 * self.n_unknown])

        for i in range(self.n_unknown):

            moment_coefficients = np.array(
                [[0, self.loads[self.n_known + i][0, 2], -self.loads[self.n_known + i][0, 1]],
                 [-self.loads[self.n_known + i][0, 2], 0, self.loads[self.n_known + i][0, 0]],
                 [self.loads[self.n_known + i][0, 1], -self.loads[self.n_known + i][0, 0], 0]])

            coefficients_mat[3:, 6 * i:6 * i + 3] = moment_coefficients

            for j in range(6):
                coefficients_mat[j, j + 6 * i] = 1

        # Removing rows that are not specified as unknowns

        mask_indicies = np.where(self.unknown_mask)[0]
        coefficients_mat = np.delete(coefficients_mat, mask_indicies, 1)

        row_indicies = np.where(coefficients_mat.any(axis=1) == 0)[0]
        coefficients_mat = np.delete(coefficients_mat, row_indicies, axis=0)
        known_vect = np.delete(known_vect, row_indicies)

        self.solution = np.linalg.solve(coefficients_mat, known_vect)

    def display_solution(self):
        print("Unknowns are")
        for i in range(len(self.unknowns)):
            print(self.unknowns[i], " : ", self.solution[i])


def main():

    # The load state is a 3x3 array consisting of
    # [px, py, pz], [Fx, Fy, Fz], [Mx, My, Mz]
    # For a known load, all of these should be filled with the known values
    # For an unknown load, the position row is still populated
    # And then the remaining 6 forces take either a 1 or 0
    # 0 if the unknown can support a force/moment in that direction
    # 1 if not
    # mx and mz may be swapped (unsure why)

    # It is very important to add unknowns in order
    # Then the unknowns variable is filled with desired variable names
    # Call solve to perform calc, and then display_solution to neatly printed calculated forces and moments

    cantilever = SolidBody()

    cantilever.add_load(load_type='known', load_state=np.array([[1, 0, 0],
                                                                [0, -600, 0],
                                                                [0, 0, 0]]))

    cantilever.add_load(load_type='known', load_state=np.array([[4, 0, 0],
                                                                [0, -800, 0],
                                                                [0, 0, 0]]))

    cantilever.add_load(load_type='known', load_state=np.array([[4, 0, 0],
                                                                [0, 0, 0],
                                                                [0, 0, -900]]))

    cantilever.add_load(load_type='unknown', load_state=np.array([[0, 0, 0],
                                                                  [0, 0, 1],
                                                                  [1, 1, 1]]))

    cantilever.add_load(load_type='unknown', load_state=np.array([[0, 1.5, 0],
                                                                  [0, 1, 1],
                                                                  [1, 1, 1]]))

    cantilever.unknowns = ['Fdx', 'Fax', 'Fay']
    cantilever.solve()
    cantilever.display_solution()


if __name__ == "__main__":
    main()
