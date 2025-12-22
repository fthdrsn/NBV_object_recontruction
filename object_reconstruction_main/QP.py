"""
# Copyright (c) 2019-2022 DQ Robotics Developers
#
#    This file is part of DQ Robotics.
#
#    DQ Robotics is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    DQ Robotics is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with DQ Robotics.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Contributors:
#   - Murilo M. Marinho, email: murilo@g.u-tokyo.ac.jp
#
# ################################################################
"""
import numpy as np
import quadprog


class DQ_QuadprogSolver_Custom():
    def __init__(self):

        # default of np.finfo(np.float64).eps is already included in the solver
        self.equality_constraints_tolerance = 0

    def set_equality_constraints_tolerance(self, tolerance):
        """
        Set allowed tolerance for the equality constraints
        :param tolerance: Tolerance allowed for equality constraints
        """
        self.equality_constraints_tolerance = tolerance

    def get_equality_constraints_tolerance(self):
        """
        Get allowed tolerance for the equality constraints
        :return: Current tolerance
        """
        return self.equality_constraints_tolerance

    def solve_quadratic_program(self, H, f, A, b, Aeq, beq):
        """
         Solves the following quadratic program
            min(x)  0.5*x'Hx + f'x
            s.t.    Ax <= b
                    Aeqx = beq.
         Method signature is compatible with MATLAB's 'quadprog'.
         :param H: the n x n matrix of the quadratic coeficitients of the decision variables.
         :param f: the n x 1 vector of the linear coeficients of the decision variables.
         :param A: the m x n matrix of inequality constraints.
         :param b: the m x 1 value for the inequality constraints.
         :param Aeq: the m x n matrix of equality constraints.
         :param beq: the m x 1 value for the inequality constraints.
         :return: the optimal x
        """
        A_internal = A
        b_internal = b
        meq = 0
        if Aeq is not None and beq is not None:
            if Aeq.shape == (0, 0) or beq.shape == 0:
                pass
            else:
                A_internal = np.vstack([Aeq, A])
                beq = beq.reshape(-1)
                b_internal = np.concatenate(
                    [beq + self.equality_constraints_tolerance, b.reshape(-1)])
                meq = Aeq.shape[0]
        if A_internal.shape == (0, 0) or b_internal.shape == 0:
            # Calls from DQRobotics CPP will trigger this condition
            A_internal = np.zeros((1, H.shape[0]))
            b_internal = np.zeros(1)

        (x, f, xu, iterations, lagrangian, iact) = quadprog.solve_qp(G=H,
                                                                     a=-f,
                                                                     C=-np.transpose(A_internal),
                                                                     b=-b_internal,
                                                                     meq=meq)
        return x

    def compute_control_signal(self, J, err, damping=0.01, gain=1, Aineq=None, bineq=None, Aeq=None, beq=None, terr=None):
        dim = J.shape[1]
        H = J.T @ J
        if terr is not None:
            damping_adaptive = 2*np.linalg.norm(terr)**1.1
            # #print(damping_adaptive)
            W_input = np.array([1, 1, 1, damping_adaptive, damping_adaptive,
                                damping_adaptive, damping_adaptive, damping_adaptive])
            H = .5 * (H + H.T) + W_input[np.newaxis, :]*np.eye(dim, dim)
        else:
            H = .5 * (H + H.T) + damping*np.eye(dim, dim)
        f = gain * J.T @ err
        if Aineq is None:
            A_internal = np.zeros((1, H.shape[0]))
            b_internal = np.zeros(1)
        else:
            A_internal = Aineq
            b_internal = bineq
        u = self.solve_quadratic_program(H,
                                         f.reshape(dim),
                                         A_internal,
                                         b_internal,
                                         Aeq,
                                         beq)
        return u
