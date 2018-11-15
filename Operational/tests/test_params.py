"""
    Unit test for parameters module
"""

from parameter.parameters import VehParameter, SimParameter, CtrParameter

import unittest

# Test values
l_veh = 18
cpcty = 2400 / 3600.0
w_cgt = 6.25
x_gap = 5
u_ffs = 20


class TestParameter(unittest.TestCase):
    """
        Test for consistency of Traffic parameters
    """

    def test_default_creation(self):
        """
        Check empty constructor
        """
        veh_par = VehParameter()

        self.assertEqual(veh_par.cpcty * veh_par.t_dsp,
                         veh_par.u_ffs / (veh_par.w_cgt + veh_par.u_ffs))

    def test_constructor_symuvia(self):
        """
        Check construction from k_x, w, u, l_veh
        """
        veh_par = VehParameter.VehParameterSym()
        # print(veh_par)
        self.assertEqual(veh_par.cpcty * veh_par.t_dsp,
                         veh_par.u_ffs / (veh_par.w_cgt + veh_par.u_ffs))

    def test_parameter_creation(self):
        """
        Check if values for specific parameters
        """
        x_hwy = l_veh + x_gap
        k_max = 1/x_hwy
        k_crt = cpcty / u_ffs
        veh_par = VehParameter(u_ffs, l_veh, x_gap, cpcty=cpcty)
        # print(veh_par)
        self.assertEqual(veh_par.k_max, k_max)
        self.assertEqual(veh_par.k_crt, k_crt)
        self.assertEqual(veh_par.x_dsp, x_hwy)

    def test_consistence_parameter_cpcty(self):
        """
        Check Q * TAU = U / (U + W)
        """
        veh_par = VehParameter(u_ffs, l_veh, x_gap, cpcty=cpcty)
        # print(veh_par)
        self.assertEqual(veh_par.cpcty * veh_par.t_dsp,
                         veh_par.u_ffs / (veh_par.w_cgt + veh_par.u_ffs))

    def test_consistence_parameter_w_cgt(self):
        """
        Check Q * TAU = U / (U + W)
        """
        veh_par = VehParameter(u_ffs, l_veh, x_gap, w_cgt=w_cgt)
        # print(veh_par)
        self.assertEqual(veh_par.cpcty * veh_par.t_dsp,
                         veh_par.u_ffs / (veh_par.w_cgt + veh_par.u_ffs))


if __name__ == "__main__":
    unittest.main()
