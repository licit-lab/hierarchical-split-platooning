"""
    Unit test for parameters module 
"""

from parameters import VehParameter, SimParameter, CtrParameter

import unittest


class ParameterTest(unittest.TestCase):

    def test_consistence(self):
        l_veh = 18
        cpcty = 2400 / 3600.0
        x_gap = 5
        u_ffs = 20

        x_hwy = l_veh + x_gap
        k_max = 1/x_hwy
        k_crt = cpcty / u_ffs

        veh_1 = VehParameter(cpcty, u_ffs, l_veh, x_gap)
        self.assertEqual(veh_1.x_hwy, x_hwy)
        self.assertEqual(veh_1.k_max, k_max)
        self.assertEqual(veh_1.k_crt, k_crt)

    def test_consistence_2(self):
        l_veh = 18
        cpcty = 2400 / 3600.0
        x_gap = 5
        u_ffs = 20

        x_hwy = l_veh + x_gap

        veh_1 = VehParameter(cpcty, u_ffs, l_veh)
        self.assertEqual(veh_1.x_hwy, x_hwy)


if __name__ == "__main__":
    unittest.main()
