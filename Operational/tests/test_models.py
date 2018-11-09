"""
    Unit test for Models
"""

from Operational.hrctrl.models import Vehicle
from Operational.hrctrl.params import VehParameter
import unittest


class TestModel(unittest.TestCase):

    def test_constructor(self):
        # veh_par = VehParameter()
        veh_1 = Vehicle()
        self.assertTrue(isinstance(veh_1, Vehicle))
        self.assertEqual(veh_1.n_veh, 1)

    # def test_single_step_evolution(self):
    #     veh_par = VehParameter()
    #     veh_1 = Vehicle(veh_par)
    #     self.assertTrue(isinstance(veh_1, Vehicle))
    #     self.assertEqual(veh_1.n_veh, 1)


if __name__ == "__main__":
    unittest.main()
