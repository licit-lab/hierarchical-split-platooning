"""
    This file defines the classes and objects regarding the control strategy 
    
"""


from parameters import SimParameter, CtrParameter


class TacticalCtrl(SimParameter):
    """
    Tactical Control Class
    """
    pass

        for veh_id, event in zip(self):
            tau0 = event.get('tau0', 1)
            tauf = event.get('tauf', 2)
            tmerge = event.get('tm', 2)
            tantcp = event.get('ta', 1)
            b = tmerge - tantcp / 2  # Phase
            K = 8 * tantcp  # Compression
            r = tau0 + (tauf-tau0) / (1 + np.exp(-K*(t-b))
            self.reference[veh_id]=r

class OperationalCtr(CtrParameter, SimParameter):
    """
    Operational Control Class 
    """
    pass
