DT = 0.1 # Sample time 

KC = 0.16 # CAV max density 
KH = 0.0896 # HDV max density
VF = 25.0 # Speed free flow
W = 6.25 # Congestion speed 
E  = 25.0*0.3 # Speed drop for relaxation 

GCAV = 1/(KC*W) # Time headway CAV 
GHDV = 1/(KH*W) # Time headway HDV 
SCAV = VF/(KC*W)+1/KC #  Desired space headway CAV 
SHDV = VF/(KH*W)+1/KH #  Desired space headway HDV

dveh_twy = {'CAV': GCAV, 'HDV': GHDV}
dveh_dwy = {'CAV': 1/KC, 'HDV': 1/KH}

U_MAX = 1.5 # Max. Acceleration
U_MIN = -1.5 # Min. AccelerationDT = 0.1 # Sample time 

from collections import Counter

# Identify Leader 
def queueveh(dLeader, veh):
    """
        This function creates a queue of vehicles 
        for a particular road segment
    """
    if veh['tron'] in dLeader.keys():
        if  veh['id'] not in dLeader[veh['tron']]:
            dLeader[veh['tron']].append(veh['id'])
    else:
        dLeader[veh['tron']] = [veh['id']]
    return dLeader 

def getlead(dLeader, veh):
    """
        This function identifies the leader of a specific 
        vehicle i
    """
    idx = dLeader[veh['tron']].index(veh['id'])
    if idx != 0:
        return dLeader[veh['tron']][idx-1]
    else: 
        return dLeader[veh['tron']][idx]
        # Spacing 

def getspace(lTrajVeh):    
    """
        This function obtains spacing between two vehicles 
    """
    # Equilibrium 
    det_eq_s = lambda x: SCAV if x['type']=='CAV' else SHDV
    
    try: 
        # Case single vehicle
        if lTrajVeh['id'] == lTrajVeh['ldr']:            
            return [{'spc':0.0+det_eq_s(lTrajVeh)}] 
        else:
            # Last vehicle
            # Leader out of Network @ ti
            return [{'spc':None}] 
    except (TypeError, IndexError):        
        # Multiple veh @ ti
        space = []
        for veh in lTrajVeh:
            if veh['id'] == veh['ldr']:
                space.append(0.0+det_eq_s(veh))
            else:             
                veh_pos = veh['abs']
                ldr_id = veh['ldr']
                ldr_pos = [ldr['abs'] for ldr in lTrajVeh if ldr['id']==ldr_id]
                if ldr_pos:
                    space.append(ldr_pos[0]-veh_pos)
                else:
                    # Leader out of Network @ ti
                    space.append(0.0)
        space_dct = [{'spc': val} for val in space]
        return space_dct

# Spacing 
    
def getleaderspeed(lTrajVeh):    
    """
        This function obtains speed from the leader. 
    """    
    try: 
        # Case single vehicle
        if lTrajVeh['id'] == lTrajVeh['ldr']:
            return [{'vld': lTrajVeh['vit']}]
        else:
            # Leader out of Network @ ti
            return [{'vld':None}]                     
    except (TypeError, IndexError):        
        # Multiple veh @ ti
        speedldr = []
        for veh in lTrajVeh:
            if veh['id'] == veh['ldr']:
                speedldr.append(veh['vit'])
            else:             
                ldr_id = veh['ldr']
                ldr_vit = [ldr['vit'] for ldr in lTrajVeh if ldr['id']==ldr_id]
                if ldr_vit:
                    speedldr.append(ldr_vit[0])
                else:
                    speedldr.append(veh['vit'])
        speedldr_dct = [{'vld': val} for val in speedldr]
        return speedldr_dct    
    
def updatelist(lTrajVeh,lDict):
    """
        Considering a list of dictionaries as an input
        the funciton updates the parameter given by lDict
    """
    try:
        lTrajVeh.update(lDict[0])
    except AttributeError:
        for d,s in zip(lTrajVeh,lDict):
            d.update(s)
    return lTrajVeh

    
def typedict(veh_dict):
    """ 
        Converts dictionary file from xmltodict 
        into numeric formats to be stored in a database
    """
    data = {'id': int(veh_dict['@id']),
        'type': veh_dict['@type'],
        'tron': veh_dict['@tron'],
        'voie': int(veh_dict['@voie']),
        'dst': float(veh_dict['@dst']),
        'abs': float(veh_dict['@abs']),
        'vit': float(veh_dict['@vit']),
       }
    return data 


def check_veh_creation(lVehDataFormat, nVehInitial):
    """
        Check that all initial conditions for the problem
    """
    
    try: 
        VehRoad = Counter([x['tron'] for x in lVehDataFormat])
    except TypeError:
        VehRoad = {lVehDataFormat['tron']:0}
    
    for key,val in VehRoad.items():
        if val<nVehInitial[key]:
            return False 
    return True
    