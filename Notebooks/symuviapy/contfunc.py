import numpy as np
import pandas as pd

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
U_MIN = -1.5 # Min. Acceleration

# Imposed leadership
dveh_ldr = {0: 0, 1: 0, 2: 1, 3: 2, 5: 3, 6: 5, 8: 6, 9: 8}
dveh_idx = {0: 0, 1: 1, 2: 2, 3: 3, 5: 4, 6: 5, 8: 6, 9: 7}

def reversedEnumerate(*args):
    """ Inverse enumeration iterator"""
    revArg = [np.flip(x, axis = 0) for x in args]
    return zip(range(len(args[0])-1, -1, -1), *revArg)

def find_idx_ldr(results):
#     """ From dbQuery finds idx or leader for CAVs"""
    # Frozen network (A-priori)
    ldrl = [dveh_ldr[x[1]] for x in results if x[2]=='CAV']
    idx_ldr = [dveh_idx[x] for x in ldrl]
    
    return idx_ldr, ldrl  

def initial_setup_mpc(results, h_ref):
    """ Initialize variables for controller
    """
    
    TGref = format_reference(h_ref)  
    h = TGref.shape[0] 
    
    n_CAV = len([ty[2] for ty in results if ty[2]=='CAV'])
    dCAVu = [h, n_CAV]
    # print(f'Dimensions control: {dCAVu}')

    Sref = np.zeros(dCAVu)

    S = np.zeros(dCAVu)
    V = np.zeros(dCAVu)
    DV = np.zeros(dCAVu)
    Lv = np.zeros(dCAVu)
    Ls = np.zeros(dCAVu)
    return (Sref, TGref, S, V, DV, Ls, Lv)

def format_reference(h_ref):
    """ Convert query from a reference into a 
        numpy array
    """

    # Rearrange 
    refDf = pd.DataFrame(h_ref, columns = ['ti','id','gapt'])
    # Pivot to pass vehicles as columns
    refMat = pd.pivot_table(refDf, index='ti', columns='id')['gapt']
    refMat = refMat.as_matrix()
    
    return refMat 
    
def compute_control(results, h_ref, u_lead):
              
    _, Tgref, S, V, DV, Ls, Lv = initial_setup_mpc(results, h_ref)
    
    # Static leadership
    ldr_pos, _ = find_idx_ldr(results)
    
    S0 = [s[9] for s in results if s[2]=='CAV']
    V0 = [v[7] for v in results if v[2]=='CAV']
    DV0 = [dv[10]-dv[7] for dv in results if dv[2]=='CAV']
    U_ext = Lv
    #U_ext[:,0] = u_lead # Head acceleration (external)

    
    # Initialize global variables 
    S[0] = S0
    V[0] = V0
    DV[0] = DV0    
    h = len(S)
    n = 0
    n_prev = 0
    
    # Parameters 
    C1 = 0.1
    C2 = 1
    C3 = 0.5
    ALPHA = 0.01
    1
    EPS  = 0.1
    error = 100
    
    
    bSuccess = 2 
    N = 100001 # number of iterations
    step = iter(range(N))
    
    while (error > EPS) and (bSuccess>0):
        try: 
            next(step)
            U_star = -Lv/(2*C3)
            U_star = np.clip(U_star, U_MIN, U_MAX) 
            
            DU = U_star[:,ldr_pos]-U_star[:] + U_ext 

            # Forward evolution
            for i,u_s,du in zip(range(h), U_star, DU):        
                if i<len(S)-1:
                    DV[i+1] = DV[i]+ DT * du
                    S[i+1] = S[i] + DT * DV[i]
                    V[i+1] = V[i]+ DT * u_s
        

            Sref = V * Tgref + 1/KC
            
            # Forward plots 
            # plot_forward(Sref, Tgref, S, V, DV, U_star)
            
            ls = np.zeros(Ls.shape) 
            lv = np.zeros(Lv.shape)


            # Backward evolution
            for i, s, v, dv, tg in reversedEnumerate(S, V, DV, Tgref):            
                if i>0:
                    sref = v * tg + 1/KC
                    lv[i-1] = lv[i] + DT * (-2 * C1 * (s-sref) * tg - C2 * dv - ls[i])
                    ls[i-1] = ls[i] + DT * (2 * C1 * (s-sref))
        

            # Update 
            Ls = (1 - ALPHA) * Ls + ALPHA * ls
            Lv = (1 - ALPHA) * Lv + ALPHA * lv      
            
            # Backwards plots 
            # plot_backwards(ls, lv, Ls, Lv)

            error = np.linalg.norm(Ls - ls) + np.linalg.norm(Lv-lv)
            # print(f'Iteration: {n}, Error: {error}')
            

            # Routine for changing convergence parameter

            if error > 10e5:
                raise AssertionError('Algorithm does not converge ')
            if n >= 500:
                ALPHA = max(ALPHA - 0.01, 0.01)
                #print(f'Reaching {n} iterations: Reducing alpha: {ALPHA}')  
                #print(f'Error before update {error}')
                if n > 10000:
                    raise AssertionError(
                        'Maximum iterations reached by the algorithm')
                n_prev = n + n_prev
                n = 0
            if error <= EPS:
                bSuccess = 0
                
            n += 1
    
        except StopIteration:
            print('Stop by iteration')
            print('Last simulation step at iteration: {}'.format(n+n_prev))
            bSuccess = 0
    
    n = n + n_prev
    return (S, V, DV, U_star, DU, n)

def determine_lane_change(CAVabsP):
    """ Returns the tuple (tron, voie) for a 
        CAV vehicle based on positions updates.
    """    
    
    # Works only in current network
    
    CAVtron = []
    CAVvoie = []
    
    for abs_x in CAVabsP:
        if (abs_x <= 0):
            CAVtron.append('In_main')
            CAVvoie.append(1)
        elif (abs_x > 0) and (abs_x <=100.0):
            CAVtron.append('Merge_zone')
            CAVvoie.append(2)            
        else:
            CAVtron.append('Out_main')
            CAVvoie.append(1)
    return CAVtron, CAVvoie

def update_state(S, V, DV, U_star, DU, n, results_closed):
    """ Updates the state and computes closed loop updates
    """

    # NOTE: To be taken into account. Closed loop simulations
    # run without Symuvia. Requires implementation of the connection 
    # NO LANE CHANGE MODEL IMPLENTED FOR HDV 
    
     # Forward evolution 
    DVp = DV[0] + DT * DU[0]
    Sp = S[0] + DT * DV[0]
    Vp = V[0] + DT * U_star[0] 
    
    # t_i, id, type, from (results_closed):
    # Keep the same along the simulation
    CAVti = [x[0]+ DT for x in results_closed if x[2]=='CAV']
    CAVti = [float(np.round(x,1)) for x in CAVti]
    CAVid = [x[1] for x in results_closed if x[2]=='CAV']
    CAVtype = [x[2] for x in results_closed if x[2]=='CAV']

    # Updates

    # Postition query
    CAVdst = [x[5] for x in results_closed if x[2]=='CAV']   
    CAVabs = [x[6] for x in results_closed if x[2]=='CAV']  
    
    # Updates from closed loop
    CAVdstP = [x+ DT * v for (x,v) in zip(CAVdst, V[0])] #or Vp?
    CAVabsP = [x+ DT * v for (x,v) in zip(CAVabs, V[0])] #or Vp?    
    
    
    # Lane change 
    CAVtron, CAVvoie = determine_lane_change(CAVabsP)
    
    # State updates 
    ldr_pos, ldr_list = find_idx_ldr(results_closed)
    CAVvitP = Vp
    CAVldr = ldr_list
    CAVldr = [int(x) for x in ldr_list]
    CAVspcP = Sp
    CAVvldP = DVp + Vp
    
    # Render CAV results for dB
    CAViter = zip(CAVti, CAVid, CAVtype, CAVtron, CAVvoie,
                  CAVdstP, CAVabsP, CAVvitP, CAVldr, 
                  CAVspcP, CAVvldP)
    
    keys = ('ti', 'id', 'type', 'tron', 'voie', 
            'dst', 'abs', 'vit', 'ldr', 
            'spc','vld')
    
    keysU = ('ti', 'id', 'type', 'tron', 'voie',
             'ctr', 'nit')
    
    lVehTrajCL = []
    for i in CAViter:
        lVehTrajCL.append(dict(zip(keys,i)))
    
    
    n_list = [n]*U_star.shape[0]
    
    CAViter = zip(CAVti, CAVid, CAVtype, CAVtron, 
                  CAVvoie, U_star[0], n_list)
    
    lVehUCL = []
    for i in CAViter:
        lVehUCL.append(dict(zip(keysU,i)))
        
    return lVehTrajCL, lVehUCL


def format_open_loop(results):
    """ Aux function 
        To write in the closed loop database 
        results from open loop without treatment
        (No control applied)
        Homogenizes results in terms of content
    """
    
    keys = ('ti', 'id', 'type', 'tron', 'voie', 
            'dst', 'abs', 'vit', 'ldr', 
            'spc','vld')
    keysU = ('ti', 'id', 'type', 'tron', 'voie',
             'ctr', 'nit')
    
    lVehTrajOL = []
    lVehUOL = []
    for i in results:
        lVehTrajOL.append(dict(zip(keys,i)))
        ti, vid, ty, tr, vo, _, _, _, _, _, _ = i
        u_tup = (ti, vid, ty, tr, vo, 0, 0)
        lVehUOL.append(dict(zip(keysU,u_tup)))
        
    return lVehTrajOL,lVehUOL

