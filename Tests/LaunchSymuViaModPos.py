
# coding: utf-8

# In[2]:


import sys 
import os
from ctypes import cdll, create_string_buffer, c_int, byref, c_double

dir_path = os.path.dirname(os.path.realpath(__file__))
lib_path_name = ('..','SymuviaModPos','Contents','Frameworks','libSymuVia.dylib')

full_name = os.path.join(dir_path,*lib_path_name)
print(full_name)

symuvialib = cdll.LoadLibrary(full_name)        

if symuvialib is None:
    print('Impossible de charger libSymuVia.dylib !')
    
print('\n Library loaded\n')

# inputfile_path = '/Volumes/Data/Dropbox/Partage_Lafayette/resaux/'
# inputfile_name = 'LafayetteVOpenSource.xml'

# full_inputfile = inputfile_path+inputfile_name
# print(full_inputfile)

# m = symuvialib.SymLoadNetworkEx(full_inputfile.encode('UTF8'))
# print('\n Network loaded\n')

# sFlow = create_string_buffer(10000)
# bEnd = c_int()

# time = range(60)
# for s in time:
    
#     # For all vehicle connected if created:
#         # symuvialib.SymDriveVehiculeEx(Id,sTroncon, nVoie, dPos, bForce)
    
#     bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 
#     print('Instant:',s+1)
    
#     stype = 'VL'
#     sOrigin= 'E_Moliere_S'
#     sDestination = 'S_Moliere_N'
#     nVoie = c_int(1)
#     dbTime = c_double(0.2)
#     nIdVeh = symuvialib.SymCreateVehicleEx(stype,sOrigin.encode('UTF8'),sDestination.encode('UTF8'),nVoie,dbTime)
#     print('Vehicle created', nIdVeh )
    
# dPos = 15
# sLink = 'Rue_Moliere_SN_1'
# nVoie = c_int(1)

# time2 = range(5)

# bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 
# nIdVeh = 59
# nres = symuvialib.SymDriveVehicleEx(nIdVeh,sLink.encode('UTF8'), nVoie, dPos, 1)
# bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 

# print('Drive ',nres )

# for s in time2:
    
#     dPos = dPos + 2
#     nres = symuvialib.SymDriveVehicleEx(nIdVeh,sLink.encode('UTF8'), nVoie, c_double(dPos), 1)
#     print('Drive ', nres )
    
    
#     bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 
    
# for s in time:
    
#     bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 
#     print('Instant:',s+1)
    
# del symuvialib


