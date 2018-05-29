
# coding: utf-8

# In[2]:


import sys 
import os
from ctypes import cdll, create_string_buffer, c_int, byref, c_double
from xmltodict import parse

dir_path = os.path.dirname(os.path.realpath(__file__))
lib_path_name = ('..','SymuviaModPos','Contents','Frameworks','libSymuVia.dylib')

full_name = os.path.join(dir_path,*lib_path_name)
print(full_name)

symuvialib = cdll.LoadLibrary(full_name)        

if symuvialib is None:
    print('Impossible de charger libSymuVia.dylib !')
    
print('\n Library loaded\n')

file_path_name = ('..','Network','Merge.xml')
file_name = os.path.join(dir_path,*file_path_name)
print(file_name)
m = symuvialib.SymLoadNetworkEx(file_name.encode('UTF8'))
print(f'Network loaded {m}')

sFlow = create_string_buffer(10000)
print(f'Value before {sFlow.value}')
bEnd = c_int()

# Run 5 steps 

time = range(5)
for s in time:
    
    # For all vehicle connected if created:
    # Drive vehicle 
   # symuvialib.SymDriveVehiculeEx(Id,sTroncon, nVoie, dPos, bForce)

    bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 
    dParsed = parse(sFlow.value.decode('UTF8'))
    print('Instant:',s+1)


    print(f'bResult: {bResult}')    
    print(f'OutputNet: {sFlow.value}')        
    ti = dParsed['INST']['@val']
    print(ti, bResult)

# Create vehicle at time step 0.5
stype = 'CAV'
sOrigin= 'Ext_In_main'
sDestination = 'Ext_Out_main'
nVoie = c_int(1)
dbTime = c_double(0.05)
nIdVeh = symuvialib.SymCreateVehicleEx(stype.encode('UTF8'), sOrigin.encode('UTF8'), sDestination.encode('UTF8'), nVoie, dbTime)
print('Vehicle created', nIdVeh)

# Run time step 

bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 
dParsed = parse(sFlow.value.decode('UTF8'))
print(f'bResult: {bResult}')    
print(f'OutputNet: {sFlow.value}')  
ti = dParsed['INST']['@val']
print(ti, bResult)   

# Drive vehicle @ 0.5

sTroncon = 'In_main'.encode('UTF8')
nVoie = c_int(1)
dPos = c_double(15.0)
Id = c_int(nIdVeh)
bForce = c_int(1)
print(sTroncon,nVoie,dPos,Id, bForce)

nres = symuvialib.SymDriveVehicleEx(Id, sTroncon, nVoie, dPos, bForce)


bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd)) 
dParsed = parse(sFlow.value.decode('UTF8'))
print(f'bResult: {bResult}')    
print(f'OutputNet: {sFlow.value}')  
ti = dParsed['INST']['@val']
print(ti, bResult)   


print('Drive ',nres )
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


