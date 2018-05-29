#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys 
import os
from ctypes import cdll, create_string_buffer, c_int, byref
from xmltodict import parse

dir_path = os.path.dirname(os.path.realpath(__file__))
lib_path_name = ('..','Symuvia','Contents','Frameworks','libSymuVia.dylib')

dir_path 
full_name = os.path.join(dir_path,*lib_path_name)
# print(full_name)

print('Library folder {}'.format(full_name))

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

time = range(800)
for s in time:
    # For all vehicle connected if created:
        # symuvialib.SymDriveVehiculeEx(Id,sTroncon, nVoie, dPos, bForce)
    bResult = symuvialib.SymRunNextStepEx(sFlow, 1, byref(bEnd))
    dParsed = parse(sFlow.value.decode('UTF8'))
    ti = dParsed['INST']['@val']
    print(ti, bResult)
    # print(f'bResult: {bResult}')
    # print(f'OutputNet: {sFlow.value}')

print('\n I finished simulating \n')
# r = symuvialib.SymRunEx(file_name.encode('UTF8'))
# print(r)


# root = tkinter.Tk()
# root.withdraw() # pour cacher la fenêtre parent Tk

# options = {'filetypes' : [('Fichiers XML','.xml')], 'title' : 'Sélection du fichier SymuVia à simuler'}
# path = askopenfilename(**options)

# bOk = symuvialib.SymRunEx(path) == 0

# if bOk:
#     print('Simulation exécutée avec succès.')
# else:
#     print('Echec de la simulation !')