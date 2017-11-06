#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 03/08/2017

#Infos
__version__ = "1.0"
__author__  = "vledoze"
__date__    = "03/08/2017"

#Importations
from scipy.integrate import ode

#Constantes de temps en milli-secondes
DT = 1        #Pas de temps
T = 0         #Instant courant
TDEB = 0      #Instant de debut de simulation
TFIN = 1000   #fin simu

