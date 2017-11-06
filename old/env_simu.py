#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from math import *
from scipy.integrate import ode
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import pdb

#Variables locales
DT = 0.01  #Pas de temps
T = 0.     #Instant courant
TFIN = 10. #fin simu
G  = 9.81  #gravite

#Constantes angulaires
D2R = pi/180.
R2D = 180./pi

def rot_matrix(att):
    """ Matrice de rotation a partir d'angles d'euler
    """
    psi = att[0]
    teta= att[1]
    phi = att[2]
    #Matrice de rotation
    cpsi = cos(psi)
    spsi = sin(psi)
    ctet = cos(teta)
    stet = sin(teta)
    cphi = cos(phi)
    sphi = sin(phi)
    r1 = np.array([cpsi*ctet,                 spsi*ctet,               -stet])
    r2 = np.array([-spsi*cphi+sphi*cpsi*stet, cpsi*cphi+sphi*spsi*stet, sphi*ctet])
    r3 = np.array([spsi*sphi+cphi*cpsi*stet, -cpsi*sphi+cphi*spsi*stet, cphi*ctet])
    return np.matrix([r1, r2, r3])

def trot_matrix(att):
    """ Matrice de transfert de rotation a partir d'angles d'euler
    """
    teta= att[1]
    phi = att[2]
    #Matrice de transfert de rotation
    ctet = cos(teta)
    stet = sin(teta)
    cphi = cos(phi)
    sphi = sin(phi)
    r1 = np.array([1.0,  0.0, -stet])
    r2 = np.array([0.0,  cphi, sphi*ctet])
    r3 = np.array([0.0, -sphi, cphi*ctet])
    return np.matrix([r1, r2, r3])
   
def minmax(val, minval, maxval):
    return min(maxval, max(minval, val))
