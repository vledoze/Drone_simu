#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 16/05/2017

#Imports
from math import cos, sin
import numpy as np

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
