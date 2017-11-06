#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 03/08/2017

#Infos
__version__ = "1.0"
__author__  = "vledoze"
__date__    = "03/08/2017"

#Importations
import math
from numpy import matrix, cross, linalg, dot, sign
from scipy.interpolate import interp1d

#Constantes angulaires
D2R = math.pi/180.
R2D = 180./math.pi
