#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

class Servo (object):
    """
    """
    
    def __init__(self):
        self._angle   = 0
        self._vitmax  = pi/2 #Vitesse max : fixee a 90deg/s
        self._angmax  = pi

    def set(self, value):
        #value est en pourcentage d'angle max[-100, 100]
        value = value/100.
        angleCmd = self._angmax*value
        diff = angleCmd - self._angle
        if (abs(diff/DT) > self._vitmax):  # On ne peut pas depasser la vitesse max des servos
              diff = np.sign(diff)*self._vitmax*DT
        self._angle = self._angle + diff

    def get_angle(self):
        return self._angle
