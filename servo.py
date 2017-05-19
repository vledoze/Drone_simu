#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

class Servo (object):
    def __init__(self):
        self.__angle   = 0
        self.__vitmax  = pi/2 #Vitesse max : fixee a 90deg/s
        self.__angmax  = pi

    def set(self, value):
        #value est en pourcentage d'angle max[-100, 100]
        value = value/100.
        angleCmd = self.__angmax*value
        diff = angleCmd - self.__angle
        if (abs(diff/DT) > self.__vitmax):  # On ne peut pas depasser la vitesse max des servos
              diff = np.sign(diff)*self.__vitmax*DT
        self.__angle = self.__angle + diff

    def get_angle(self):
        return self.__angle

if __name__ == "__main__":
    print "Servo"
