#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

class Servo (object):
    """ Classe Servo : Defini le fonctionnement des servomoteurs
    """
    
    def __init__(self):
        """ Constructeur servomoteur
        """
        self.__angle  = 0.
        self.__vitmax = pi/2 #Vitesse max : fixee a 90deg/s
        self.__angmax = pi

    @property
    def angle(self):
        """ Angle du servomoteur 
        """
        return self.__angle
        
    @angle.setter
    def angle(self, value):
        """ Angle commande au servomoteur
        """
        #value est en pourcentage d'angle max[-100, 100]
        value = float(value)/100.
        value = minmax(value, -100., 100.)
        #angle commande
        angleCmd = self.__angmax*value
        diff = angleCmd - self.__angle
        # On ne peut pas depasser la vitesse max des servos
        if (abs(diff/DT) > self.__vitmax): 
              diff = np.sign(diff)*self.__vitmax*DT
        self.__angle = self.__angle + diff
