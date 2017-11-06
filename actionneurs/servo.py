#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 16/05/2017

#Importations
from util_maths import math, sign
from util_maths.minmax import minmax
from util_simu import DT
from util_simu.objetsimu import ObjetSimu

#Classe Servomoteur
class Servo(ObjetSimu):
    """ Classe Servo : Defini le fonctionnement des servomoteurs
    """

    def __init__(self, angle=0.0):
        """ Constructeur servomoteur
        """
        #Attributs
        self.__angle  = angle
        self.__vitmax = math.pi/2 #Vitesse max : fixee a 90deg/s
        #Initialisation de l'objet simu
        ObjetSimu.__init__(self)

    @property
    def angle(self):
        """ Angle du servomoteur
        """
        return self.__angle

    @angle.setter
    def angle(self, angle_co):
        """ Angle commande au servomoteur
        """
        angle_co = minmax(angle_co, -math.pi/2.0, math.pi/2.0)
        #angle commande
        diff =  angle_co - self.__angle
        # On ne peut pas depasser la vitesse max des servos
        if (abs(diff/(float(DT)/1000)) > self.__vitmax):
              diff = sign(diff)*self.__vitmax*(float(DT)/1000)
        self.__angle += diff
