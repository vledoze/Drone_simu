#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

from motor import Motor
from servo import Servo

class Mixer(object):
    """ Ensemble moteurs + Servo-moteur
    """

    def __init__(self):
        #Vecteur d'etat
        self._trottle = 0.     #Gaz
        self._roulis = 0.      #Angle de Roulis
        self._gite = 0.        #Angle de Gite
        self._cap = 0.         #Angle de Cap
        self._angle = 0.       #Angle de configuration (0: a plat | 90: vertical)
        #Ensemble des moteurs
        self._motors = []
        self._motors.append(Motor( 0.0, 0.1,  0.3, 0.025, self._angle,  1))
        self._motors.append(Motor( 0.0, 0.1, -0.3, 0.025, self._angle, -1))
        self._motors.append(Motor(-0.2, 0.1,  0.3, 0.025, self._angle,  1))
        self._motors.append(Motor(-0.2, 0.1, -0.3, 0.025, self._angle, -1))
        #Ensemble des servomoteurs
        self._servos = []
        for ii in range(4):
            self._servos.append(Servo())  #1: Cap, 2+3: Roulis, 4:Angle de config

    def commande(self, trottle_co, roulis_co, gite_co, cap_co, angle_co):
        """ Commandes envoyees a l'ensemble moteur + servos
        """
        #les valeurs commandee sont en pourcentage de capacite max
        angle = self._servos[3].get_angle()
        if (angle < 0.088): #Environ 5deg
            #Rien est pilote avec les servos
            for servo in self._servos:
                servo.set(0.0)
            #Tout est pilote avec les moteurs
            value = [
                (+roulis_co - gite_co),
                (-roulis_co - gite_co),
                (-roulis_co + gite_co),
                (+roulis_co + gite_co)]
            for val, mot in zip(value, self._motors):
                val = (val + trottle_co*10 + mot.get_sens()*cap_co)/14
                if val < 0.0 :
                    val = 0.0
                elif val > 100.0:
                    val = 100.0
                mot.commande(val, angle)
        else:
            #Cap pilote avec servo 1
            self._servos[0].set(cap_co)
            #Roulis commande avec servos 2 et 3
            self._servos[1].set(roulis_co)
            #Vitesse de rotation des moteurs fixe la vitesse demandee
            for mot in self._motors:
                mot.commande(trottle_co, angle)
        #Angle de configuration commande
        self._servos[3].set(angle_co)

    def get_motor(self):
        return self._motors
