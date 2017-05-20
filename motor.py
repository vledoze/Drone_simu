#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

class Motor(object):
    """
    """
    
    def __init__(self, l, e, y, h, angle, sens):
        #Vecteur d'etat
        self._w = 0.
        #Derivee vecteur d'etat
        self._dw = 0.
        #Position du moteur
        self._l = l         #(l+e, y, h): position dans le repere drone a plat
        self._e = e
        self._h = h
        self._pos = np.matrix([[0.], [y], [0.]])
        self.angle_to_pos(angle)
        #Dynamique
        self._force = 0.
        self._vforce = np.zeros(3)
        self._moment = 0.
        self._vmoment = np.zeros(3)
        #Caracteristiques mecaniques
        self._inert  = 0.001     #inertie des helices
        self._sens   = sens      #signe des vitesses de rotation des helices (-1, 1)
        self._vitmax = 30000*D2R #vitesse max des moteurs (5000tr/min)
        self._formax = 0.5*G     #force max a 500g de pousse

    def angle_to_pos(self, angle):
        self._angle  = angle #angle d'inclinaison des moteurs (0: a plat | 90: vertical)
        self._pos[0] = self._l + self._e*cos(self._angle) - self._h*sin(self._angle)
        self._pos[2] = self._h*cos(self._angle) + self._e*sin(self._angle)

    def commande(self, value, angle):
        #value est en pourcentage de puissance
        value = value/100.
        #On fait tourner le moteur
        self._dw = (self._vitmax*value - self._w)/DT #Derivee de la vitesse de rotation
        self._w = self._vitmax*value #vitesse de rotation
        #Position moteur
        self.angle_to_pos(angle)

    def mecanique(self):
        #Bilan mecanique moteur (force)
        self._force = self._formax*self._w/self._vitmax
        self._vforce = np.matrix(
            [[self._force*sin(self._angle)],
             [0.],
             [-self._force*cos(self._angle)]])
        #Bilan mecanique moteur (moment)
        self._moment = self._sens*self._inert*self._dw
        self._vmoment = np.matrix(
            [[self._moment*cos(self._angle)],
             [0.],
             [self._moment*sin(self._angle)]])
        self._vmoment = np.add(
            self._vmoment,
            np.cross(-self._pos, self._vforce, axis=0))

    def get_sens(self):
        return self._sens

    def get_vforce(self):
        return self._vforce

    def get_vmoment(self):
        return self._vmoment

    def get_pos(self):
        return self._pos
