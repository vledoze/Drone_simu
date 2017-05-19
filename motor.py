#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

class Motor (object):
    def __init__(self, l, e, y, h, angle, sens):
        #Vecteur d'etat
        self.__w = 0.
        #Derivee vecteur d'etat
        self.__dw = 0.
        #Position du moteur
        self.__l = l         #(l+e, y, h): position dans le repere drone a plat
        self.__e = e
        self.__h = h
        self.__pos = np.matrix([[0.], [y], [0.]])
        self.angle_to_pos(angle)
        #Dynamique
        self.__force = 0.
        self.__vforce = np.zeros(3)
        self.__moment = 0.
        self.__vmoment = np.zeros(3)
        #Caracteristiques mecaniques
        self.__inert  = 0.001     #inertie des helices
        self.__sens   = sens      #signe des vitesses de rotation des helices (-1, 1)
        self.__vitmax = 30000*D2R #vitesse max des moteurs (5000tr/min)
        self.__formax = 0.5*G     #force max a 500g de pousse

    def angle_to_pos(self, angle):
        self.__angle  = angle #angle d'inclinaison des moteurs (0: a plat | 90: vertical)
        self.__pos[0] = self.__l + self.__e*cos(self.__angle) - self.__h*sin(self.__angle)
        self.__pos[2] = self.__h*cos(self.__angle) + self.__e*sin(self.__angle)

    def commande(self, value, angle):
        #value est en pourcentage de puissance
        value = value/100.
        #On fait tourner le moteur
        self.__dw = (self.__vitmax*value - self.__w)/DT #Derivee de la vitesse de rotation
        self.__w = self.__vitmax*value #vitesse de rotation
        #Position moteur
        self.angle_to_pos(angle)

    def dyn(self):
        #Bilan mecanique moteur (force)
        self.__force = self.__formax*self.__w/self.__vitmax
        self.__vforce = np.matrix(
            [[self.__force*sin(self.__angle)],
             [0.],
             [-self.__force*cos(self.__angle)]])
        #Bilan mecanique moteur (moment)
        self.__moment = self.__sens*self.__inert*self.__dw
        self.__vmoment = np.matrix(
            [[self.__moment*cos(self.__angle)],
             [0.],
             [self.__moment*sin(self.__angle)]])
        self.__vmoment = np.add(
            self.__vmoment,
            np.cross(-self.__pos, self.__vforce, axis=0))

    def get_sens(self):
        return self.__sens

    def get_vforce(self):
        return self.__vforce

    def get_vmoment(self):
        return self.__vmoment

    def get_pos(self):
        return self.__pos

if __name__ == "__main__":
    print "Moteur"
