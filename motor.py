#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

class Motor(object):
    """ Classe Motor : Definition d'un moteur
    """
    
    def __init__(self, l, e, y, h, angle, sens_cap, sens_gite, sens_roulis):
        """ Constructeur
        """
        #Vecteur d'etat
        self.__w = 0.
        #Derivee vecteur d'etat
        self.__dw = 0.
        #Position du moteur
        self.__l = l         #(l+e, y, h): position dans le repere drone a plat
        self.__e = e
        self.__y = y
        self.__h = h
        self.__pos = np.matrix('0; 0; 0')
        self.angle = angle
        #Dynamique
        self.__force = 0.
        self.__vforce = np.zeros(3)
        self.__moment = 0.
        self.__vmoment = np.zeros(3)
        #Caracteristiques mecaniques
        self.__inert  = 0.001     #inertie des helices
        self.__vitmax = 30000*D2R #vitesse max des moteurs (5000tr/min)
        self.__formax = 0.5*G     #force max a 500g de pousse
        #sens de rotation
        self.__sens_cap = sens_cap       #signe des vitesses de rotation des helices pour prise en compte du cap (-1, 1)
        self.__sens_gite = sens_gite     #signe pour prise en compte du gite
        self.__sens_roulis = sens_roulis #signe pour prise en compte du roulis
    
    @property
    def angle(self):
        """ Angle courant du bras moteur 
        """
        return self.__angle
        
    @angle.setter
    def angle(self, angle):
        """ Angle commande pour le bras moteur 
        """
        #angle d'inclinaison des moteurs (0: a plat | 90: vertical)
        self.__angle  = minmax(angle, 0., 90.)
        self.__pos[0] = self.__l + self.__e*cos(self.__angle) - self.__h*sin(self.__angle)
        self.__pos[1] = self.__y
        self.__pos[2] = self.__h*cos(self.__angle) + self.__e*sin(self.__angle)

    @property
    def w(self):
        """ Vitesse de rotation du moteur 
        """
        return self.__w
    
    @w.setter
    def w(self, value):
        """ Commande en vitesse en % de vitesse max
        """
        #value est en pourcentage de puissance
        value = float(value)/100.
        minmax(value, 0., 100.)
        #On fait tourner le moteur
        self.__dw = (self.__vitmax*value - self.__w)/DT #Derivee de la vitesse de rotation
        self.__w = self.__vitmax*value #vitesse de rotation
    
    @property
    def sens_cap(self):
        return self.__sens_cap
    
    @property 
    def sens_roulis(self):
        return self.__sens_roulis
    
    @property
    def sens_gite(self):
        return self.__sens_gite
    
    @property
    def vforce(self):
        """ Force generee par le moteur
        """
        #Bilan mecanique moteur (force)
        self.__force = self.__formax*self.__w/self.__vitmax
        self.__vforce = np.matrix(
            [[self.__force*sin(self.__angle)],
             [0.],
             [-self.__force*cos(self.__angle)]])
        return self.__vforce
    
    @property
    def vmoment(self):
        """ Moment generee par le moteur 
        """
        #Bilan mecanique moteur (moment)
        self.__moment = self.__sens_cap*self.__inert*self.__dw
        self.__vmoment = np.matrix(
            [[self.__moment*cos(self.__angle)],
             [0.],
             [self.__moment*sin(self.__angle)]])
        self.__vmoment += np.cross(self.__pos, self.__vforce, axis=0)
        return self.__vmoment
        
