#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 16/05/2017

#Importations
import numpy as np

from util_simu.objetsimu import ObjetSimu
from motor import Motor
from servo import Servo

class Mixer(ObjetSimu):
    """ Ensemble moteurs + Servo-moteur
    """

    def __init__(self, angle):
        #Ensemble des servomoteurs
        self.__servos = {
            "cap":Servo(),
            "roulis":Servo(),
            "gite":Servo(),
            "config":Servo(angle)}
        #Ensemble des moteurs
        self.__moteur_av_dr = Motor( 0.0, 0.1,  0.3, 0.025, angle, 1, -1, 1)
        self.__moteur_av_gch = Motor( 0.0, 0.1, -0.3, 0.025, angle, -1, -1, -1)
        self.__moteur_ar_dr = Motor(-0.2, 0.1,  0.3, 0.025, angle, 1, 1, -1)
        self.__moteur_ar_gch = Motor(-0.2, 0.1, -0.3, 0.025, angle, -1, 1, 1)
        self.__motors = {
            "moteur_av_dr":self.__moteur_av_dr,
            "moteur_av_gch":self.__moteur_av_gch,
            "moteur_ar_dr":self.__moteur_ar_dr,
            "moteur_ar_gch":self.__moteur_ar_gch}
        #Ensemble des forces et moments delivré par la chaine propulsive
        self.__sum_vforce = np.matrix("0.; 0.; 0.")
        self.__sum_vmoment = np.matrix("0.; 0.; 0.")
        #Initialisation de l'objet simu
        ObjetSimu.__init__(self)

    def commande(self, trottle_co, roulis_co, gite_co, cap_co, angle_co):
        """ Commandes envoyees a l'ensemble moteur + servos
        """
        #commande des servo
        self.__servos["cap"].angle = cap_co
        self.__servos["roulis"].angle = roulis_co
        self.__servos["gite"].angle = gite_co
        self.__servos["config"].angle = angle_co
        #commande des moteurs
        for mot in self.__motors.values():
            mot.angle = self.__servos["config"].angle
            mot.w = (trottle_co
                   + mot.sens_cap*cap_co
                   + mot.sens_gite*gite_co
                   + mot.sens_roulis*roulis_co)

    @property
    def vforce(self):
        self.__sum_vforce = np.matrix("0.; 0.; 0.")
        for mot in self.__motors.values():
            self.__sum_vforce += mot.vforce
        return self.__sum_vforce

    @property
    def vmoment(self):
        self.__sum_vmoment = np.matrix("0.; 0.; 0.")
        for mot in self.__motors.values():
            self.__sum_vmoment += mot.vmoment
        return self.__sum_vmoment
