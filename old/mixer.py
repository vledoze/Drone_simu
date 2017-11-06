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
        #Ensemble des servomoteurs
        self.__servos = {
            "cap":Servo(),
            "roulis":Servo(),
            "gite":Servo(),
            "config":Servo()}
        #Ensemble des moteurs
        angle = self.__servos["config"].angle
        self.__motors = {
            "moteur_av_dr":Motor( 0.0, 0.1,  0.3, 0.025, angle, 1, -1, 1),
            "moteur_av_gch":Motor( 0.0, 0.1, -0.3, 0.025, angle, -1, -1, -1),
            "moteur_ar_dr":Motor(-0.2, 0.1,  0.3, 0.025, angle, 1, 1, -1),
            "moteur_ar_gch":Motor(-0.2, 0.1, -0.3, 0.025, angle, -1, 1, 1)}

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
        sum_vforce = np.matrix("0.; 0.; 0.")
        for mot in self.__motors.values():
            sum_vforce += mot.vforce
        return sum_vforce

    @property
    def vmoment(self):
        sum_vmoment = np.matrix("0.; 0.; 0.")
        for mot in self.__motors.values():
            sum_vmoment += mot.vmoment
        return sum_vmoment
