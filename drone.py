#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 16/05/2017

from util_maths import matrix, linalg
from util_maths.matrices_rotation import *
from util_simu.flyobject import FlyObject
from util_simu.objetsimu import ObjetSimu
from environnement import G
from actionneurs.mixer import Mixer
from aero.wing import Wing

class Drone (FlyObject):
    """ Repere drone
        O: Centre
        Ox: vers l'avant
        Oy: vers la droite
        Oz: vers le bas
    """

    def __init__(self, angle):
        #Donnees physique du drone
        self._posg_d  = matrix('0.; 0.; 0.')   #Position du centre de gravite drone dans le repere drone
        #Motorisation du drone
        self._mixer = Mixer(angle)                           #Ensemble des servos+moteurs embarques
        #Aile(s) du drone
        self._wing = Wing(0.25, 0.12, matrix('0.05; 0.; 0.'), inclinaison=np.radians(5.0))
        #Initialisation objet FlyObject
        FlyObject.__init__(self)

    def mecanique(self):
        """ Equation de la dynamique pour le drone
        """
        #Sommes des forces et moments lies aux moteurs
        self._sum_forces = self._mixer.vforce.copy()
        self._sum_moments = self._mixer.vmoment.copy()
        #Sommes des forces et moments lies aux ailes
        self._wing.apply(self._vit_m)
        self._sum_forces += self._wing.vforce
        self._sum_moments += self._wing.vmoment
        #Influence de la gravite
        forceg_i = matrix("0.; 0.; 0.")
        forceg_i[2] = self._masse*G
        forceg_d = np.dot(self._Rmsi, forceg_i)
        #Somme des forces et moments
        self._sum_forces += forceg_d
        self._sum_moments += np.cross(self._posg_d, forceg_d, axis=0)

    def commande(self, throttle_co, roulis_co, gite_co, cap_co, angle_co ):
        self._mixer.commande(throttle_co, roulis_co, gite_co, cap_co, angle_co)

    def run(self):
        self.cinematique()
        self.save()

    def set_X(self, X):
        """ Recuperation du vecteur d'etat """
        self._pos_i[0] = X[0]
        self._pos_i[1] = X[1]
        self._pos_i[2] = X[2]
        self._att_i[0] = X[3]
        self._att_i[1] = X[4]
        self._att_i[2] = X[5]
        self._vit_m[0] = X[6]
        self._vit_m[1] = X[7]
        self._vit_m[2] = X[8]
        self._omg_m[0] = X[9]
        self._omg_m[1] = X[10]
        self._omg_m[2] = X[11]

    def get_X(self):
        """ Mise à jour du vecteur d'etat """
        X = []
        for pos in self._pos_i:
            X.append(pos)
        for att in self._att_i:
            X.append(att)
        for vit in self._vit_m:
            X.append(vit)
        for omg in self._omg_m:
            X.append(omg)
        return X

    def get_dX(self):
        """ Mise a jour de la derivee du vecteur d'etat """
        dX = []
        for vit in self._vit_i:
            dX.append(vit)
        for omg in self._omg_i:
            dX.append(omg)
        for accl in self._accl_m:
            dX.append(accl)
        for accr in self._accr_m:
            dX.append(accr)
        return dX
