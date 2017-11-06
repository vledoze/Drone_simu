#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 16/05/2017

from util_maths import matrix, cross, linalg, dot
from util_maths.matrices_rotation import *
from objetsimu import ObjetSimu

class FlyObject(ObjetSimu):
    """ Object carracterisant une mecanique du vol
        Ri : Repere fixe - inertiel
        Rm : Repere mobile
    """

    def __init__(self):
        #Vecteur d'etat
        self._pos_i = matrix('0. ; 0.; 0.') #Position dans le repere inertiel (x, y, z)
        self._att_i = matrix('0. ; 0.; 0.') #Attitude dans le repere inertiel (psi, teta, phi)
        self._vit_m = matrix('0. ; 0.; 0.') #Vitesse dans le repere mobile    (u, v, w)
        self._omg_m = matrix('0. ; 0.; 0.') #Vitesse de rotation dans le repere mobile (p, q, r)
        #Derivee vecteur d'etat
        self._vit_i = matrix('0. ; 0.; 0.') #Vitesse dans le repere interiel (xp, yp, zp)
        self._omg_i = matrix('0. ; 0.; 0.') #Vitesse de rotation dans le repere inertiel (psip, tetap, phip)
        self._accl_m = matrix('0. ; 0.; 0.') #Acceleration lineaire dans le repere mobile (up, vp, wp)
        self._accr_m = matrix('0. ; 0.; 0.') #Acceleration rotation dans le repere mobile (pp, qp, rp)
        #matrices de rotations
        self.maj_matrices()
        #Donnees physique
        self._masse   = 1.0                         #masse du drone
        self._inertie = matrix(np.eye(3, 3))     #matrice d'inertie du drone
        #Dynamique
        self._sum_forces = matrix('0. ; 0.; 0.')  #Somme des forces
        self._sum_moments = matrix('0. ; 0.; 0.') #Somme des moments
        #Initialisation de l'objet simu
        ObjetSimu.__init__(self)

    def maj_matrices(self):
        self._Rmsi = rot_matrix(self._att_i.A1)
        self._Rism = self._Rmsi.transpose()
        self._TRmsi = trot_matrix(self._att_i.A1)
        self._TRism = self._TRmsi.transpose()

    def mecanique(self):
        self._sum_forces = matrix('0.; 0.; 0.')  #Somme des forces
        self._sum_moments = matrix('0.; 0.; 0.') #Somme des moments

    def dynamique(self):
        """ Equations de la dynamique
        """
        # Bilan mecanique
        self.mecanique()
        # Somme des forces
        self._accl_m = self._sum_forces/self._masse - np.cross(self._omg_m, self._vit_m, axis=0)
        # Si on touche le sol
        if self._pos_i[2] >= 0:
            accl_m = self._accl_m.copy()
            accl_i = dot(self._Rism, accl_m)
            accl_i[2] = min(0.0, accl_i[2])
            self._accl_m = dot(self._Rmsi, accl_i)
        # Somme des moments
        mcin = self._inertie*self._omg_m
        dmcin = self._sum_moments - cross(self._omg_m, mcin, axis=0)
        self._accr_m = linalg.pinv(self._inertie)*dmcin

    def cinematique(self):
        """ Cinematique du drone
        """
        #Mise a jour des matrices de rotation
        self.maj_matrices()
        #Vitesses dans le repere inertiel
        self._vit_i  = dot(self._Rism, self._vit_m)
        self._omg_i  = dot(self._TRism, self._omg_m)
        #Si on touche le sol
        if self._pos_i[2] >= 0:
            self._vit_i[2] = min(0.0, self._vit_i[2])
            self._vit_m = dot(self._Rmsi, self._vit_i)
        #Equation de la dynamique
        self.dynamique()

    def get_pos_i(self):
        return self._pos_i

    def get_att_i(self):
        return self._att_i

    def get_vit_m(self):
        return self._vit_m

    def get_omg_m(self):
        return self._omg_m

    def get_vit_i(self):
        return self._vit_i

    def get_omg_i(self):
        return self._omg_i

    def get_accl_m(self):
        return self._accl_m

    def get_accr_m(self):
        return self._accr_m

    def get_Rmsi(self):
        self.maj_matrices()
        return self._Rmsi

    def get_Rism(self):
        self.maj_matrices()
        return self._Rism

    def get_TRmsi(self):
        self.maj_matrices()
        return self._TRmsi

    def get_TRism(self):
        self.maj_matrices()
        return self._TRism

    def get_masse(self):
        return self._masse

    def get_inertie(self):
        return self._inertie

    def get_sum_forces(self):
        return self._sum_forces

    def get_sum_moments(self):
        return self._sum_moments
