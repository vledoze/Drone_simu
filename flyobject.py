#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *

class FlyObject(object):
    """ Object carracterisant une mecanique du vol
        Ri : Repere fixe - inertiel
        Rm : Repere mobile 
    """
    
    def __init__(self):
        #Vecteur d'etat
        self._pos_i = np.matrix('0. ; 0.; 0.') #Position dans le repere inertiel (x, y, z)
        self._att_i = np.matrix('0. ; 0.; 0.') #Attitude dans le repere inertiel (psi, teta, phi)
        self._vit_m = np.matrix('0. ; 0.; 0.') #Vitesse dans le repere mobile    (u, v, w)
        self._omg_m = np.matrix('0. ; 0.; 0.') #Vitesse de rotation dans le repere mobile (p, q, r)
        #Derivee vecteur d'etat
        self._vit_i = np.matrix('0. ; 0.; 0.') #Vitesse dans le repere interiel (xp, yp, zp)
        self._omg_i = np.matrix('0. ; 0.; 0.') #Vitesse de rotation dans le repere inertiel (psip, tetap, phip)
        self._accl_m = np.matrix('0. ; 0.; 0.') #Acceleration lineaire dans le repere mobile (up, vp, wp)
        self._accr_m = np.matrix('0. ; 0.; 0.') #Acceleration rotation dans le repere mobile (pp, qp, rp)
        #matrices de rotations
        self.maj_matrices()
        #Donnees physique
        self._masse   = 1.0                         #masse du drone
        self._inertie = np.matrix(np.eye(3, 3))     #matrice d'inertie du drone
        #Dynamique
        self._sum_forces = np.matrix('0. ; 0.; 0.')  #Somme des forces
        self._sum_moments = np.matrix('0. ; 0.; 0.') #Somme des moments

    def maj_matrices(self):
        self._Rmsi = rot_matrix(self._att_i.A1)
        self._Rism = self._Rmsi.transpose()
        self._TRmsi = trot_matrix(self._att_i.A1)
        self._TRism = self._TRmsi.transpose()
    
    def mecanique(self):
        self._sum_forces = np.matrix('0.; 0.; 0.')  #Somme des forces
        self._sum_moments = np.matrix('0.; 0.; 0.') #Somme des moments
    
    def dynamique(self):
        """ Equations de la dynamique
        """
        # Bilan mecanique
        self.mecanique()
        # Somme des forces
        self._accl_m = self._sum_forces/self._masse - np.cross(self._omg_m, self._vit_m, axis=0)
        if self._pos_i[2] > 0:
            self._accl_m[2] = 0.0
        # Somme des moments
        mcin = self._inertie*self._omg_m
        dmcin = self._sum_moments - np.cross(self._omg_m, mcin, axis=0)
        self._accr_m = np.linalg.pinv(self._inertie)*dmcin
        
    def cinematique(self):
        """ Cinematique du drone
        """
        #Mise a jour des matrices de rotation
        self.maj_matrices()
        #Vitesses dans le repere inertiel
        self._vit_i  = np.dot(self._Rism, self._vit_m)
        self._omg_i  = np.dot(self._TRism, self._omg_m)
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
