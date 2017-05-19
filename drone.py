#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *
from mixer import Mixer

class Drone (object):
    """ Repere drone (O: Centre, Ox: vers l'avant, Oy: vers la gauche )
    """

    def __init__(self):
        #Vecteur d'etat
        self.__pos_i   = np.matrix([[0. ], [0. ], [0.]]) #Position dans le repere inertiel (x, y, z)
        self.__att_i   = np.matrix([[0. ], [0. ], [0.]]) #Attitude dans le repere inertiel (psi, teta, phi)
        self.__vit_d   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse dans le repere drone    (u, v, w)
        self.__omg_d   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse de rotation dans le repere drone (p, q, r)
        #Derivee vecteur d'etat
        self.__vit_i   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse dans le repere interiel (xp, yp, zp)
        self.__omg_i   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse de rotation dans le repere inertiel (psip, tetap, phip)
        self.__accl_d  = np.matrix([[0. ], [0. ], [0.]]) #Acceleration lineaire dans le repere drone (up, vp, wp)
        self.__accr_d  = np.matrix([[0. ], [0. ], [0.]]) #Acceleration rotation dans le repere drone (pp, qp, rp)
        #matrices de rotations du drone
        self.maj_matrices()
        #Donnees physique du drone
        self.__posg_d  = np.matrix([[0.], [0.], [0.]])   #Position du centre de gravite drone dans le repere drone
        self.__masse   = 1.0                             #masse du drone
        self.__inertie = np.matrix(np.eyes(3, 3))        #matrice d'inertie du drone
        #Motorisation du drone
        self.__mixer = Mixer()                           #Ensemble des servos+moteurs embarques

    def maj_matrices(self):
        self.__Rdsi = rot_matrix(self.__att_i.A1)
        self.__Risd = self.__Rdsi.transpose()
        self.__TRdsi = trot_matrix(self.__att_i.A1)
        self.__TRdsi = self.__TRdsi.transpose()

    def dyn_drone(self):
        """ Equation de la dynamique pour le drone
        """
        #Influence de la gravite
        forceg_i = np.matrix(np.zeros([3, 1]))
        forceg_i[2] = self.__masse*G
        forceg_d = np.dot(self.__Rdsi, forceg_i)
        #Sommes des forces et moments dans le repere drone
        sumForce_d  = np.matrix(np.zeros([3, 1]))
        sumMoment_d = np.matrix(np.zeros([3, 1]))
        for mot in self.__mixer.get_motor():
            mot.dyn()
            sumForce_d = np.add(sumForce_d, mot.get_vforce())
            sumMoment_d = np.add(sumMoment_d, mot.get_vmoment())
        sumForce_d = np.add(sumForce_d, forceg_d)
        sumMoment_d = np.add(sumMoment_d, np.cross(-self.__posg_d, forceg_d, axis=0))
        #Acceleration dans le repere drone
        self.__accl_d = sumForce_d/self.__masse
        self.__accr_d = sumMoment_d/self.__inertie

    def cin_drone(self):
        """ Cinematique du drone
        """
        #Mise a jour des matrices de rotation
        self.maj_matrices()
        #Vitesses dans le repere inertiel
        self.__vit_i  = np.dot(self.__Risd, self.__vit_d)
        self.__omg_i  = np.dot(self.__Risd, self.__omg_d)
        #Equation de la dynamique
        self.dyn_drone()

    def commande(self, throttle_co, roulis_co, gite_co, cap_co, angle_co ):
        self.__mixer.commande(throttle_co, roulis_co, gite_co, cap_co, angle_co)

    def run(self):
        self.cin_drone()

    def set_X(self, X):
        self.__pos_i[0] = X[0]
        self.__pos_i[1] = X[1]
        self.__pos_i[2] = X[2]
        self.__att_i[0] = X[3]
        self.__att_i[1] = X[4]
        self.__att_i[2] = X[5]
        self.__vit_d[0] = X[6]
        self.__vit_d[1] = X[7]
        self.__vit_d[2] = X[8]
        self.__omg_d[0] = X[9]
        self.__omg_d[1] = X[10]
        self.__omg_d[2] = X[11]

    def get_X(self):
        X = []
        for i in range(len(self.__pos_i)):
            X.append(self.__pos_i.A1[i])
        for i in range(len(self.__att_i)):
            X.append(self.__att_i.A1[i])
        for i in range(len(self.__vit_d)):
            X.append(self.__vit_d.A1[i])
        for i in range(len(self.__omg_d)):
            X.append(self.__omg_d.A1[i])
        return X

    def get_dX(self):
        dX = []
        self.run()
        for i in range(len(self.__pos_i)):
            dX.append(self.__vit_i.A1[i])
        for i in range(len(self.__att_i)):
            dX.append(self.__omg_i.A1[i])
        for i in range(len(self.__vit_d)):
            dX.append(self.__accl_d.A1[i])
        for i in range(len(self.__omg_d)):
            dX.append(self.__accr_d.A1[i])
        return dX

    def get_pos_i(self):
        return self.__pos_i

    def get_att_i(self):
        return self.__att_i

    def get_vit_d(self):
        return self.__vit_d

    def get_omg_d(self):
        return self.__omg_d

    def get_vit_i(self):
        return self.__vit_i

    def get_omg_i(self):
        return self.__omg_i

    def get_accl_d(self):
        return self.__accl_d

    def get_accr_d(self):
        return self.__accr_d

    def get_Rdsi(self):
        self.maj_matrices()
        return self.__Rdsi

    def get_Risd(self):
        self.maj_matrices()
        return self.__Risd

    def get_mixer(self):
        return self.__mixer
