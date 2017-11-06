#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 01/08/2017

#Importation
from util_maths import R2D
from util_maths import math, interp1d, matrix, dot, cross, linalg, sign
from util_maths.matrices_rotation import *
from util_simu.objetsimu import ObjetSimu

from aero import RHO

class Wing(ObjetSimu):
    """ Definition d'un objet aile """

    def __init__(self, corde, largeur, pos_foyer_aero, inclinaison=0.0, file_name="aero/default_wing.dat"):
        """ Initialisation """
        self.__surface = corde*largeur
        self.__pos = pos_foyer_aero
        self.__inclinaison = inclinaison
        self.__vforce = matrix("0.;0.;0.")
        self.__vmoment = matrix("0.;0.;0.")
        self.load(file_name)
        self.init_interpolation()
        self.__alpha = 0.0
        self.__beta = 0.0
        self.interpolation()
        #TODO manque position foyer aero
        #Initialisation de l'objet simu
        ObjetSimu.__init__(self)

    def load(self, file_name):
        """ Chargement des caracteristiques de l'aile a partir d'un fichier texte """
        #Creation des listes de coef et incidences
        self.__tab_cl = []
        self.__tab_cd = []
        self.__tab_cm = []
        self.__tab_alpha = []
        #Fichier de donnees a lire
        f = open(file_name, 'r')
        #Recuperation des coefficients et incidences
        for l in f:
            ligne = [float(n) for n in l.split() ]
            self.__tab_cl.append(ligne[1])
            self.__tab_cd.append(ligne[2])
            self.__tab_cm.append(ligne[4])
            self.__tab_alpha.append(ligne[0])
        #Pour les angles à 90 et -90 on assimile l'aile a une plaque
        for a in(90, 180):
            self.__tab_cl.insert(0, 0.0)
            self.__tab_cd.insert(0, 1.1)
            self.__tab_cm.insert(0, 0.0)
            self.__tab_alpha.insert(0, -a)
            self.__tab_cl.append(0.0)
            self.__tab_cd.append(1.1)
            self.__tab_cm.append(0.0)
            self.__tab_alpha.append(a)

    def init_interpolation(self):
        """ Initialisation des tables d'interpolations """
        #ATTENTION : les tables d'interpolations sont en degre pour alpha
        self.__f_cl = interp1d(self.__tab_alpha, self.__tab_cl)
        self.__f_cd = interp1d(self.__tab_alpha, self.__tab_cd)
        self.__f_cm = interp1d(self.__tab_alpha, self.__tab_cm)

    def interpolation(self):
        """ Interpolation des coefficients aero """
        self.__incidence = (self.__alpha+self.__inclinaison)*R2D
        print self.__incidence
        try:
            self.__cl = self.__f_cl(self.__incidence)
            self.__cd = self.__f_cd(self.__incidence)
            self.__cm = self.__f_cm(self.__incidence)
        except (ValueError):
            self.__cd = 1.1
            self.__cl = 0.0
            self.__cm = 0.0
        print self.__cd

    def apply(self, v_aero):
        """ Calcul des forces et moments generes par l'aile
            Repere aile
            O: Foyer
            Ox: vers l'avant
            Oy: vers la droite
            Oz: vers le  bas
        """
        #Calcul des angles d'incidence et de derapge de l'aile
        self.derapage(v_aero)
        self.incidence(v_aero)
        norm_v_aero = linalg.norm(v_aero)
        #Calcul des coefficients aero
        self.interpolation()
        #Forces dans le repere aile
        self.__vforce[0] = -0.5*RHO*(norm_v_aero**2)*self.__surface*self.__cd
        self.__vforce[1] =  0.0
        self.__vforce[2] = -0.5*RHO*(norm_v_aero**2)*self.__surface*self.__cl
        #Forces dans le repere drone
        self.__vforce = dot(rot_matrix([-self.__beta, -self.__alpha, 0.0]), self.__vforce)
        #Moments dans le repere drone
        self.__vmoment[0] = 0.0
        self.__vmoment[1] = -0.5*RHO*(norm_v_aero**2)*self.__surface*self.__cm
        self.__vmoment[2] = 0.0
        self.__vmoment += cross(self.__pos, self.__vforce, axis=0)

    def incidence(self, v_aero):
        """ Calcul de l'incidence de l'aile """
        self.__alpha = -math.atan2(v_aero[2], v_aero[0])

    def derapage(self, v_aero):
        """ Calcul de l'angle de dérapage de l'aile """
        self.__beta = -math.atan2(v_aero[1], v_aero[0])

    @property
    def vforce(self):
        return self.__vforce

    @property
    def vmoment(self):
        return self.__vmoment
