#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 16/05/2017

from util_maths import math
from util_simu import ode, T, DT, TDEB, TFIN
from drone import Drone

def derivate(t, x, obj):
    """ Derivee de l'etat de l'objet """
    obj.run()
    return obj.get_dX()

def integrate(obj):
    """ Integration de l'état de l'objet """
    r = ode(derivate).set_integrator('dopri5', method='bdf')
    r.set_initial_value(obj.get_X(), TDEB).set_f_params(obj)
    for T in range(TDEB,TFIN,DT):
        obj.commande(100., 0. , 0., 0., 0. )
        obj.set_X(r.integrate(r.t + float(DT)/1000.))

def main():
    """ Fonction principale """
    drone = Drone(math.pi/2.0)
    drone.observables = ("_pos_i","_Wing__vforce", "_Wing__cd")
    integrate(drone)
    drone.plot("_pos_i", "_Wing__vforce", "_Wing__cd")

if __name__ == "__main__":
    main()
