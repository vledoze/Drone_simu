#!/usr/bin/python

# Auteur : vledoze
# Date   : 16/05/2017

from env_simu import *
from drone import Drone

def f(t, x, obj):
    return obj.get_dX()

def main():
    drone = Drone()
    r = ode(f).set_integrator('vode', method='bdf')
    r.set_initial_value(drone.get_X(), T).set_f_params(drone)
    x = []
    y = []
    z = []
    iimax = int(TFIN/DT)
    for ii in range(iimax):
        angle = (pi/2.0)*float(ii)/float(iimax)
        drone.commande(100, 0, 0, 0, angle)
        X = r.integrate(r.t + DT)
        drone.set_X(X)
        x.append(drone.get_pos_i().A1[0])
        y.append(drone.get_pos_i().A1[1])
        z.append(-drone.get_pos_i().A1[2])
    fig = plt.figure()
    ax  = Axes3D(fig)
    ax.plot(x, y, z)
    plt.show()

if __name__ == "__main__":
    main()
