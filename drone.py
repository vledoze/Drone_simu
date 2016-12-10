#!/usr/bin/python

from math import *
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

#Variables locales
dt = 0.01 #Pas de temps
t  = 0    #Instant courant
d2r= pi/180.
r2d= 180./pi
g  = 9.81

tfin = 10 #fin simu

def rot_matrix(att):
  psi = att[0]
  teta= att[1]
  phi = att[2]
  #Matrice de rotation interiel -> drone
  cpsi = cos(psi)
  spsi = sin(psi)
  ctet = cos(teta)
  stet = sin(teta)
  cphi = cos(phi)
  sphi = sin(phi)
  r1 = np.array([cpsi*ctet,                 spsi*ctet,               -stet])
  r2 = np.array([-spsi*cphi+sphi*cpsi*stet, cpsi*cphi+sphi*spsi*stet, sphi*ctet])
  r3 = np.array([spsi*sphi+cphi*cpsi*stet, -cpsi*sphi+cphi*spsi*stet, cphi*ctet])
  return np.matrix([r1, r2, r3])

class Servo (object):
  def __init__(self):
    self.angle   = 0
    self.vitmax  = pi/2 #Vitesse max : fixee a 90deg/s
    self.angmax  = pi

  def set_servo(self, value):
    #value est en pourcentage d'angle max[-100, 100]
    value    = value/100.
    angleCmd = self.angmax*value
    diff     = angleCmd - self.angle
    if (abs(diff/dt) > self.vitmax):  # On ne peut pas depasser la vitesse max des servos
      diff = np.sign(diff)*self.vitmax*dt
    self.angle = self.angle + diff

class Motor (object):
  def __init__(self, l, e, y, h, angle, sens):
    #(L+e, y, h): position dans le repere drone a plat
    self.vitrot = 0.
    self.force  = 0.
    self.vforce = np.zeros(3)
    self.moment = 0.
    self.vmoment= np.zeros(3)
    self.pos    = np.matrix([[0.], [y], [0.]])
    self.l      = l
    self.e      = e
    self.h      = h
    self.angle  = angle     #angle d'inclinaison des moteurs (0: a plat | 90: vertical)
    self.set_pos()
    self.inert  = 0.001     #inertie des helices
    self.sens   = sens      #signe des vitesses de rotation des helices (-1, 1)
    self.vitmax = 30000*d2r #vitesse max des moteurs (5000tr/min)
    self.formax = 0.5*g     #force max a 500g de pousse

  def set_pos(self):
    self.pos[0] = self.l + self.e*cos(self.angle) - self.h*sin(self.angle)
    self.pos[2] = self.h*cos(self.angle) + self.e*sin(self.angle)

  def set_motor(self, value, angle):
    #value est en pourcentage de puissance
    value       = value/100.
    #On fait tourner le moteur
    w           = self.vitmax*value    #vitesse de rotation
    wd          = (w - self.vitrot)/dt #Derivee de la vitesse de rotation
    self.vitrot = w
    #Position moteur
    self.angle  = angle
    self.set_pos()
    #Bilan mecanique moteur (force)
    self.force  = self.formax*value   #force generee
    self.vforce = np.matrix([[self.force*sin(self.angle)], [0.], [-self.force*cos(self.angle)]])  #vecteur force dans repere drone
    #Bilan mecanique moteur (moment)
    self.moment = self.sens*self.inert*wd
    self.vmoment= np.matrix([[self.moment*cos(self.angle)], [0.], [self.moment*sin(self.angle)]]) #vecteur moment a l'origine 0 du repere drone
    self.vmoment= np.add(self.vmoment, np.cross(-self.pos, self.vforce, axis=0))
    #print 'self.vmoment \n' , self.vmoment

class Mixer (object):
  def __init__(self, angle):
    #angle : angle d'inclinaison des moteurs (0: a plat | 90: vertical)
    self.motors = []    #Ensemble des moteurs
    self.motors.append(Motor( 0., 0.1, 0.1, 0.025, angle,  1))
    self.motors.append(Motor( 0., 0.1, 0.3, 0.025, angle, -1))
    self.motors.append(Motor(0.2, 0.1, 0.3, 0.025, angle,  1))
    self.motors.append(Motor(0.2, 0.1, 0.1, 0.025, angle, -1))
    self.servos = []    #Ensemble des servos
    for ii in range(4):
       self.servos.append(Servo())  #1: Cap, 2+3: Roulis, 4:Angle de config
    self.trottleCmd = 0. #Gaz commande
    self.roulisCmd  = 0. #Angle de Roulis commande
    self.giteCmd    = 0. #Angle de Gite commande
    self.capCmd     = 0. #Angle de Cap commande
    self.angleCmd   = angle #Angle de configuration commande

  def set_mixer(self, trottleCmd, roulisCmd, giteCmd, capCmd, angleCmd):
    #les valeurs commandee sont en pourcentage de capacite max
    angle = self.servos[3].angle
    if (angle < 0.088): #Environ 5deg
      #Rien est pilote avec les servos
      for ii in range(3):
        self.servos[ii].set_servo(0.0)
      #Tout est pilote avec les moteurs
      value = []
      value.append(+roulisCmd - giteCmd)
      value.append(-roulisCmd - giteCmd)
      value.append(-roulisCmd + giteCmd)
      value.append(+roulisCmd + giteCmd)
      for ii in range(4):
        value[ii] = (value[ii] + trottleCmd*10 + self.motors[ii].sens*capCmd)/14
        if value[ii] < 0.0 :
          value[ii] = 0.0
        elif value[ii] > 100.0:
          value[ii] = 100.0
        self.motors[ii].set_motor(value[ii], angle)
    else:
      #Cap pilote avec servo 1
      self.servos[0].set_servo(capCmd)
      #Roulis commande avec servos 2 et 3
      self.servos[1].set_servo(roulisCmd)
      #Vitesse de rotation des moteurs fixe la vitesse demandee
      for ii in range(4):
        self.motors[ii].set_motor(trottleCmd, angle)
    #Angle de configuration commande
    self.servos[3].set_servo(angleCmd)

class Drone (object):
  # Repere drone (O: Coin arriere haut, Ox: vers l'avant, Oy: vers la gauche )
  def __init__(self):
    self.angle   = 0.
    self.mixer   = Mixer(self.angle)      #Ensemble des servos+moteurs embarques
   # self.aero    = Aero(self.mixer)       #Aero du drone
    self.pos_i   = np.matrix([[0. ], [0. ], [0.]]) #Position dans le repere inertiel (x, y, z)
    self.posg_d  = np.matrix([[0.2], [0.2], [0.2]]) #Position du centre de gravite drone dans le repere drone
    self.att_i   = np.matrix([[0. ], [0. ], [0.]]) #Attitude dans le repere inertiel (psi, teta, phi)
    self.vit_i   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse dans le repere interiel (xp, yp, zp)
    self.vit_d   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse dans le repere drone    (u, v, w)
    self.omg_i   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse de rotation dans le repere inertiel (psip, tetap, phip)
    self.omg_d   = np.matrix([[0. ], [0. ], [0.]]) #Vitesse de rotation dans le repere drone (p, q, r)
    self.accl_d  = np.matrix([[0. ], [0. ], [0.]]) #Acceleration lineaire dans le repere drone (up, vp, wp)
    self.accr_d  = np.matrix([[0. ], [0. ], [0.]]) #Acceleration rotation dans le repere drone (pp, qp, rp)
    self.masse   = 1                               #masse de 1kg
    self.inertie = 1                               #inertie du drone

  def meca_drone(self):
    sumForce  = np.matrix('0.; 0.; 0.')
    sumMoment = np.matrix('0.; 0.; 0.')
    for ii in range(4):
      sumForce = np.add(sumForce, self.mixer.motors[ii].vforce)
      sumMoment= np.add(sumMoment, self.mixer.motors[ii].vmoment)
    Rdsi     = rot_matrix(self.att_i.A1)
    forceg   = np.matrix([[0.], [0.], [self.masse*g]])
    forceg   = np.dot(Rdsi, forceg)
    sumForce = np.add(sumForce, forceg)
    sumMoment= np.add(sumMoment, np.cross(-self.posg_d, forceg, axis=0))
    #print 'sumMoment \n', sumMoment
    #Acceleration
    self.accl_d =  sumForce/self.masse
    self.accr_d =  sumMoment/self.inertie

  def cin_drone(self):
    Rdsi        = rot_matrix(self.att_i.A1)
    Risd        = Rdsi.transpose()
    #Evolution position
    self.pos_i  = np.add(self.pos_i, np.dot(self.vit_i,  dt))
    self.vit_d  = np.add(self.vit_d, np.dot(self.accl_d, dt))
    self.vit_i  = np.dot(Risd, self.vit_d)
    #Evolution attitude
    self.att_i  = np.add(self.att_i, np.dot(self.omg_i, dt))
    self.omg_d  = np.add(self.omg_d, np.dot(self.accr_d, dt))
    #print 'self.accr_d \n', self.accr_d
    self.omg_i  = np.dot(Risd, self.omg_d)
    #print 'self.omg_i \n', self.omg_i
    self.meca_drone()

  def run_drone(self):
    trottle = 30.
    cap     = 0.
    gite    = 0.
    roulis  = 0.
    angle   = 45.*d2r*cos(t) + 45.
    self.mixer.set_mixer(trottle, roulis, gite, cap, angle)
    self.cin_drone()

if __name__ == "__main__":
  drone = Drone()
  x = []
  y = []
  z = []
  iimax = int(tfin/dt)
  for ii in range(iimax):
    t = t + dt
    drone.run_drone()
    x.append(drone.pos_i.A1[0])
    y.append(drone.pos_i.A1[1])
    z.append(drone.pos_i.A1[2])
  fig = plt.figure()
  ax  = Axes3D(fig)
  ax.plot(x, y, z)
  plt.show()
