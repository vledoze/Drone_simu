#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 16/05/2017

import matplotlib.pyplot as plt
import re

from util_maths import matrix

class ObjetSimu(object):
    """ Classe de base pour un objet simulé """

    def __init__(self, **observables):
        """ Initialisation """
        self.__obs = {}
        self.__sous_objets = []
        for k in self.__dict__.keys():
            if issubclass(type(self.__dict__[k]), ObjetSimu):
                self.__sous_objets.append(k)

    def __repr__(self):
        """ Representation de l'objet """
        str = ""
        for k in self.__obs.keys():
            str += "{0} : {1}\n".format(k, self.__obs[k])
        return str

    def save(self):
        """ Sauvegarde au fur et a mesure des valeurs des attributs """
        for obs_name in self.__dict__.keys():
            if obs_name is not "_ObjetSimu__obs":
                if not obs_name in self.__sous_objets:
                    if obs_name in self.__obs.keys():
                        if "copy" in dir(self.__dict__[obs_name]):
                            self.__obs[obs_name].append(self.__dict__[obs_name].copy())
                        else:
                            self.__obs[obs_name].append(self.__dict__[obs_name])
                else:
                    self.__dict__[obs_name].save()

    def plot(self, *names):
        """ trace l'evolution des observables demandes """
        for name in names:
            if name in self.__obs.keys():
                list_obs = self.__obs[name]
                if not isinstance(list_obs[0], matrix):
                    fig = plt.figure()
                    plt.plot(self.__obs[name])
                else:
                    fig = plt.figure()
                    for i in range(list_obs[0].size):
                        plt.plot([float(obs[i]) for obs in list_obs], label="Dimension {0}".format(i))
                    plt.legend()
                plt.ylabel(name)
                plt.show()
            else:
                for sous_objet in self.__sous_objets:
                    if re.match((sous_objet+"?").encode('string-escape'), name.lower()):
                        self.__dict__[sous_objet].plot(name)

    @property
    def observables(self):
        """ Recuperation de la liste d'observables """
        return self.__obs

    @observables.setter
    def observables(self, names):
        """ Choix des observables à enregistrer au cours de la simu """
        #On cree les entrees du dictionnaire
        for name in names:
            if name in self.__dict__.keys():
                self.__obs[name] = []
            else:
                for sous_objet in self.__sous_objets:
                    if re.match((sous_objet+"?").encode('string-escape'), name.lower()):
                        self.__dict__[sous_objet].observables = (name,)





