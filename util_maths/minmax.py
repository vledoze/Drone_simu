#!/usr/bin/python
#-*- coding: utf-8 -*-

# Auteur : vledoze
# Date   : 03/08/2017

def minmax(val, minval, maxval):
    """ Valeur encadree par son minima et maxima """
    return min(maxval, max(minval, val))
