#===============================================
# version Python de ROBIZAR
#-----------------------------------------------
# a utiliser avec le fichier de simulation dotant
# ROBIZAR de camera (mobile-eyes.ttt)
#-----------------------------------------------
# Jacques BOONAERT - mars 2020
# UV Robotique & Vision
import time

#===============================================
# tentative d'importation de la librairie V-REP :
try:
    import ctypes as ct
    import sim         # importation des fonction de l'API Coppelia
    import simConst
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
# gestion du temps
import time
# maths
import math
# interface avec le shell
import sys
# open CV
#import cv2
#import numpy as np
#*******************************
# jouent le role des constantes
# de le version C du programme :
# ******************************
VREP_PORT = 19997
#........................
# fonctions de conversion
#........................
#&&&&&&&&&&&&
# deg --> rad
#&&&&&&&&&&&&
def DEG2RAD(x):
  y = MON_PI * ( x / 180.0)
  return( y )
#&&&&&&&&&&&&
# rad --> deg
#&&&&&&&&&&&&
def RAD2DEG(x):
  y = 180.0 * ( y / MON_PI )
  return( y )
#&&&&&&&&&&&&&&&&&&&&&
# aide de ce programme
#&&&&&&&&&&&&&&&&&&&&&
def usage(szPgmName):
  print(szPgmName + '< Adresse IP du serveur V-REP>')
######################
# script principal
######################
argc = len(sys.argv)
if( argc == 1 ):
    szServer = '127.0.0.1'
else:
    szServer = sys.argv[0]
#.........................................
# tentative de connexion au serveur V-REP
#.........................................
sim.simxFinish(-1)
time.sleep(1)
siID = sim.simxStart(szServer, VREP_PORT, 1, 1, 10000, 50)
if( siID < 0):
  print('ERREUR : main() ---> appel a simxStart() : impossible de se connecter a ' +  sys.argv[1])
  print('         valeur de retour = ' + str(siID))
  exit()
#.................................................
# recuperation du handle sur le moteur
#.................................................
siErrorCode, iMoteur = sim.simxGetObjectHandle(siID, 'rightMotor', sim.simx_opmode_blocking)
siErrorCode, aMoteur = sim.simxGetObjectHandle(siID, 'leftMotor', sim.simx_opmode_blocking)
siErrorCode, ROBOT = sim.simxGetObjectHandle(siID, 'KOBUKI', sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
    print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
    print('         code de retour V-REP = ' + str(siErrorCode))
    sim.simxFinish(siID)
    exit()
# handle sur le moteur OK :
print('handle sur le moteur = OK ! ')
#changement de la vitesse du moteur
#sim.simxSetJointTargetVelocity(siID,iMoteur,15,sim.simx_opmode_blocking)
#distance
sim.simxSetObjectPosition(siID,ROBOT,-1,(0,0,0.37),sim.simx_opmode_blocking)
sim.simxSetJointTargetVelocity(siID,iMoteur,0,sim.simx_opmode_blocking)
sim.simxSetJointTargetVelocity(siID,aMoteur,0,sim.simx_opmode_blocking)
import dij
nodes = [(0,0,0.37), (0.2,0,0.37), (0.2,0.4,0.37), (0.4,0.4,0.37), (0.6,0.4,0.37), (0.6,0.6,0.37), (0.8,0.6,0.37), (0.8,0.8,0.37)]
init_graph = {}
for node in nodes:
    init_graph[node] = {}

init_graph[(0,0,0.37)][(0.2,0,0.37)] = 1
init_graph[(0.2,0,0.37)][(0.2,0.4,0.37)] = 2
init_graph[(0.2,0.4,0.37)][(0.4,0.4,0.37)] = 3
init_graph[(0.4,0.4,0.37)][(0.6,0.4,0.37)] = 4
init_graph[(0.6,0.4,0.37)][(0.6,0.6,0.37)] = 5
init_graph[(0.6,0.6,0.37)][(0.8,0.6,0.37)] = 6
init_graph[(0.8,0.6,0.37)][(0.8,0.8,0.37)] = 7
init_graph[(0.8,0.8,0.37)][(0.4,0.4,0.37)] = 20
init_graph[(0.8,0.6,0.37)][(0.6,0.6,0.37)] = 50
graph=dij.Graph(nodes,init_graph)
def stop():
    sim.simxSetJointTargetVelocity(siID,iMoteur,0,sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID,aMoteur,0,sim.simx_opmode_blocking)
def movey(dy):
   (a,[x0,y0,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
   while True:
      (a,[x0,y,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
      if abs(y-y0) >= abs(dy):
            break
      elif dy>0:
         sim.simxSetJointTargetVelocity(siID,iMoteur,10,sim.simx_opmode_blocking)
         sim.simxSetJointTargetVelocity(siID,aMoteur,-10,sim.simx_opmode_blocking)
      else :
         sim.simxSetJointTargetVelocity(siID,iMoteur,-10,sim.simx_opmode_blocking)
         sim.simxSetJointTargetVelocity(siID,aMoteur,10,sim.simx_opmode_blocking)
         break
      stop()

def movex(dx):
   (a,[x0,y0,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
   while True:
      (a,[x,y0,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
      if abs(x-x0) >= abs(dx):
         break
      elif dx>0:
         sim.simxSetJointTargetVelocity(siID,iMoteur,10,sim.simx_opmode_blocking)
         sim.simxSetJointTargetVelocity(siID,aMoteur,10,sim.simx_opmode_blocking)
      else :
         sim.simxSetJointTargetVelocity(siID,iMoteur,-10,sim.simx_opmode_blocking)
         sim.simxSetJointTargetVelocity(siID,aMoteur,-10,sim.simx_opmode_blocking)
         break
      stop()

def rotateleftz(teta):
   (a,[x0,y0,z0])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
   while True:
      (a,[x0,y0,z])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
      teta0=abs(z-z0)
      
      if teta0<teta:
         sim.simxSetJointTargetVelocity(siID,iMoteur,10,sim.simx_opmode_blocking)
         sim.simxSetJointTargetVelocity(siID,aMoteur,-10,sim.simx_opmode_blocking)
         break
      stop()

def rotaterightz(teta):
   (a,[x0,y0,z0])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
   while True:
      (a,[x0,y0,z])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
      teta0=z-z0
      
      if teta0<teta:
         sim.simxSetJointTargetVelocity(siID,iMoteur,-10,sim.simx_opmode_blocking)
         sim.simxSetJointTargetVelocity(siID,aMoteur,10,sim.simx_opmode_blocking)
         break 
      stop()

movex(10)
movey(10)



   
#..............................
# deconnexion du serveur V-REP
#..............................
sim.simxFinish(siID)

