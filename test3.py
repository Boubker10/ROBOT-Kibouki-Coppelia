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
import dij
nodes = [(3,2,0.30), (3,1,0.30), (1,2,0.30), (2,2,0.30), (2,2,0.30), (3,2,0.30), (2,3,0.30), (3,3,0.30)]
init_graph = {}
for node in nodes:
    init_graph[node] = {}

init_graph[(3,2,0.3)][(3,1,0.3)] = 5
init_graph[(3,2,0.3)][(1,2,0.3)] = 4
init_graph[(3,1,0.3)][(2,2,0.3)] = 1
init_graph[(3,1,0.3)][(2,2,0.3)] = 3
init_graph[(2,2,0.3)][(3,2,0.3)] = 5
init_graph[(2,2,0.3)][(2,3,0.3)] = 4
init_graph[(2,3,0.3)][(3,2,0.3)] = 1
init_graph[(3,3,0.3)][(2,2,0.3)] = 2
init_graph[(3,3,0.3)][(2,3,0.3)] = 2
graph=dij.Graph(nodes,init_graph)
def moveX(dx):
   
      (a,[x0,y0,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
      while True:
        (a,[x,y0,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
        d0=x-x0
        if dx==d0:
            break
      return 
def MoveY(dy):
   
      (a,[x0,y0,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
      while True :
        (a,[x0,y,z0])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
        d0=y-y0
        if dy==d0:
            break 
      return 
def rotad(teta):
   
      (a,[x0,y0,z0])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
      while True :
        (a,[x0,y0,z])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
        dz=z-z0
        sim.simxSetJointTargetVelocity(siID,iMoteur,10,sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(siID,aMoteur,-10,sim.simx_opmode_blocking)
        if teta==dz:
            break 
      return 
def rotag(teta):
   
      (a,[x0,y0,z0])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
      while True :
        (a,[x0,y0,z])=sim.simxGetObjectOrientation(siID,ROBOT,-1,sim.simx_opmode_blocking)
        dz=z-z0
        sim.simxSetJointTargetVelocity(siID,iMoteur,10,sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(siID,aMoteur,-10,sim.simx_opmode_blocking)
        if teta==dz:
            break 
      return 

def move_robot(start, end):
    # Appeler la fonction Dijkstra pour obtenir le chemin le plus court
    # entre le point de départ (start) et le point d'arrivée (end)  
    shortest_path = dij.dijkstra_algorithm(graph, start)
    # VÃ©rifier si le chemin le plus court existe
    if shortest_path is None:
        print("Aucun chemin n'a été trouvé.")
        return

    # Parcourir le chemin le plus court pour faire bouger le robot
    for i in range(len(shortest_path) - 1):
        # Convertir les tuples en listes de nombres réels
        
        next_point = shortest_path[i]
        current_point = shortest_path[i + 1]
        c_position = (ct.c_float * 3)(*next_point)
      # Déplacer le robot de la position actuelle à la position suivante
        sim.simxSetObjectPosition(siID, ROBOT , -1, c_position, sim.simx_opmode_blocking)
        print("Le robot se déplace de", current_point, "à    ", next_point, ".")
    


# Utiliser la fonction pour faire circuler le robot
start_point = (1,1)  # Définir le point de départ du robot
end_point = (3,3)  # Définir le point d'arrivée du robot
move_robot(start_point, end_point)
      
   
      

      
      
      
  
   
#..............................
# deconnexion du serveur V-REP
#..............................
sim.simxFinish(siID)