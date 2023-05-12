#===============================================
# version Python de ROBIZAR
from Graph import dijkstra_algorithm
#-----------------------------------------------
# a utiliser avec le fichier de simulation dotant
# ROBIZAR de camera (mobile-eyes.ttt)
#-----------------------------------------------
# Jacques BOONAERT - mars 2020
# UV Robotique & Vision
#===============================================
# tentative d'importation de la librairie V-REP :
try:
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
        
        # Initialiser le graphe
        nodes = [(0, 0), (2, 2), (4, 4), (6, 6)]
        init_graph = {
            (0, 0): {(2, 2): 2, (4, 4): 4},
            (2, 2): {(4, 4): 2, (6, 6): 4},
            (4, 4): {(6, 6): 2},
            (6, 6): {}
        }
        
        # Exécuter l'algorithme de Dijkstra
        graph = Graph(nodes, init_graph)
        previous_nodes, shortest_path =dijkstra_algorithm(graph=graph, start_node=(0, 0))
        
        # Déplacer le robot vers les destinations
        for node in shortest_path:
            # Récupérer les coordonnées (x, y) de la destination
            x = node[0]
            y = node[1]
            
            # Envoyer les coordonnées au robot dans CoppeliaSim
            sim.simxSetObjectPosition(clientID,'rightMotor', -1, [x, y, 0.5], sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(clientID, True)
            sim.simxSetObjectPosition(clientID, 'rightMotor', -1, [x, y, 0.5], sim.simx_opmode_oneshot_wait)
            sim.simxPauseCommunication(clientID, False)
            
            # Attendre que le robot atteigne la destination
            while True:
                return_code, robot_position = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
                if return_code == 0 and robot_position[0] == x and robot_position[1] == y:
                    break
                    
        print('Robot déplacé vers les destinations')
        
        # Fermer la connexion avec CoppeliaSim
        sim.simxFinish(clientID)
    else:
        print('Échec du chargement de la scène dans CoppeliaSim')
        sys.exit(1)
else:
    print('Échec de la connexion avec CoppeliaSim')
    sys.exit(1)
