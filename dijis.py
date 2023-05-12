#===============================================
# version Python de ROBIZAR
#-----------------------------------------------
# a utiliser avec le fichier de simulation dotant
# ROBIZAR de camera (mobile-eyes.ttt)
#-----------------------------------------------
# Jacques BOONAERT - mars 2020
# UV Robotique & Vision
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
import Graph
nodes = [(3,2,0), (3,1,0), (1,2,0), (2,2,0), (2,2,1), (3,2,1), (2,3,1), (3,3,2)]
init_graph = {}
for node in nodes:
    init_graph[node] = {}

init_graph[(3,2,0)][(3,1,0)] = 5
init_graph[(3,2,0)][(1,2,0)] = 4
init_graph[(3,1,0)][(2,2,0)] = 1
init_graph[(3,1,0)][(2,2,1)] = 3
init_graph[(2,2,1)][(3,2,1)] = 5
init_graph[(2,2,1)][(2,3,1)] = 4
init_graph[(2,3,1)][(3,2,1)] = 1
init_graph[(3,3,2)][(2,2,0)] = 2
init_graph[(3,3,2)][(2,3,1)] = 2
graph=Graph.Graph(nodes,init_graph)
def move_robot(start, end):
    # Appeler la fonction Dijkstra pour obtenir le chemin le plus court
    # entre le point de dÃ©part (start) et le point d'arrivÃ©e (end)  
    shortest_path = Graph.dijkstra_algorithm(graph, start)
    # VÃ©rifier si le chemin le plus court existe
    if shortest_path is None:
        print("Aucun chemin n'a Ã©tÃ© trouvÃ©.")
        return

    # Parcourir le chemin le plus court pour faire bouger le robot
    for i in range(len(shortest_path) - 1):
        # Convertir les tuples en listes de nombres rÃ©els
        
        next_point = shortest_path[i]
        current_point = shortest_path[i + 1]
        c_position = (ct.c_float * 3)(*next_point)
      # DÃ©placer le robot de la position actuelle Ã  la position suivante
        sim.simxSetObjectPosition(siID, ROBOT , -1, c_position, sim.simx_opmode_blocking)
        print("Le robot se dÃ©place de", current_point, "Ã ", next_point, ".")


# Utiliser la fonction pour faire circuler le robot
start_point = (0, 0)  # DÃ©finir le point de dÃ©part du robot
end_point = (3, 4)  # DÃ©finir le point d'arrivÃ©e du robot
move_robot(start_point, end_point)
      
      
      
  
   
#..............................
# deconnexion du serveur V-REP
#..............................
sim.simxFinish(siID)
