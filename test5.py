#===============================================
# version Python de ROBIZAR
#-----------------------------------------------
# a utiliser avec le fichier de simulation dotant
# ROBIZAR de camera (mobile-eyes.ttt)
#-----------------------------------------------
# Jacques BOONAERT - mars 2020
# UV Robotique & Vision
#==============================================
#===============================================
import numpy as np
import math 

def cosine_similarity(vector1, vector2):
    dot_product = np.dot(vector1, vector2)
    norm1 = np.linalg.norm(vector1)
    norm2 = np.linalg.norm(vector2)
    return dot_product / (norm1 * norm2)

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
#...........................................
# tentative de connexion au serveur V-REP
#...........................................
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
#=====================================================
def orientation():
    # récupérer l'orientation du robot
    a, [x, y, z] = sim.simxGetObjectOrientation(siID, ROBOT, -1, sim.simx_opmode_blocking)

    # calculer les angles d'Euler correspondant aux axes x et y
    x_angle = abs(x) * 180 / math.pi
    y_angle = abs(y) * 180 / math.pi

# déterminer l'orientation du robot
    if x_angle < y_angle:
        print("suivant x")
        
    else:
        print("suivant y")
        


#===========================================
y0 = 0 # initial position
def move_y(dy, velocity=5,angle=0.5):
    global y0 # use the global y0 variable
    (a,[x,y,z])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
     # calculer les angles d'Euler correspondant aux axes x et y
    x_angle = abs(x) * 180 / math.pi
    y_angle = abs(y) * 180 / math.pi
    if y_angle<x_angle:
        rotate(angle,1)
    if dy<0:
        velocity= -velocity
    else:
        velocity=velocity
    distance_traveled=0     
    while abs(y)<abs(dy):
        (a,[x,y,z])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
        print((a,[x,y,z]))
        distance_traveled=y-dy
        sim.simxSetJointTargetVelocity(siID, iMoteur, velocity, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(siID, aMoteur, velocity, sim.simx_opmode_blocking)
        sim.simxSynchronousTrigger(siID)
        print(distance_traveled)
    sim.simxSetJointTargetVelocity(siID, iMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)
    y0 = y # update the initial position

#=========================================== 

import math
#===================================================

#==============================================
def rotate(angle, velocity=10):
    global a0
    a0, [x0, y0, z0] = sim.simxGetObjectOrientation(siID, iMoteur, -1, sim.simx_opmode_blocking)
    if angle >= 0:
        velocity = abs(velocity)
    else:
        velocity = -abs(velocity)
    angle_traveled = 0
    start_time = time.time()
    while abs(angle_traveled) < abs(angle):
        a, [x, y, z] = sim.simxGetObjectOrientation(siID, iMoteur, -1, sim.simx_opmode_blocking)
        angle_traveled = (a - a0)
        sim.simxSetJointTargetVelocity(siID, iMoteur,-velocity, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(siID, aMoteur, velocity, sim.simx_opmode_blocking)
        sim.simxSynchronousTrigger(siID)
        if time.time() - start_time > 5:  # sortir de la boucle si rien ne se passe pendant 5 secondes
            break
    
        
    sim.simxSetJointTargetVelocity(siID, iMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)
    a0=a

def move_x(dx, velocity=5,angle=0.5):
    (a,[x,y,z])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
    x_angle = abs(x) * 180 / math.pi
    y_angle = abs(y) * 180 / math.pi
    if x_angle<y_angle:
        rotate(angle,1)
    if dx<0:
        velocity= -velocity
    else:
        velocity=velocity
    while abs(x)<abs(dx):
        (a,[x,y,z])=sim.simxGetObjectPosition(siID,ROBOT,-1,sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(siID, iMoteur, velocity, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(siID, aMoteur, velocity, sim.simx_opmode_blocking)
        sim.simxSynchronousTrigger(siID)
    sim.simxSetJointTargetVelocity(siID, iMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)
    

def go(x, velocity=1):
    (a, [x0, y, z]) = sim.simxGetObjectPosition(siID, ROBOT, -1, sim.simx_opmode_blocking)
    while abs(x0) < abs(x):
        sim.simxSetJointTargetVelocity(siID, iMoteur, velocity, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(siID, aMoteur, velocity, sim.simx_opmode_blocking)
        sim.simxSynchronousTrigger(siID)
        (a, [x0, y, z]) = sim.simxGetObjectPosition(siID, ROBOT, -1, sim.simx_opmode_blocking)
        if abs(x0) >= abs(x):
            break
    sim.simxSetJointTargetVelocity(siID, iMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)






    
    

    





def rotatediagsens1():
    sim.simxSetJointTargetVelocity(siID, iMoteur, -1, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, 1, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)
    time.sleep(0.83)
    sim.simxSetJointTargetVelocity(siID, iMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)
def rotatediagsens2():
    sim.simxSetJointTargetVelocity(siID, iMoteur, 1, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, -1, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)
    time.sleep(0.83)
    sim.simxSetJointTargetVelocity(siID, iMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(siID, aMoteur, 0, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(siID)





#=====================================================
#initialisation 
sim.simxSetObjectPosition(siID,ROBOT,-1,[0,0,0.2],sim.simx_opmode_blocking)
sim.simxSetJointTargetVelocity(siID,iMoteur,0,sim.simx_opmode_blocking)
sim.simxSetJointTargetVelocity(siID,aMoteur,0,sim.simx_opmode_blocking)
#-----------------------------------------------------
#move
orientation()
move_y(0.4,1)
time.sleep(0.2)
move_x(0.6,1)
time.sleep(0.2)
rotatediagsens2()
time.sleep(0.2)
go(1.6,1)# tu donnes la conditions d'arrêt sur la cordonnée x 
time.sleep(0.2)
rotatediagsens1()
time.sleep(0.2)
move_x(0.46+0.92+3*0.23,1)
rotatediagsens1()
time.sleep(0.1)
go(0.46+0.92+3*0.23+0.2,1)
time.sleep(0.2)
rotatediagsens2()
time.sleep(0.2)
move_x(0.46+0.92+3*0.23+0.2,1)
move_y(2.2,1.5)

















    













   
#..............................
# deconnexion du serveur V-REP
#..............................
sim.simxFinish(siID)

