# ce script est destine a pretraiter une "bitmap" (image)
# correspondant a une grille d'occupation de l'espace 
# d'evolution du robot mobile : une valeur de 255 (blanc)
# indique un emplacement "libre" tandis que 0 (noir)
# est associe a la presence d'un obstacle.
# On va construire une liste de connexite des cellules
# libre de l'environnement qui sera ensuite exploitee
# pour le recherche du chemin
#---------------------------------------------------------
# Jacques BOONAERT-LEPERS  module electif ROMOB 2022-2023
import os
import sys
import math
import numpy as np
import cv2
import pathsearch as ps
#.............
# "constantes"
# ............ 
iNB_ARGS = 4
#.........
# switches
#.........
bUSER_DBG = True
#________________________________________________
if bUSER_DBG:
    print('version Opencv = ' + cv2.__version__)
    print('version numpy = ' + np.__version__)
#________________________________________________
#&&&&&&&&&&&&&&&&&&&&&
# aide de ce programme 
#&&&&&&&&&&&&&&&&&&&&&
def usage( szPgmName):
    print(szPgmName + ' <fichier bitmap> <reduction x> <reduction y>')
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# cette fonction "descend" la resolution de la 
# carte initiale pour generer un nombre de sommets
# du graphe raisonnable (traiter chaque pixel sur 
# une image 512 x 512 conduirait a devoir gerer un 
# graphe dote de 256 000 sommets environ)
# IN : 
#     imgSrc   --> image de depart (bitmap couleur)
#     x_factor --> facteur de reduction en x
#     y_factor --> facteur de reduction en y
# OUT :
#     mDest    --> matrice destination (float)
#                  chaque coefficient represente
#                  le taux d'occupation :      
#                     0 = libre
#                     1 = completement occupe
#                     0 < t < 1 : partiellement occupee
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def LowerGridResolution( imgSrc, x_factor, y_factor ):
    iNl = imgSrc.shape[0]    # nombre de lignes
    iNc = imgSrc.shape[1]    # nombre de colonnes
    #_________________________________________________________________________
    if bUSER_DBG:
        print("dimension de l'image de depart = " + str(iNc) + "x" + str(iNl))
    #_________________________________________________________________________
    # initialisation
    iDestNl = int(iNl / y_factor)   # nombre de lignes de la destination
    iDestNc = int(iNc / x_factor)   # nombre de colonnes de la destination
    #_________________________________________________________________________________
    if bUSER_DBG:
        print("dimension de la grille reduite = " + str(iDestNc) + "x" + str(iDestNl))
    #_________________________________________________________________________________
    mDest = np.zeros((iDestNl, iDestNc), dtype=float)
    # comptabilisation de l'occupation moyenne 
    # dans des elements de taille x_factor par y_factor 
    # dans imgSrc : 
    for i in range(iDestNl):
        for j in range(iDestNc):
            # extraction de "l'imagette"
            iStart = i * y_factor
            iStop  = iStart + y_factor
            jStart = j * x_factor
            jStop  = jStart + x_factor
            imgSub = imgSrc[iStart:iStop, jStart:jStop, :]
            # on compte le pixels eteints de l'imagette
            # quelle qu'en soit la couleur : 
            iOn = 0     # nombre de pixels "eteints"
            for u in range(y_factor):
                for v in range(x_factor):
                    if (imgSub[u,v,0] == 0) and (imgSub[u,v,1] == 0) and (imgSub[u,v,2] == 0):
                        iOn+=1
            # d'ou le taux d'occupation moyen 
            fTaux = float(iOn) / float(x_factor * y_factor)
            # qu'on place dans la matrice retournee par la fonction : 
            mDest[i,j] = fTaux
    return mDest
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# affichage de la grille d'occupation au format 'texte'
# IN : 
#    mMap --> la grille d'occupation
# DISPLAY :
# 0 ---> libre
# 1 ---> occupee
# - ---> partiellement occupee
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def DisplayOccupencyGridAsText( mMap, lstPath=[] ):
    if lstPath == None:
        for i in range(mMap.shape[0]):
            szLine = ""
            for j in range(mMap.shape[1]):
                if mMap[i,j] == 0.0:
                    szLine+="0"
                else:
                    if mMap[i,j] == 1.0:
                        szLine+="1"
                    else:
                        szLine+="-"
            print(szLine)
    else:
        for i in range(mMap.shape[0]):
            szLine = ""
            for j in range(mMap.shape[1]):
                if  [i,j] in lstPath:
                    szLine+="*"
                else:
                    if mMap[i,j] == 0.0:
                        szLine+="0"
                    else:
                        if mMap[i,j] == 1.0:
                            szLine+="1"
                        else:
                            szLine+="-"
            print(szLine)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# enumeration des sommets du graphe 
# sous la forme d'une liste d'identifiant entiers 
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def EnumerateNodes( mMap):
    lstNodes=[]
    for i in range(mMap.shape[0]):
        for j in range(mMap.shape[1]):
            if mMap[i,j] == 0:
                iID = i * mMap.shape[0] + j
                lstNodes.append(iID)
    return(lstNodes)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# construction des sommets et des arcs du graphe 
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def BuildEvGraph( mMap):
    # recuperation des sommets
    lstNodes = EnumerateNodes( mMap)
    #...............
    # initialisation
    #............... 
    lstNodesAsText = []
    init_graph = {}
    for n in lstNodes:
        lstNodesAsText.append(str(n))   # on construit au prealable la liste des noeuds en texte
        init_graph[str(n)] = {}
    w = mMap.shape[1]       # nombre de lignes
    h = mMap.shape[0]       # nombre de colonnes
    #......................................................
    # parcours des noeuds, recherche des noeuds connexes et 
    # valuation des arcs : 
    #......................................................
    for n in lstNodes:
        # calcul des indices dans la map : 
        i = int(n/w)
        j = n % w
        # recherche dans le voisinage 
        for x in [-1,0,1]:
            for y in [-1,0,1]:
                ic = i + x
                jc = j + y
                # on verifie qu'on en deborde pas (droite, gauche, haut et bas) et qu'on 
                # ne teste pas le sommet courant (i,j)<=>n lui-meme : 
                if( ic > 0) and (jc > 0) and (ic < h) and (jc < w) and (( i != ic ) or ( j != jc)):
                    kc = ic * w + jc
                    if kc in lstNodes: # si le voisin est libre, il doit figurer parmi les sommets: 
                        dbCout = math.sqrt((i-ic)**2 + (j-jc)**2)
                        init_graph[str(n)][str(kc)] = dbCout
    #on peut construire l'objet graphe correpondant : 
    graph = ps.Graph(lstNodesAsText, init_graph)
    return graph
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# construction d'une liste de "coordonnees" 
# en lieu et place d'une liste de sommets 
# de graphe :                             
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def Nodes2CoordsList( lstPath, iNl, iNc):
    lstCoord = []
    for n in lstPath:
        k = eval(n)
        i = int(k/iNl)
        j = k % iNl
        lstCoord.append([i,j])
    return lstCoord
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# "derivation" (a gauche) de la liste de coordonnees 
#  issue du chemin. Le but est de mettre
#  evidence les changements de direction
#  afin de simplifier le plan initial
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def DerivateCoord( lstCoords):
    n = len(lstCoords)
    lstDerCoords = []
    if n >= 2:
        for i in range(n-1):
            dx = lstCoords[i+1][0] - lstCoords[i][0]
            dy = lstCoords[i+1][1] - lstCoords[i][1]
            lstDerCoords.append([dx,dy])
    else:
        lstDerCoords.append([0,0])
    return lstDerCoords
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# simplification de la liste de coordonnees correspondant
# a la trajectoire "brute"
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def RefineTrajectory( lstCoords ):
    # on derive 2 fois la sequence de coordonnees
    # et on s'interesse aux indices pour lesquels
    # la derivee seconde est non-nulle : 
    lstD1Coords = DerivateCoord( lstCoords)
    lstD2Coords = DerivateCoord( lstD1Coords)
    lstChanges=[]
    n = len(lstD2Coords)
    for i in range(n):
        if (lstD2Coords[i][0] == 0 ) and (lstD2Coords[i][1] == 0):
            lstChanges.append(0)    # pas de changement
        else:
            lstChanges.append(1)    # changement
    # on repere maintenant l'ensemble des indices pour lequel un 
    # changement est observe :
    # on en deduit les indices des cellules ou s'operent 
    # un changement de direction (on ajoute 1...) 
    lstIndex = []
    n =len( lstChanges )
    for i in range(n):
        if lstChanges[i] != 0:
            lstIndex.append(i+1)
    # a partir de la, on peut "epurer" la liste
    # des coordonnes de cellules a joindre par
    # une ligne droite : 
    lstFilteredCoords=[lstCoords[0]]
    for i in range(len(lstIndex)):
        lstFilteredCoords.append(lstCoords[lstIndex[i]])
    # OK
    return lstFilteredCoords
############################
# point d'entree du script : 
############################
argc = len(sys.argv)
if argc != iNB_ARGS:
    usage( sys.argv[0])
# recuperation des arguments 
szImgFileName = "C:\ROMOB\scirpts\occupency.bmp"
iX_factor = 10
iY_factor = 12
#_________________________________________________________
if bUSER_DBG:
    print("fichier d'entree          = " + szImgFileName)
    print("facteur de reduction en X = " + str(iX_factor))
    print("facteur de reduction en Y = " + str(iY_factor))
#__________________________________________________________
# chargement et affichage de l'image : 
imgSrc = cv2.imread(szImgFileName)
print(imgSrc.shape)

cv2.waitKey(0)
# reduction de la carte d'occupation
mMap = LowerGridResolution(imgSrc, 30, 30)
print("grille d'occupation : [0 : libre, 1 : occupee, - : partiellement ]")
DisplayOccupencyGridAsText(mMap)
lstNodes = EnumerateNodes( mMap)
print('liste des sommets = {}'.format(lstNodes))
# construction du graphe de l'environnement 
g = BuildEvGraph(mMap)
# saisie de la cellule de depart : 
iFrom = eval(input("ligne de depart = "))
jFrom = eval(input("colonne de depart = "))
iTo   = eval(input("ligne d'arrivee = "))
jTo   = eval(input("colonne d'arrivee = "))
kFrom = iFrom * mMap.shape[1] + jFrom
kTo   = iTo * mMap.shape[1] + jTo
previous_nodes, shortest_path = ps.dijkstra_algorithm(graph=g, start_node=str(kFrom))
ps.print_result(previous_nodes, shortest_path, start_node=str(kFrom), target_node=str(kTo)) 
lstPath = ps.GetShortestPath(previous_nodes, shortest_path, start_node=str(kFrom), target_node=str(kTo))
#affichage de la grille avec le chemin trouve :
lstCoords = Nodes2CoordsList(lstPath, mMap.shape[1], mMap.shape[0])  
DisplayOccupencyGridAsText(mMap, lstCoords)
# tentative de simplification de la trajectoire :
lstFilteredCoords = RefineTrajectory(lstCoords)
print("liste des coordonnees de cellules a rejoindre : {}".format(lstFilteredCoords))