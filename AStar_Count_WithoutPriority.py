from collections import deque
import numpy as np
import copy
import time
from operator import add

def numberOfRows(n):
    row = 1
    for i in n:
        if(i == '\n'):
            row = row +1
    return row

def checkGoalState(map): #map: 2D array
    indexesOfP = np.where(np.array(map) == 'P')
    indexesOfAP = np.where(np.array(map) == 'AP')
    if(indexesOfP[0].size == 0 and indexesOfP[1].size == 0 and indexesOfAP[0].size == 0 and indexesOfAP[1].size == 0):
        return True
    return False

def isAnyP(array):
    numberOfPs = (array == 'P').sum()
    return numberOfPs 

def expandNeighbors(currentState):
    iA, jA = np.where((np.array(currentState) >= 'A') & (np.array(currentState) != 'P') & (np.array(currentState) != 'AP') & (np.array(currentState) < 'AP'))
    if(iA.size != 0 and jA.size != 0):
        stateOfA = 'A'
        iA = int(iA)
        jA = int(jA)
    else:
        stateOfA = 'AP'
        iA, jA = np.where((np.array(currentState) >= 'AP') & (np.array(currentState) != 'P') & (np.array(currentState) != '#'))
        iA = int(iA)
        jA = int(jA)
    nodes = []
    upNeighbor = currentState[iA-1][jA]
    downNeighbor = currentState[iA + 1][jA]
    rightNeighbor = currentState[iA][jA + 1]
    leftNeighbor = currentState[iA][jA - 1]
    if (stateOfA == 'A'):
        if(len(currentState[iA][jA]) == 1):
            replaceA = ' '
        else:
            replaceA = currentState[iA][jA][1:len(currentState[iA][jA])]
        if(upNeighbor == 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = 'AP' + replaceA
            newAction = 'UP'
            nodes.append((newState, newAction))
   
        if(downNeighbor == 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = 'AP' + replaceA
            newAction = 'DOWN'
            nodes.append((newState, newAction))

        if(rightNeighbor == 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = 'AP' + replaceA
            newAction = 'RIGHT'
            nodes.append((newState, newAction))
 
        if(leftNeighbor == 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = 'AP' + replaceA
            newAction = 'LEFT'
            nodes.append((newState, newAction))
                                              
        if(upNeighbor != '#' and upNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA - 1][jA] = 'A' + newState[iA - 1][jA]
            newAction = 'UP'
            nodes.append((newState, newAction)) 
        if(downNeighbor != '#' and downNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA + 1][jA] = 'A' + newState[iA + 1][jA]
            newAction = 'DOWN'
            nodes.append((newState, newAction)) 
        if(rightNeighbor != '#' and rightNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA][jA + 1] = 'A' + newState[iA][jA + 1]
            newAction = 'RIGHT'
            nodes.append((newState, newAction)) 
        if(leftNeighbor != '#' and leftNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA][jA - 1] = 'A' + newState[iA][jA - 1]
            newAction = 'LEFT'
            nodes.append((newState, newAction)) 

    elif stateOfA == 'AP':
        if(len(currentState[iA][jA]) == 2):
            replaceA = ' '
        else:
            replaceA = currentState[iA][jA][2:len(currentState[iA][jA])]
        if(upNeighbor == 'P'):
            PPosition = 'UP'
            iP = iA - 1
            jP = jA
        elif(downNeighbor == 'P'):
            PPosition = 'DOWN'
            iP = iA + 1
            jP = jA
        elif(leftNeighbor == 'P'):
            PPosition = 'LEFT'
            iP = iA
            jP = jA - 1
        elif(rightNeighbor == 'P'):
            PPosition = 'RIGHT'
            iP = iA
            jP = jA + 1
        elif(downNeighbor != 'P' and upNeighbor != 'P' and leftNeighbor != 'P' and rightNeighbor != 'P'):
            dimUpRightNeighbor = currentState[iA -1][jA + 1]
            dimUpLeftNeighbor = currentState[iA -1][jA - 1]
            dimDownRightNeighbor = currentState[iA + 1][jA + 1]
            dimDownLeftNeighbor = currentState[iA + 1][jA - 1]
            if(dimUpRightNeighbor == 'P'):
                PPosition = 'UPRIGHT'
                iP = iA - 1
                jP = jA + 1
            if(dimUpLeftNeighbor == 'P'):
                PPosition = 'UPLEFT'
                iP = iA - 1
                jP = jA - 1
            if(dimDownRightNeighbor == 'P'):
                PPosition = 'DOWNRIGHT'
                iP = iA + 1
                jP = jA + 1
            if(dimDownLeftNeighbor == 'P'):
                PPosition = 'DOWNLEFT'
                iP = iA + 1
                jP = jA - 1
        
        upNeighborOfP = currentState[iP-1][jP]
        downNeighborOfP = currentState[iP + 1][jP]
        rightNeighborOfP = currentState[iP][jP + 1]
        leftNeighborOfP = currentState[iP][jP - 1] 

        if(PPosition == 'UP' and upNeighborOfP.isdigit() and int(upNeighborOfP) > 0):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'A'
            newState[iP - 1][jP] = str(int(upNeighborOfP) - 1)
            newState[iA][jA] = replaceA
            newAction = 'UP'
            nodes.append((newState, newAction)) 
    
        if(PPosition == 'DOWN' and downNeighborOfP.isdigit() and int(downNeighborOfP) > 0):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'A'
            newState[iP + 1][jP] = str(int(downNeighborOfP) - 1)
            newState[iA][jA] = replaceA
            newAction = 'DOWN'
            nodes.append((newState, newAction))
          
        if(PPosition == 'RIGHT' and rightNeighborOfP.isdigit() and int(rightNeighborOfP) > 0):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'A'
            newState[iP][jP + 1] = str(int(rightNeighborOfP) - 1)
            newState[iA][jA] = replaceA
            newAction = 'RIGHT'
            nodes.append((newState, newAction))
 
        if(PPosition == 'LEFT' and leftNeighborOfP.isdigit() and int(leftNeighborOfP) > 0):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'A'
            newState[iP][jP - 1] = str((int(leftNeighborOfP) - 1))
            newState[iA][jA] = replaceA
            newAction = 'LEFT'
            nodes.append((newState, newAction)) 

        if(PPosition == 'UP' and upNeighborOfP != '#' and upNeighborOfP != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'AP'
            newState[iP - 1][jP] = 'P'
            newState[iA][jA] = replaceA
            newAction = 'UP'
            nodes.append((newState, newAction)) 
        if(PPosition == 'DOWN' and downNeighborOfP != '#' and downNeighborOfP != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'AP'
            newState[iP + 1][jP] = 'P'
            newState[iA][jA] = replaceA
            newAction = 'DOWN'
            nodes.append((newState, newAction))
        if(PPosition == 'RIGHT' and rightNeighborOfP != '#' and rightNeighborOfP != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'AP'
            newState[iP][jP + 1] = 'P'
            newState[iA][jA] = replaceA
            newAction = 'RIGHT'
            nodes.append((newState, newAction))
         
        if(PPosition == 'LEFT' and leftNeighborOfP != '#' and leftNeighborOfP != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iP][jP] = 'AP'
            newState[iP][jP - 1] = 'P'
            newState[iA][jA] = replaceA
            newAction = 'LEFT'
            nodes.append((newState, newAction)) 

        if((PPosition == 'UP' or  PPosition == 'DOWN') and leftNeighbor != '#' and leftNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA][jA - 1] = 'AP' + newState[iA][jA - 1]
            newAction = 'LEFT'
            nodes.append((newState, newAction)) 
        if((PPosition == 'UP' or PPosition == 'DOWN') and rightNeighbor != '#' and rightNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA][jA + 1] = 'AP' + newState[iA][jA + 1]
            newAction = 'RIGHT'
            nodes.append((newState, newAction)) 
        if((PPosition == 'RIGHT' or  PPosition == 'LEFT') and downNeighbor != '#' and downNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA + 1][jA] = 'AP' + newState[iA + 1][jA]
            newAction = 'DOWN'
            nodes.append((newState, newAction))   
        if((PPosition == 'RIGHT' or PPosition == 'LEFT') and upNeighbor != '#' and upNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA - 1][jA] = 'AP' + newState[iA - 1][jA]
            newAction = 'UP'
            nodes.append((newState, newAction)) 

        if((PPosition == 'DOWNLEFT' or PPosition == 'DOWNRIGHT') and downNeighbor != '#' and downNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA + 1][jA] = 'AP' +  newState[iA + 1][jA]
            newAction = 'DOWN'
            nodes.append((newState, newAction)) 

        if((PPosition == 'UPRIGHT' or PPosition == 'DOWNRIGHT') and rightNeighbor != '#' and rightNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA][jA + 1] = 'AP' +  newState[iA][jA + 1]
            newAction = 'RIGHT'
            nodes.append((newState, newAction))
        if((PPosition == 'UPLEFT' or PPosition == 'DOWNLEFT') and leftNeighbor != '#' and leftNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA][jA - 1] = 'AP' +  newState[iA][jA - 1]
            newAction = 'LEFT'
            nodes.append((newState, newAction))
        if((PPosition == 'UPRIGHT' or PPosition == 'UPLEFT') and upNeighbor != '#' and upNeighbor != 'P'):
            newState = copy.deepcopy(currentState)
            newState[iA][jA] = replaceA
            newState[iA - 1][jA] = 'AP' +  newState[iA - 1][jA]
            newAction = 'UP'
            nodes.append((newState, newAction))                      
    return nodes



def computeNumOfPsAndAs(currentState):
    array = np.array(currentState)
    PAndAs = np.where((array == 'P') | ((np.array(currentState) >= 'A') & (np.array(currentState) != 'P') & (np.array(currentState) != 'AP') & (np.array(currentState) < 'AP')))
    return len(PAndAs)

def findMinFInOpenLeast(openList):
    l = [item[-1] for item in openList]
    minVal = min(l)
    indexOfMin = np.argwhere(np.array(l) == minVal)
    return openList[int(indexOfMin[0])]

def checkIfNotInExplored(explored, state, dataArray2D):
    notInExplored = True
    for x in explored:
        if(x == state):
            notInExplored = False
            break
    return notInExplored

def AStar():
    with open('D:/AI/CA1/test3.txt', 'r') as f:
        data = f.read()
        print(data)
        print("FIL OPENED:::::::::::::::::::")
    dataArray = np.array(list(data))
    numOfRow = numberOfRows(dataArray)
    dataArraydeleted = np.delete(dataArray, np.argwhere(dataArray == '\n'))
    dataArray2D = np.reshape(dataArraydeleted, (numOfRow, -1))
    openList = deque()
    closedList = deque()
    cost = 0
    path = []
    iA, jA = np.where((dataArray2D >= 'A') & (dataArray2D != 'P') & (dataArray2D != 'AP'))
    if(iA.size != 0 and jA.size != 0):
        iA = int(iA)
        jA = int(jA)
    else:
        iA, jA = np.where(dataArray2D == 'AP')
        iA = int(iA)
        jA = int(jA)
    h = computeNumOfPsAndAs(dataArray2D.tolist())
    nodeA = [dataArray2D.tolist(), path, cost, h]
    openList.append(nodeA)
    allStates = 0
    uniqueStates = 0
    while True:
        if(len(openList) == 0):
            print("LEN0")
            break
        currentNode = findMinFInOpenLeast(openList)
        currentState, path, currentCost, currentF = currentNode
        openList.remove(currentNode)
        closedList.append(currentState)
        if(checkGoalState(currentState)):
            print("SUCCESS")
            return [currentState, currentCost, path, allStates, uniqueStates]
        neighbors = expandNeighbors(currentState)
        for state, action in neighbors:
            cost = currentCost + 1
            allStates = allStates + 1
            if(not checkIfNotInExplored(closedList, state, dataArray2D)):
                continue
            uniqueStates = uniqueStates + 1
            iA, jA = np.where((np.array(state) >= 'A') & (np.array(state) != 'P') & (np.array(state) != 'AP'))
            if(iA.size != 0 and jA.size != 0):
                iA = int(iA)
                jA = int(jA)
            else:
                iA, jA = np.where(np.array(state) == 'AP')
                iA = int(iA)
                jA = int(jA)

            h = computeNumOfPsAndAs(state)
            f = cost + h
            newNode = [state, path, cost, f]
            if(len(openList) == 0):
                notInOpenList = True
            else:
                notInOpenList = not newNode in openList
            if(not notInOpenList):
                continue
            else:
                path.append(action)
                openList.append(newNode)

   

start = time.time()
currentState, cost, path, allStates, uniqueStates = AStar()
end = time.time()
print("Time: ", end - start)
print("Goal State: \n", np.array(currentState))
print("Length of Path: ", len(path))
print("Number of All States: ", allStates)
print("Number of Unique States: ", uniqueStates)
print("Cost: ", cost)