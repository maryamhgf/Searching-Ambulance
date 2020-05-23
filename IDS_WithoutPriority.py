import numpy as np
from collections import deque
import time
import copy

def numberOfRows(n):
    row = 1
    for i in n:
        if(i == '\n'):
            row = row +1
    return row

def checkGoalState(map): #map: 2D array
    indexesOfP = np.where(np.array(map) == 'P')
    if(indexesOfP[0].size == 0 and indexesOfP[1].size == 0):
        return True
    return False



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

            if(dimUpLeftNeighbor == 'P'):
                PPosition = 'UPLEFT'

            if(dimDownRightNeighbor == 'P'):
                PPosition = 'DOWNRIGHT'

            if(dimDownLeftNeighbor == 'P'):
                PPosition = 'DOWNLEFT'

        if(PPosition != 'DOWNLEFT' and PPosition != 'DOWNRIGHT' and PPosition != 'UPLEFT' and PPosition != 'UPRIGHT'):
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

def checkIfNotInExplored(explored, state):
    notInExplored = True
    for x in explored:
        if(x == state):
            notInExplored = False
            break
    return notInExplored

def DFS(currentNode, explored, depth, dataArray2D, allStates, uniqueStates):
    currentState, path, currentCost = currentNode
    if(checkGoalState(currentState)):
        return [currentState, path, currentCost, 'SUCCESS', allStates, uniqueStates]
    if checkIfNotInExplored(explored, currentState):
        if(checkGoalState(currentState)):
            return [currentState, path, currentCost, 'SUCCESS', allStates, uniqueStates]
        if depth == 0:
            return [currentState, path, currentCost, 'END', allStates, uniqueStates]
        explored.append(currentState)
        neighbors = expandNeighbors(currentState)
        for state, action in neighbors:
            allStates = allStates + 1
            path.append(action)
            cost = currentCost + 1
            if checkIfNotInExplored(explored, state):
                uniqueStates = uniqueStates + 1
                nextState, nextPath, nextCost, stateOfSearch, allStates, uniqueStates = DFS([state, path, cost], explored, depth - 1, dataArray2D, allStates, uniqueStates)
                if(stateOfSearch == 'SUCCESS'):
                    return [nextState, nextPath, nextCost, stateOfSearch, allStates, uniqueStates]
                else:
                    continue
    return [currentState, path, currentCost, 'CONTINUE', allStates, uniqueStates]


def IDS():
    with open('D:/AI/CA1/test3.txt', 'r') as f:
        data = f.read()
        print(data)
        print("FILE OPENED:::::::::::::::::::")
    dataArray = np.array(list(data))
    numOfRow = numberOfRows(dataArray)
    dataArraydeleted = np.delete(dataArray, np.argwhere(dataArray == '\n'))
    dataArray2D = np.reshape(dataArraydeleted, (numOfRow, -1))
    explored = deque()
    path = []
    cost = 0
    iterate = 0
    depth = -1
    allStates = 0
    uniqueStates = 0
    while True:
        iterate = iterate + 1
        depth = depth + 1
        if(iterate == 246):
            print("::::::::::::::::::CHECK", iterate)
            break
        finalState, finalPath, finalCost, stateOfSearch, allStates, uniqueStates = DFS([dataArray2D.tolist(), path, cost], explored, depth + 1, dataArray2D, allStates, uniqueStates)
        if stateOfSearch == 'SUCCESS':
            print("SUCCESS")
            return [finalState, finalPath, finalCost, allStates, uniqueStates]
        explored = deque()
        cost = cost + 1
  


start = time.time()
finalState, finalPath, finalCost, finalAllStates, finalUniqueStates = IDS()
end = time.time()
print("Time: ", end - start)
print("finalState: \n", np.array(finalState))
print("Path Length: ", len(finalPath))
print("Final Cost: ", finalCost)
print("Number of All States: ", finalAllStates)
print("Number of Unique States: ", finalUniqueStates)
