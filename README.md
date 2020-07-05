# Searching-Ambulance
In this project, we consider the problem of getting some patients to rhe nearest hospital according to distance on the given map. The ambulance should move behind a patient to reach the nearest hospital. In the shortest time, the ambulance should take all Ps (stands for patients) to hospitals.

In the testcase.zip file you can find the map of a city where #s are obstacles, Ps are patients, A is the ambulance, and numbers are hospitals showing their capacity. I used BFS(Breadth-first search), IDS(Iterative deepening depth-first search), A Star with Manhattan heuristic function, and A Star with the number of remaining patients heuristic function.

Also, I implemented an improvement to the algorithm by adding states with priority at first. The codes have -priority suffix.


