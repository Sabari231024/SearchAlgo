from collections import deque
import random
import heapq

adj = [
    [0, 1, 1, 0, 0, 0, 0],  
    [1, 0, 1, 0, 1, 0, 0], 
    [1, 1, 0, 1, 0, 0, 0],  
    [0, 0, 1, 0, 0, 1, 0],  
    [0, 1, 0, 0, 0, 0, 1],  
    [0, 0, 0, 1, 0, 0, 0],  
    [0, 0, 0, 0, 1, 0, 0]   
]


adj_cost = [
    [0, 3, 8, 0, 0, 0, 0],  
    [3, 0, 4, 0, 3, 0, 0], 
    [5, 4, 0, 4, 0, 0, 0],  
    [0, 0, 4, 0, 0, 6, 0],  
    [0, 3, 0, 0, 0, 0, 5],  
    [0, 0, 0, 6, 0, 0, 0],  
    [0, 0, 0, 0, 5, 0, 0]   
]

'''
adj = [
    [0, 1, 1, 0, 0, 0, 0],  
    [1, 0, 0, 1, 0, 0, 1], 
    [1, 0, 0, 1, 0, 0, 0],  
    [0, 1, 1, 0, 0, 1, 0],  
    [0, 0, 0, 0, 0, 0, 1],  
    [0, 0, 0, 1, 0, 0, 1],  
    [0, 1, 0, 0, 1, 1, 0]   
]
'''

def goal_test(i, j):
    return i == j

def move_gen(adj_list):
    return [index for index, value in enumerate(adj_list) if value == 1]

def rearrange_stack(neighbors, heuristic):
    neighbors.sort(key=lambda node: heuristic[node])
    
def BMS(adj, goal, current, path, visited):
    path.append(current)
    visited[current] = True
    
    if goal_test(current, goal):
        print(f"Goal {goal} achieved! Path:", ' '.join(map(str, path)))
        visited[current] = False
        path.pop()
        return
    
    neighbors = move_gen(adj[current])
    random.shuffle(neighbors)
    for next_node in neighbors:
        if not visited[next_node]:
            print(f"Exploring neighbor: {next_node} from node {current}")
            BMS(adj, goal, next_node, path, visited)
    visited[current] = False
    path.pop()
    print(f"Backtracking from node: {current}")

def DFS(adj, goal, current ,visited ,path):
    stack = [current]  
    while stack:
        current = stack.pop()    
        if not visited[current]:
            visited[current] = True
            path.append(current)       
            if goal_test(current, goal):
                print(f"Goal {goal} achieved! Path:", ' '.join(map(str, path)))
                return
            
            neighbors = move_gen(adj[current])
            for next_node in reversed(neighbors):  
                if not visited[next_node]:
                    stack.append(next_node)
            if not neighbors:
                path.pop()   
    print(f"Goal {goal} not found.")
    
def BFS(adj, goal, start,visited,path):
    queue = [start]
    parent = {start: None}
    visited[start] = True
    while queue:
        current = queue.pop(0)
        if goal_test(current, goal):
            while current is not None:
                path.append(current)
                current = parent[current]
            path.reverse()
            print(f"Goal {goal} achieved! Path:", ' '.join(map(str, path)))
            #print(debug)
            return
        neighbors = move_gen(adj[current])
        print(neighbors)
        for next_node in neighbors:
            if not visited[next_node]:
                queue.append(next_node)
                visited[next_node] = True
                parent[next_node] = current
    print(f"Goal {goal} not found.")

def HILL(adj, goal, start , visited,path ,heur):
    stack = [start]
    while stack:
        current = stack.pop()
        visited[current] = True
        path.append(current)
        if goal_test(current, goal):
            print(f"Goal {goal} achieved! Path:", ' '.join(map(str, path)))
            return
        neighbors = move_gen(adj[current])
        rearrange_stack(neighbors, heur)
        for next_node in reversed(neighbors):
            if not visited[next_node]:
                stack.append(next_node)
        if not any(not visited[n] for n in neighbors):
            path.pop()  
    print(f"Goal {goal} not found.")

def ORACLE(adj, goal, current, path, visited, orc, current_cost):
    path.append(current)
    visited[current] = True
    if current_cost > orc:
        print(f"Stopping exploration at node {current} due to high cost: {current_cost} (orc: {orc})")
        visited[current] = False
        path.pop()
        return
    if goal_test(current, goal):
        print(f"Goal {goal} achieved! Path:", ' '.join(map(str, path)))
        print(f"Total path cost: {current_cost}")
        visited[current] = False
        path.pop()
        return
    for next_node in range(len(adj[current])):
        if adj[current][next_node] > 0 and not visited[next_node]:
            next_cost = current_cost + adj[current][next_node]
            if next_cost <= orc:
                print(f"Exploring neighbor: {next_node} from node {current} with total cost: {next_cost}")
                ORACLE(adj, goal, next_node, path, visited, orc, next_cost)
            else:
                print(f"Skipping neighbor: {next_node} from node {current} due to high cost: {next_cost} (orc: {orc})")
    visited[current] = False
    path.pop()
    print(f"Backtracking from node: {current}")
    
def ORACLE_HEUR(adj, heur, goal, current, path, visited, orc, current_cost):
    path.append(current)
    visited[current] = True
    if current_cost + heur[current] > orc:
        print(f"Stopping exploration at node {current} due to high estimated cost: {current_cost + heur[current]} (orc: {orc})")
        visited[current] = False
        path.pop()
        return
    if goal_test(current, goal):
        print(f"Goal {goal} achieved! Path:", ' '.join(map(str, path)))
        print(f"Total path cost: {current_cost}")
        visited[current] = False
        path.pop()
        return
    for next_node in range(len(adj[current])):
        if adj[current][next_node] > 0 and not visited[next_node]:
            next_cost = current_cost + adj[current][next_node]
            if next_cost + heur[next_node] <= orc:
                print(f"Exploring neighbor: {next_node} from node {current} with estimated total cost: {next_cost + heur[next_node]}")
                ORACLE_HEUR(adj, heur, goal, next_node, path, visited, orc, next_cost)
            else:
                print(f"Skipping neighbor: {next_node} from node {current} due to high estimated cost: {next_cost + heur[next_node]} (orc: {orc})")
    visited[current] = False
    path.pop()
    print(f"Backtracking from node: {current}")

def BEAM(adj, goal, start, visited, path, heur, w):
    found_paths = []  
    queue = deque([[start]])  
    print(f"Starting Beam Search from node {start} to node {goal} with beam width {w}")
    while queue:
        current_path = queue.popleft() 
        current_node = current_path[-1]    
        print(f"Exploring path: {' -> '.join(map(str, current_path))}")
        if current_node == goal:
            found_paths.append(current_path)
            print(f"Goal {goal} achieved! Path: {' -> '.join(map(str, current_path))}")
            return found_paths     
        successors = []
        for neighbor in range(len(adj[current_node])):
            if adj[current_node][neighbor] > 0 and neighbor not in current_path:  
                new_path = current_path + [neighbor]
                successors.append(new_path)
        successors.sort(key=lambda path: heur[path[-1]]) 
        print(f"Successors from node {current_node} sorted by heuristic: {[f'{path[-1]} (heur: {heur[path[-1]]})' for path in successors]}")
        queue.extend(successors[:w])       
    if not found_paths:
        print("No path to the goal found within the given beam width.")


def BNB(adj_cost, goal, start):
    queue = [(0, start, [start])]
    visited = set() 
    while queue:
        print(queue)
        current_cost, current_node, path = heapq.heappop(queue)
        if current_node in visited:
            continue
        visited.add(current_node)
        if current_node == goal:
            print(f"Goal {goal} achieved! Path: {' -> '.join(map(str, path))}")
            print(f"Total path cost: {current_cost}")
            return path, current_cost
        for neighbor in range(len(adj_cost[current_node])):
            if adj_cost[current_node][neighbor] > 0 and neighbor not in visited:
                next_cost = current_cost + adj_cost[current_node][neighbor]
                heapq.heappush(queue, (next_cost, neighbor, path + [neighbor]))
    print("Goal not reachable")
    return None, float('inf')

def BNB_EXT(adj_cost, goal, start):
    queue = [(0, start, [start])]
    visited_cost = [float('inf')] * len(adj_cost)
    visited_cost[start] = 0  
    while queue:
        print(queue)
        current_cost, current_node, path = heapq.heappop(queue)
        if current_cost > visited_cost[current_node]:
            continue
        if current_node == goal:
            print(f"Goal {goal} achieved! Path: {' -> '.join(map(str, path))}")
            print(f"Total path cost: {current_cost}")
            return path, current_cost
        for neighbor in range(len(adj_cost[current_node])):
            if adj_cost[current_node][neighbor] > 0:
                next_cost = current_cost + adj_cost[current_node][neighbor]
                if next_cost < visited_cost[neighbor]:
                    visited_cost[neighbor] = next_cost
                    heapq.heappush(queue, (next_cost, neighbor, path + [neighbor]))
    print("Goal not reachable")
    return None, float('inf')

def ASTAR(adj_cost, goal, start, orc, heur):
    queue = [(heur[start], 0, start, [start])]
    visited_cost = [float('inf')] * len(adj_cost)
    visited_cost[start] = 0 
    while queue:
        print("Queue:", queue)
        _, current_cost, current_node, path = heapq.heappop(queue)
        if current_node == goal:
            print(f"Goal {goal} achieved! Path: {' -> '.join(map(str, path))}")
            print(f"Total path cost: {current_cost}")
            return path, current_cost
        for neighbor in range(len(adj_cost[current_node])):
            if adj_cost[current_node][neighbor] > 0:
                next_cost = current_cost + adj_cost[current_node][neighbor]
                estimated_total_cost = next_cost + heur[neighbor]
                if estimated_total_cost > orc:
                    print(f"Pruning path to neighbor {neighbor} from node {current_node} due to oracle limit "
                          f"(estimated total cost: {estimated_total_cost}, oracle: {orc})")
                    continue
                if next_cost < visited_cost[neighbor]:
                    visited_cost[neighbor] = next_cost
                    heapq.heappush(queue, (estimated_total_cost, next_cost, neighbor, path + [neighbor]))
    print("Goal not reachable")
    return None, float('inf')

def BESTFS(adj_cost, goal, start, heur):
    stack = [(heur[start], start, [start])]  
    visited = set() 
    while stack:
        stack.sort(key=lambda x: x[0]) 
        print("Stack:", stack)
        _, current_node, path = stack.pop(0)  
        if current_node in visited:
            continue
        visited.add(current_node)
        if current_node == goal:
            print(f"Goal {goal} achieved! Path: {' -> '.join(map(str, path))}")
            print(f"Total path heuristic cost: {heur[current_node]}")
            return path, heur[current_node]
        for neighbor in range(len(adj_cost[current_node])):
            if adj_cost[current_node][neighbor] > 0 and neighbor not in visited:
                stack.append((heur[neighbor], neighbor, path + [neighbor]))
    print("Goal not reachable")
    return None, float('inf')

def AO(start: int) -> float:
    """Defines the graph nodes and performs the AO* algorithm from 'start'."""

    # Define nodes with heuristic, type, and children in a simplified format
    nodes = {
        0: {"heuristic": 2, "type": "OR", "children": [1, 2], "cost": float("inf"), "solved": False, "best_child": None},
        1: {"heuristic": 1, "type": "AND", "children": [3, 4], "cost": float("inf"), "solved": False, "best_child": None},
        2: {"heuristic": 3, "type": "OR", "children": [5], "cost": float("inf"), "solved": False, "best_child": None},
        3: {"heuristic": 2, "type": "AND", "children": [], "cost": 0, "solved": True, "best_child": None},
        4: {"heuristic": 1, "type": "AND", "children": [], "cost": 0, "solved": True, "best_child": None},
        5: {"heuristic": 4, "type": "AND", "children": [], "cost": 0, "solved": True, "best_child": None},
    }

    def recursive_ao(node: int) -> float:
        current_node = nodes[node]
        print(f"\nExploring node: {node}")

        # Return cost if already solved
        if current_node["solved"]:
            print(f"Node {node} already solved with cost {current_node['cost']}")
            return current_node["cost"]

        # Initialize min_cost and best_child for OR nodes
        if current_node["type"] == "OR":
            min_cost, best_child = float("inf"), None

            for child in current_node["children"]:
                print(f"Examining OR child: {child}")
                child_cost = recursive_ao(child)
                total_cost = child_cost + current_node["heuristic"]

                if total_cost < min_cost:
                    min_cost, best_child = total_cost, child

            current_node.update({"cost": min_cost, "best_child": best_child, "solved": True})
            print(f"OR node {node} solved with cost {min_cost}, best child: {best_child}")

        # Process AND nodes by summing child costs
        elif current_node["type"] == "AND":
            total_cost = current_node["heuristic"] + sum(recursive_ao(child) for child in current_node["children"])
            current_node["cost"], current_node["solved"] = total_cost, True
            print(f"AND node {node} solved with total cost {total_cost}")

        return current_node["cost"]

    # Start AO* search from the start node
    total_cost = recursive_ao(start)
    print(f"Total cost from start: {total_cost}")
    return total_cost

def BNBWH(adj_cost, goal, start, orc, heur):
    queue = [(0, start, [start])]
    visited = set() 
    while queue:
        print(queue)
        current_cost, current_node, path = heapq.heappop(queue)
        if current_node in visited:
            continue
        visited.add(current_node)
        if current_node == goal:
            print(f"Goal {goal} achieved! Path: {' -> '.join(map(str, path))}")
            print(f"Total path cost: {current_cost}")
            return path, current_cost
        for neighbor in range(len(adj_cost[current_node])):
            if adj_cost[current_node][neighbor] > 0 and neighbor not in visited:
                next_cost = current_cost + adj_cost[current_node][neighbor] 
                if heur[neighbor]+next_cost > orc:
                    continue
                heapq.heappush(queue, (next_cost, neighbor, path + [neighbor]))
    print("Goal not reachable")
    return None, float('inf')


goal = 6  
start = 0 
path = []
visited = [False] * len(adj)  
heur = [10, 8, 6, 8, 5, 10, 0]
#heur = [12, 10, 9, 7, 4, 4, 0]

w=2
orc=11
#BMS(adj, goal, start, path, visited)
#DFS(adj,goal,start,visited,path)
#BFS(adj,goal,start,visited,path)
#HILL(adj,goal,start,visited,path,heur)
#BEAM(adj, goal, start, visited, path, heur, w)
#ORACLE(adj_cost, goal, start, path, visited, orc,0)
#ORACLE_HEUR(adj_cost, heur, goal, start, path, visited, orc, 0)
#BNB(adj_cost, goal, start)
#BNB_EXT(adj_cost, goal, start)
#ASTAR(adj_cost,goal,start,orc,heur)
#BNBWH(adj_cost, goal, start, orc, heur)
#BESTFS(adj_cost, goal, start, heur)
AO(start)