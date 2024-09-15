import numpy as np
import math
import plotly.graph_objects as px
import matplotlib.pyplot as plt
from simpleai.search import (breadth_first,depth_first,greedy,astar,iterative_limited_depth_first,SearchProblem)

def convert_path_to_points(path):
    points = []
    for step in path:
        points.append(list(step[0]))
    return points

class MarsSearchProblem(SearchProblem):
    def __init__(self, initial_state, goal, mars_map, scale, height_threshold=0.25):
        self.mars_map = mars_map
        self.scale = scale
        self.height_threshold = height_threshold
        super(MarsSearchProblem, self).__init__(initial_state=initial_state)
        self.goal = goal


    def actions(self, state):
        row, col = state
        #movements = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        movements = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        valid_actions = []

        for dr, dc in movements:
            new_row, new_col = row + dr, col + dc
            if (
                0 <= new_row < len(self.mars_map)
                and 0 <= new_col < len(self.mars_map[0])
                and self.mars_map[new_row][new_col] != -1
                and abs(self.mars_map[new_row][new_col] - self.mars_map[row][col]) <= self.height_threshold):
                valid_actions.append((new_row, new_col))

        return valid_actions

    def result(self, state, action):
        return action

    def is_goal(self, state):
        return state == self.goal

    def cost(self, state, action, state2):
        row, col = state
        new_row, new_col = action
        return abs(self.mars_map[new_row][new_col] - self.mars_map[row][col]) + 1

    def heuristic(self, state):
            
            current_row, current_col = state
            goal_row, goal_col = self.goal

            distance = np.sqrt((current_row - goal_row)**2 + (current_col - goal_col)**2)
            return distance

    def show_points(self):
        x = [i[0] for i in self.points]
        y = [i[1] for i in self.points]
        plt.scatter(x, y)
        plt.title('Points')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xticks(np.arange(0, 201, 10))
        plt.yticks(np.arange(0, 201, 10))
        plt.show()

    def show_route(self, route):
        x = [self.points[i][0] for i in route]
        y = [self.points[i][1] for i in route]
        plt.scatter(x, y)
        
        x.append(self.points[route[0]][0])
        y.append(self.points[route[0]][1])
        plt.plot(x, y)
        plt.title('Greedy Route')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xticks(np.arange(0, 201, 10))
        plt.yticks(np.arange(0, 201, 10))

        total_distance = self.total_distance(route)
        plt.text(10, 190, f'Total Distance: {total_distance:.2f}', fontsize=10, bbox=dict(facecolor='white', alpha=0.5))
        print(f'Total Distance: {total_distance:.2f}')
        plt.show()
        
# Mapa de alturas de Marte
mars_map = np.load('mars_map.npy')
nr, nc = mars_map.shape

# posición inicial y final
start_position = (1242,10920)
goal_position =  (3856,540)

#scale
scale = 10.0174

#
row_ini = nr-round(start_position[1]/scale)
col_ini = round(start_position[0]/scale)

row_goal = nr-round(goal_position[1]/scale)
col_goal = round(goal_position[0]/scale)

mars_problem = MarsSearchProblem((row_ini, col_ini), (row_goal, col_goal), mars_map, 10.0174)


# RESULTS
'''
# iterative_limited_depth_first
result_limted_depth = iterative_limited_depth_first(mars_problem,graph_search=True)

# Greedy Search
result_greedy = greedy(mars_problem,graph_search=True)

# BFS
result_bfs = breadth_first(mars_problem,graph_search=True)
'''
# A*
result_astar = astar(mars_problem,graph_search=True)



## Imprimiendo secuencia de acciones, costo y distancia

# A*
print("\nA*:")
if result_astar:
    path_astar = result_astar.path()
    cost_astar = result_astar.cost * 10.0174
    path_astar = [point for point in path_astar if None not in point]
    astar_points = convert_path_to_points(path_astar)
    print("A* Points:", astar_points)
    print("Cost:", cost_astar)
else:
    print("Unable to find a valid path.")

'''
# BFS
print("\nBFS:")
if result_bfs:
    path_bfs = result_bfs.path()
    price_bfs = result_bfs.cost * 10.0174
    path_bfs = [point for point in path_bfs if None not in point]
    bfs_points = convert_path_to_points(path_bfs)
    print("BFS Points:", bfs_points)
    print("Cost:", price_bfs)
else:
    print("Unable to find a valid path.")


# Greedy Search
print("\nGreedy Search:")
if result_greedy:
    path_greedy = result_greedy.path()
    cost_greedy = result_greedy.cost * 10.0174
    path_greedy = [point for point in path_greedy if None not in point]
    greedy_points = convert_path_to_points(path_greedy)
    print("Greedy Points:", greedy_points)
    print("Cost:", cost_greedy)
else:
    print("Unable to find a valid path.")
    
 
# Limited depth first
print("\nIterative limited depth first:")
if result_limted_depth:
    path_limited_depth = result_limted_depth.path()
    cost_limited_depth = result_limted_depth.cost * 10.0174
    path_limited_depth = [point for point in path_limited_depth if None not in point]
    limited_depth_points = convert_path_to_points(path_limited_depth)
    print("Limited Depth First Points:", limited_depth_points)
    print("Cost:", cost_limited_depth)
else:
    print("Unable to find a valid path.")
'''


## Generando mapa
if result_astar != None:
    path_x = []
    path_y = []
    path_z = []
    prev_state = []
    distance = 0
    for i, (action, state) in enumerate(result_astar.path()):    
        path_x.append( state[1] * scale  )            
        path_y.append(  (nr - state[0])*scale  )
        path_z.append(mars_map[state[0]][state[1]]+1)
        
        if len(prev_state) > 0:
            distance +=  math.sqrt(
            scale*scale*(state[0] - prev_state[0])**2 + scale*scale*(state[1] - prev_state[1])**2 + (
                mars_map[state[0], state[1]] - mars_map[prev_state[0], prev_state[1]])**2)

        prev_state = state

    print("Total distance: ", distance)

else:
    print("Unable to find a path between that connect the specified points")

## Plot results
if result_astar != None: 

    x = scale*np.arange(mars_map.shape[1])
    y = scale*np.arange(mars_map.shape[0])
    X, Y = np.meshgrid(x, y)

    fig = px.Figure(data = [px.Surface(x=X, y=Y, z=np.flipud(mars_map), colorscale='hot', cmin = 0, 
                                        lighting = dict(ambient = 0.0, diffuse = 0.8, fresnel = 0.02, roughness = 0.4, specular = 0.2),
                                        lightposition=dict(x=0, y=nr/2, z=2*mars_map.max())),
                        
                            px.Scatter3d(x = path_x, y = path_y, z = path_z, name='path', mode='markers',
                                            marker=dict(color=np.linspace(0, 1, len(path_x)), colorscale="Bluered", size=4))],
                
                    layout = px.Layout(scene_aspectmode='manual', 
                                        scene_aspectratio=dict(x=1, y=nr/nc, z=max(mars_map.max()/x.max(), 0.2)), 
                                        scene_zaxis_range = [0,mars_map.max()])
                    )
    fig.show() 


