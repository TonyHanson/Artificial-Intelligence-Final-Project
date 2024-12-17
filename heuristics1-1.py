import osmnx as ox
import matplotlib.pyplot as plt  # Import pyplot for figure control
from shapely.geometry import box
import time  # Import time module for measuring execution time
import heapq
import math
import random

# Define the area of interest (bounding box for University of Minnesota campus)
north, south, east, west = 44.98795, 44.96187, -93.25482, -93.21358

# Define the specific coordinates (latitude, longitude) for the restaurants
red_nodes_coords = {
    "McDonald's": (44.98032, -93.23443),
    "Raising Cane's": (44.97905, -93.23485),
    "Blue Door": (44.98772, -93.23002),
    "Domino's": (44.97419, -93.22676),
    "Chick-fil-A": (44.97404, -93.22921),
    "Pho Mai": (44.98017, -93.23638),
    "Frank & Andrea": (44.98130, -93.23769),
    "Blaze Pizza": (44.97347, -93.22340),
    "Buffalo Wild Wings": (44.97600, -93.22673),
    "The Corner Bar": (44.97283, -93.24766),
    "Maxwell's Cafe and Grill": (44.98385, -93.24377),
    "Alma": (44.98385, -93.24805),
    "Potter's Pasties & Pies": (44.98762, -93.22586),
    "Fred's Chicken 'N' Waffles": (44.96904, -93.24523),
    "Davanni's Pizza and Hot Hoagies": (44.96562, -93.23593),
}

# Define the coordinates for the starting node
starting_node_coords = (44.97303, -93.23532)  # New starting point
# Define heuristic functions
def heuristic_euclidean(u, v, graph):
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    return ((u_coords[0] - v_coords[0]) ** 2 + (u_coords[1] - v_coords[1]) ** 2) ** 0.5

def heuristic_manhattan(u, v, graph):
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    return abs(u_coords[0] - v_coords[0]) + abs(u_coords[1] - v_coords[1])

def heuristic_zero(u, v, graph):
    return 0  # Acts as Dijkstra's algorithm


scenic_locations = [
     (44.977034, -93.227070),  # Stadium
     (44.973194471442845, -93.23705506708603),  # Weisman Art Museum
     (44.976244845144564, -93.2353631884106),   # Northrup Auditorium
     (44.975112, -93.240431), #bohemian flats 
     (44.982340, -93.236528) #dinky


]

def heuristic_scenic(u, v, graph):
    """
    A non-admissible heuristic that prioritizes passing by scenic locations.
    Encourages revisiting nodes to generate a scenic path.
    
    Parameters:
    - u: Current node
    - v: Goal node
    - graph: The graph where nodes have 'x' and 'y' coordinates
    - scenic_locations: List of scenic locations [(y1, x1), (y2, x2), ...]
    
    Returns:
    - A heuristic value that balances direct distance to the goal and scenic attraction.
    """
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    direct_distance = ((u_coords[0] - v_coords[0]) ** 2 + (u_coords[1] - v_coords[1]) ** 2) ** 0.5
    scenic_weight = 0
    for scenic_loc in scenic_locations:
        scenic_dist = ((u_coords[0] - scenic_loc[0]) ** 2 + (u_coords[1] - scenic_loc[1]) ** 2) ** 0.5
        scenic_weight += max(0, 1.0 / (scenic_dist * 0.001))  # Use an inverse distance to add scenic attraction
    scenic_bias = -5000  # Controls how much the scenic route is favored (tune this parameter)
    total_heuristic = direct_distance + scenic_bias * scenic_weight
    return total_heuristic
#
#

def heuristic_random(u, v, graph):
    """
    A random heuristic that introduces unpredictability in the pathfinding process.
    
    Parameters:
    - u: Current node
    - v: Goal node
    - graph: The graph where nodes have 'x' and 'y' coordinates
    
    Returns:
    - A random heuristic value.
    """
    # Coordinates of current node (u) and goal node (v)
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    
    # 1️⃣ Direct Euclidean distance from u to v (for a baseline)
    direct_distance = ((u_coords[0] - v_coords[0]) ** 2 + (u_coords[1] - v_coords[1]) ** 2) ** 0.5
    
    # 2️⃣ Add a random value to the heuristic to introduce unpredictability
    random_factor = random.uniform(0, 1000000)  # Random value between 0 and 10
    
    # 3️⃣ Combine direct distance with random factor
    total_heuristic = direct_distance - random_factor
    
    return total_heuristic
#
#
#
#


def Astar(graph, start, goal, heuristic):
    """A* search to find the shortest path in a graph."""
    open_set = []  # Priority queue for open nodes
    heapq.heappush(open_set, (0, start))  # Push the start node into the queue
    
    came_from = {}  # To reconstruct the path later
    g_score = {node: float('inf') for node in graph.nodes}  # Distance from start to each node
    g_score[start] = 0
    
    f_score = {node: float('inf') for node in graph.nodes}  # Estimated distance from start to goal
    f_score[start] = heuristic(start, goal, graph)

    visited_nodes_count = 0 # Initialize counter for visited nodes
    
    while open_set:
        current_f, current_node = heapq.heappop(open_set)
        
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            return list(reversed(path)), visited_nodes_count  # Return the path and the count of visited nodes
        
        for neighbor in graph.neighbors(current_node):
            tentative_g_score = g_score[current_node] + graph[current_node][neighbor][0]['length']
            
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal, graph)
                
                if neighbor not in [item[1] for item in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        visited_nodes_count += 1
    
    return float('inf')  # Return infinity if no path is found

heuristics = {
    "1": ("Euclidean Distance", heuristic_euclidean),
    "2": ("Manhattan Distance", heuristic_manhattan),
    "3": ("No Heuristic (Dijkstra)", heuristic_zero),
    "4": ("Scenic Route Heuristic", heuristic_scenic),
    "5": ("heuristic_random", heuristic_random)
}

try:
    bbox_polygon = box(west, south, east, north)
    graph = ox.graph_from_polygon(bbox_polygon, network_type="walk")

    if graph.number_of_edges() == 0:
        raise ValueError("The graph contains no edges. Try a different area or network type.")

    starting_node = ox.nearest_nodes(graph, X=starting_node_coords[1], Y=starting_node_coords[0])

    scenic_nodes = []
    for coords in scenic_locations:
        node = ox.nearest_nodes(graph, X=coords[1], Y=coords[0])
        scenic_nodes.append(node)

    while True:
        # Ask the user to select a restaurant
        print("\nSelect a restaurant to find the shortest path to, or type 'exit' to quit:")
        for i, restaurant in enumerate(red_nodes_coords.keys(), start=1):
            print(f"{i}. {restaurant}")

        user_input_restaurant = input("Enter the number corresponding to the restaurant or 'exit': ").strip().lower()

        if user_input_restaurant == "exit":
            print("Exiting the program. Goodbye!")
            break

        if not user_input_restaurant.isdigit() or int(user_input_restaurant) < 1 or int(user_input_restaurant) > len(red_nodes_coords):
            print("Invalid input. Please enter a valid number or 'exit'.")
            continue

        selected_index = int(user_input_restaurant)
        restaurant_names = list(red_nodes_coords.keys())
        selected_restaurant = restaurant_names[selected_index - 1]
        destination_coords = red_nodes_coords[selected_restaurant]
        GOAL = destination_coords = red_nodes_coords[selected_restaurant]

        # Ask the user if they want to use all heuristics
        print("\nSelect an option:")
        print("1. Use a single heuristic")
        print("2. Use all three heuristics and compare results")

        user_input_option = input("Enter the number corresponding to the option: ").strip()

        if user_input_option == "1":
            # Ask the user to select a heuristic
            print("\nSelect a heuristic for the A* search:")
            for key, (name, _) in heuristics.items():
                print(f"{key}. {name}")
            user_input_heuristic = input("Enter the number corresponding to the heuristic: ").strip()
            if user_input_heuristic not in heuristics:
                print("Invalid heuristic selection. Please try again.")
                continue
            heuristic_name, heuristic_function = heuristics[user_input_heuristic]
            # Find the nearest node to the selected restaurant's coordinates
            destination_node = ox.nearest_nodes(graph, X=destination_coords[1], Y=destination_coords[0])
            # Start measuring time
            start_time = time.time()
            # Find the shortest path from the starting node to the destination node using A*
            try:
                path, visited_nodes = Astar(graph, starting_node, destination_node, heuristic=heuristic_function)

                # End measuring time
                end_time = time.time()

                # Calculate the time taken for the pathfinding process with higher precision
                time_taken = end_time - start_time
                print(f"\nShortest path to {selected_restaurant} using {heuristic_name} visualized successfully.")
                print(f"Time taken for pathfinding: {time_taken:.8f} seconds.")  # More significant digits
                print(f"Total number of nodes visited: {visited_nodes}")

                # Generate the map when using a single heuristic
                node_color = ['lightgreen' if node == starting_node else ('r' if node == destination_node else ('purple' if node in scenic_nodes else 'gray'))for node in graph.nodes]
                node_size = [35 if node == starting_node or node == destination_node or node in scenic_nodes else 0 for node in graph.nodes]
                path_edges = list(zip(path[:-1], path[1:]))
                edge_color = ['b' if (u, v) in path_edges or (v, u) in path_edges else 'w' for u, v, k in graph.edges]
                
                

                # Plot the graph without blocking the input loop
                fig, ax = ox.plot_graph(
                    graph,
                    bgcolor='k',
                    node_color=node_color,
                    edge_color=edge_color,
                    node_size=node_size,
                    edge_linewidth=[3 if (u, v) in path_edges or (v, u) in path_edges else 0.5 for u, v, k in graph.edges],
                    figsize=(20, 16),
                    dpi=100,
                    show=False  # Do not block execution with the plot
                )

                plt.show(block=False)  # Show the plot in non-blocking mode
                plt.pause(0.001)       # Allow rendering of the plot

                print("\n")
                print("-----------------------------------------------------------------------------------------")
                print("-----------------------------------------------------------------------------------------")
                print("\n")

            except Exception as ex:
                print(f"An error occurred during pathfinding: {ex}")

        elif user_input_option == "2":
            # Compare all heuristics
            print("\nComparing results using all three heuristics:")

            destination_node = ox.nearest_nodes(graph, X=destination_coords[1], Y=destination_coords[0])

            for key, (heuristic_name, heuristic_function) in heuristics.items():
                # Start measuring time
                start_time = time.time()

                # Find the shortest path from the starting node to the destination node using A*
                try:
                    path, visited_nodes = Astar(graph, starting_node, destination_node, heuristic=heuristic_function)

                    # End measuring time
                    end_time = time.time()

                    # Calculate the time taken for the pathfinding process with higher precision
                    time_taken = end_time - start_time

                    # Print the results for each heuristic
                    print(f"\n{heuristic_name} Heuristic:")
                    print(f"Time taken for pathfinding: {time_taken:.8f} seconds.")  # More significant digits
                    print(f"Total number of nodes visited: {visited_nodes}")

                except Exception as ex:
                    print(f"An error occurred during pathfinding with {heuristic_name} heuristic: {ex}")

            print("\nComparison complete.")
            print("\n-----------------------------------------------------------------------------------------")
            print("-----------------------------------------------------------------------------------------")
            print("\n")

        else:
            print("Invalid option. Please try again.")

except Exception as e:
    print(f"An error occurred: {e}")
