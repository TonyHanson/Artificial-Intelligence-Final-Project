import osmnx as ox
import matplotlib.pyplot as plt  # Import pyplot for figure control
from shapely.geometry import box
from networkx import astar_path_length
import time  # Import time module for measuring execution time

'''
This file shows the shortest path (with heuristic) from Coffman to a selected restaurant using A* search.
'''

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

heuristics = {
    "1": ("Euclidean Distance", heuristic_euclidean),
    "2": ("Manhattan Distance", heuristic_manhattan),
    "3": ("No Heuristic (Dijkstra)", heuristic_zero),
}

try:
    # Create a bounding box polygon using Shapely
    bbox_polygon = box(west, south, east, north)
    # Download the graph for walking paths, streets, etc.
    graph = ox.graph_from_polygon(bbox_polygon, network_type="walk")

    # Check if the graph contains edges
    if graph.number_of_edges() == 0:
        raise ValueError("The graph contains no edges. Try a different area or network type.")

    # Find the nearest node to the starting coordinates
    starting_node = ox.nearest_nodes(graph, X=starting_node_coords[1], Y=starting_node_coords[0])

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
                path_length = astar_path_length(graph, starting_node, destination_node, heuristic=lambda u, v: heuristic_function(u, v, graph))
                path = ox.shortest_path(graph, starting_node, destination_node, weight='length')

                # End measuring time
                end_time = time.time()

                # Calculate the time taken for the pathfinding process with higher precision
                time_taken = end_time - start_time
                print(f"\nShortest path to {selected_restaurant} using {heuristic_name} visualized successfully.")
                print(f"Path length: {path_length:.2f} meters.")
                print(f"Time taken for pathfinding: {time_taken:.8f} seconds.")  # More significant digits

                # Generate the map when using a single heuristic
                node_color = ['lightgreen' if node == starting_node else ('r' if node == destination_node else 'gray') for node in graph.nodes]
                node_size = [35 if node == starting_node or node == destination_node else 0 for node in graph.nodes]
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
                    path_length = astar_path_length(graph, starting_node, destination_node, heuristic=lambda u, v: heuristic_function(u, v, graph))
                    path = ox.shortest_path(graph, starting_node, destination_node, weight='length')

                    # End measuring time
                    end_time = time.time()

                    # Calculate the time taken for the pathfinding process with higher precision
                    time_taken = end_time - start_time

                    # Print the results for each heuristic
                    print(f"\n{heuristic_name} Heuristic:")
                    print(f"Path length: {path_length:.2f} meters.")
                    print(f"Time taken for pathfinding: {time_taken:.8f} seconds.")  # More significant digits

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







