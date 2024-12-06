import osmnx as ox
from shapely.geometry import box

# Define the area of interest (bounding box for University of Minnesota campus)
north, south, east, west = 44.98795, 44.96187, -93.25482, -93.21358

# Define the specific coordinates (latitude, longitude) for the restuarants
red_nodes_coords = [
    (44.98032, -93.23443),  # McDonald's
    (44.97905, -93.23485),  # Raising Cane's
    (44.98772, -93.23002),  # Blue Door
    (44.97419, -93.22676),  # Domino's
    (44.97404, -93.22921),  # Chick-fil-A
    (44.98017, -93.23638),  # Pho Mai
    (44.98130, -93.23769),  # Frank & Andrea
    (44.97347, -93.22340),  # Blaze Pizza
    (44.97600, -93.22673),  # Buffalo Wild Wings
    (44.97283, -93.24766),  # The Corner Bar
    (44.98385, -93.24377),  # Maxwell's Cafe and Grill
    (44.98385, -93.24805),  # Alma
    (44.98762, -93.22586),  # Potter's Pasties & Pies
    (44.96904, -93.24523),  # Fred's Chicken 'N' Waffles
    (44.96562, -93.23593),  # Davanni's Pizza and Hot Hoagies
]

# Define the coordinates for the starting node
starting_node_coords = (44.97303, -93.23532)  # New starting point

try:
    # Create a bounding box polygon using Shapely
    bbox_polygon = box(west, south, east, north)

    # Download the graph for walking paths, streets, etc.
    graph = ox.graph_from_polygon(bbox_polygon, network_type="walk")

    # Check if the graph contains edges
    if graph.number_of_edges() == 0:
        raise ValueError("The graph contains no edges. Try a different area or network type.")

    # Find the nearest nodes to the coordinates of the 5 red nodes
    red_nodes = [ox.nearest_nodes(graph, X=lon, Y=lat) for lat, lon in red_nodes_coords]

    # Find the nearest node to the starting coordinates
    starting_node = ox.nearest_nodes(graph, X=starting_node_coords[1], Y=starting_node_coords[0])

    # Visualize the graph with the red nodes and starting node highlighted
    # Color all nodes gray, except the red nodes and starting node
    node_color = ['lightgreen' if node == starting_node else ('r' if node in red_nodes else 'gray') for node in graph.nodes]

    # Make red nodes and starting node bigger
    node_size = [35 if node == starting_node else (35 if node in red_nodes else 0) for node in graph.nodes]

    # Plot the graph
    ox.plot_graph(graph, bgcolor='k', node_color=node_color, edge_color='w', node_size=node_size)

    # Save the graph to a file in GraphML format
    ox.save_graphml(graph, filepath="umn_campus.graphml")

    # Convert the graph to GeoDataFrames
    nodes, edges = ox.graph_to_gdfs(graph)

    # Save the nodes as a GeoJSON file
    nodes.to_file("umn_campus_nodes.geojson", driver="GeoJSON")

    # Save the edges as a GeoJSON file
    edges.to_file("umn_campus_edges.geojson", driver="GeoJSON")

    print("Map created successfully. Files saved: umn_campus.graphml, umn_campus_nodes.geojson, umn_campus_edges.geojson.")

except Exception as e:
    print(f"An error occurred: {e}")













