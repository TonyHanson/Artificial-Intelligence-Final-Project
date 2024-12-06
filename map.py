import osmnx as ox
from shapely.geometry import box

# Define the area of interest (bounding box for University of Minnesota campus)
north, south, east, west = 44.98795, 44.96187, -93.25482, -93.21358

# Define the specific coordinates (latitude, longitude) for the 5 red nodes
red_nodes_coords = [
    (44.98032, -93.23443),  # McDonald's
    (44.97490, -93.23320),  # Example point 1
    (44.98110, -93.23750),  # Example point 2
    (44.97560, -93.22900),  # Example point 3
    (44.97050, -93.23200),  # Example point 4
]

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

    # Visualize the graph with the red nodes highlighted
    # Color all nodes gray, except the red nodes
    node_color = ['r' if node in red_nodes else 'gray' for node in graph.nodes]

    # Plot the graph
    ox.plot_graph(graph, bgcolor='k', node_color=node_color, edge_color='w', node_size=10)

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













