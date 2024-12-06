import osmnx as ox
from shapely.geometry import box

# Define the area of interest (bounding box for University of Minnesota campus)
north, south, east, west = 44.98795, 44.96187, -93.25482, -93.21358

try:
    # Create a bounding box polygon using Shapely
    bbox_polygon = box(west, south, east, north)

    # Download the graph for walking paths, streets, etc.
    graph = ox.graph_from_polygon(bbox_polygon, network_type="walk")

    # Check if the graph contains edges
    if graph.number_of_edges() == 0:
        raise ValueError("The graph contains no edges. Try a different area or network type.")

    # Visualize the graph
    ox.plot_graph(graph, bgcolor='k', node_color='r', edge_color='w', node_size=10)

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





