import folium
import numpy as np
from pyproj import Transformer

_transformer_cache = {}

def visualize_path(full_path, points, start_idx, target_nodes, epsg):

    if not full_path:
        print("Warning: path is empty, nothing to visualize.")
        return

    # Cache the transformer so repeated calls (or future real-time use)
    # don't rebuild it from scratch each time
    if epsg not in _transformer_cache:
        _transformer_cache[epsg] = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)
    transformer = _transformer_cache[epsg]

    def node_to_latlon(node_idx):
        # look up the XYZ coordinates of this node in the points array
        x, y, z = points[node_idx]
        # transform from LAZ projection to lon/lat, then swap to (lat, lon)
        # because folium expects (lat, lon) not (lon, lat)
        lon, lat = transformer.transform(x, y)
        return lat, lon

    # ---------- Build flat list of path coords ----------
    # full_path is already a flat list of node indices, so just convert each one
    path_coords = [node_to_latlon(idx) for idx in full_path]

    # ---------- Build map ----------
    # center the map on the first point in the path (the start)
    # zoom_start=17 gives a close enough view to see individual terrain features
    # Esri.WorldImagery gives real satellite imagery as the base map
    m = folium.Map(location=path_coords[0], zoom_start=17)
    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri",
        name="Esri Satellite"
    ).add_to(m)

    # ---------- Draw path ----------
    folium.PolyLine(path_coords, color="red", weight=3, opacity=0.9).add_to(m)

    # ---------- Start marker ----------
    start_lat, start_lon = node_to_latlon(start_idx)
    folium.CircleMarker(
        (start_lat, start_lon),
        radius=7, color="white", fill=True,
        fill_color="lime", fill_opacity=1,
        tooltip="Start"
    ).add_to(m)

    # ---------- Target markers ----------
    # target_nodes is a list of dicts with "index" and "point" keys
    for i, target in enumerate(target_nodes):
        lat, lon = node_to_latlon(target["index"])
        folium.CircleMarker(
            (lat, lon),
            radius=7, color="white", fill=True,
            fill_color="orange", fill_opacity=1,
            tooltip=f"Target {i+1}"
        ).add_to(m)

    # ---------- Save ----------
    output_path = "path_output.html"
    m.save(output_path)
    print(f"Map saved to {output_path}, open it in your browser to view the path.")