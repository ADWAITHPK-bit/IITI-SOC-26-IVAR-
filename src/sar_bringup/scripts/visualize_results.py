#!/usr/bin/env python3
import csv
import os
import math
import numpy as np
import folium
import matplotlib.pyplot as plt
import matplotlib.patches as patches
# A matplotlib showing the 20*20 m search area,the lawnmover path and dots where people are detected
# A folium HTML interactive map saved to ~/soc_ws/results_map.html

def load_geotag(filepath):
    geotags = [] 
    with open(filepath, mode='r', newline='', encoding='utf-8') as file:
        csv_reader = csv.reader(file)

        next(csv_reader) # Skip the header row

        for row in csv_reader:
            geotag = { # Only loading the header with these names
                "id": row[0],
                "x_ned": float(row[1]),
                "y_ned": float(row[2]),
                "latitude": float(row[3]),
                "longitude": float(row[4]),
                "confidence": float(row[5]),
                "timestamp": row[6]
                }

            geotags.append(geotag)
    return geotags

def plot_coverage_map(geotags):
    fig, ax = plt.subplots(1, 1, figsize=(10,10)) # An empty figure with axes
    ax.set_aspect('equal') # Set axis equal

    rect = patches.Rectangle((-10,-10), 20, 20,
                     facecolor='grey',
                     edgecolor='black',
                     linewidth=2,
                     alpha=0.5)
     # Bottom-left corner - (-10, -10), 20 width and 20 height
    ax.add_patch(rect)

    path_x = []
    path_y = []

    y = -10.0
    strip_num = 0

    while y <= 10.0:

        if strip_num % 2 == 0:
            path_x.append(-10)
            path_x.append(+10)

            path_y.append(y)
            path_y.append(y)

        else:
            path_x.append(+10)
            path_x.append(-10)

            path_y.append(y)
            path_y.append(y)

        y = y + 4.0
        strip_num = strip_num + 1


    ax.plot(path_x, path_y, 'b-', linewidth=1,label='Coverage Path', alpha=0.7)
    x_vals = [g['x_ned'] for g in geotags] #Extracts x values
    y_vals = [g['y_ned'] for g in geotags]
    ax.scatter(x_vals, y_vals, c='red', s=100, zorder=5, label='Detected Person')

    # Label and formatting
    ax.set_xlabel('X NED (meters)')
    ax.set_ylabel('Y NED (meters)')
    ax.set_title('SAR Mission Results')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-12, 12)
    ax.set_ylim(-12, 12)

    # Save and show
    output_path = os.path.expanduser('~/soc_ws/results_map.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close() # close instead of show - non-blocking
    print(f'Results map saved to {output_path}')

def create_folium_map(geotags):
    m = folium.Map(location=[47.397742, 8.545594], zoom_start=19, 
               tiles='OpenStreetMap')
    
    for g in geotags:
        folium.CircleMarker(
            location=[g['latitude'], g['longitude']],
            radius=8,
            color='red',
            fill=True,
            fill_color='red',
            fill_opacity=0.7,
            popup=f"Confidence: {g['confidence']:.2f}\nTimestamp: {g['timestamp']}"
        ).add_to(m)

    path_gps = []
    home_lat = 47.397742
    home_lon = 8.545594
    y_strip = -10.0
    strip_num = 0
    while y_strip <= 10.0:
        if strip_num % 2 == 0:
            x_vals_strip = [-10.0, 10.0]
        else:
            x_vals_strip = [10.0, -10.0]
        for x_strip in x_vals_strip:
            lat = home_lat + (y_strip / 111111.0)
            lon = home_lon + (x_strip / (111111.0 * math.cos(math.radians(home_lat))))
            path_gps.append([lat, lon])
        y_strip += 4.0
        strip_num += 1

    folium.PolyLine(
        path_gps,
        color='blue',
        weight=2,
        opacity=0.7,
        tooltip='Coverage Path'
    ).add_to(m)

    title_html = '''
    <h3 style="text-align:center">
    SAR Mission Results (NED coordinates)
    </h3>
    '''

    m.get_root().html.add_child(folium.Element(title_html))

    output_path = os.path.expanduser('~/soc_ws/results_map.html')
    m.save(output_path)

    print(f'Folium map saved to {output_path}')

def calculate_coverage_percentage():
    search_area = 400.0
    altitude = 8.0
    hfov = 1.3962634
    footprint_width = 2 * altitude * math.tan(hfov / 2.0) # camera footprint width at 8m
    num_strips = 6 # we have 6 strips from y=-10 to y=+10 stepping by 4
    strip_length = 20.0
    covered_area = num_strips * strip_length * footprint_width
    percentage = min((covered_area / search_area) * 100, 100.0)
    print(f'Coverage: {percentage:.1f}%')
    return percentage

def main():
    geotags = load_geotag(os.path.expanduser('~/soc_ws/geotags.csv'))
    plot_coverage_map(geotags)
    create_folium_map(geotags)
    percentage = calculate_coverage_percentage()
    print(f'Total geotags: {len(geotags)}')
    print(f'Coverage percentage: {percentage:.1f}%')

if __name__ == '__main__':
    main()