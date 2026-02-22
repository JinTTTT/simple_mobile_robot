import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw

# Map Settings
resolution = 0.05  # meters per pixel
width_m = 14.0     # map width in meters (X axis)
height_m = 20.0    # map height in meters (Y axis)

img_width = int(width_m / resolution)
img_height = int(height_m / resolution)

# Create a white image (254 is standard for free space in ROS)
img = Image.new('L', (img_width, img_height), color=254)
draw = ImageDraw.Draw(img)

# Define the center of the image as Gazebo's (0,0)
cx = img_width / 2
cy = img_height / 2

# Parse the SDF file ../../simulation/models/simple_room/model.sdf
tree = ET.parse('../../simulation/models/simple_room/model.sdf')
root = tree.getroot()

for link in root.findall('.//link'):
    pose = link.find('.//pose')
    size = link.find('.//box/size')
    
    if pose is not None and size is not None:
        px, py, pz, ro, pi, ya = map(float, pose.text.split())
        sx, sy, sz = map(float, size.text.split())
        
        # Calculate bounding box coordinates in meters
        tl_x = px - (sx / 2)
        tl_y = py + (sy / 2)
        br_x = px + (sx / 2)
        br_y = py - (sy / 2)
        
        # Convert meters to image pixels (Note: Image Y-axis is inverted)
        pix_tl_x = int(cx + (tl_x / resolution))
        pix_tl_y = int(cy - (tl_y / resolution))
        pix_br_x = int(cx + (br_x / resolution))
        pix_br_y = int(cy - (br_y / resolution))
        
        # Draw the wall as a black rectangle (0 is standard for obstacles)
        draw.rectangle([pix_tl_x, pix_tl_y, pix_br_x, pix_br_y], fill=0)

# Save the generated map to ../maps/maze_map.pgm
img.save('../maps/maze_map.pgm')
print("maze_map.pgm generated successfully.")