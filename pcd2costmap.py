import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy import ndimage
from PIL import Image


# Load point cloud
pcd = o3d.io.read_point_cloud("scans.pcd")

# Downsample to improve processing speed
pcd = pcd.voxel_down_sample(voxel_size=0.02)

# Estimate normals for better plane segmentation
pcd.estimate_normals()

# Remove ground plane using RANSAC
plane_model, inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
a, b, c, d = plane_model

# Get non-ground points (i.e., obstacles/walls)
norm = np.sqrt(a**2 + b**2 + c**2)
points = np.asarray(pcd.points)
distances = (points[:, 0] * a + points[:, 1] * b + points[:, 2] * c + d) / norm

# Create a mask for points above the plane
mask = distances >= 0.1
filtered_points = points[mask]

# Create a new point cloud with filtered points
obstacles = o3d.geometry.PointCloud()
obstacles.points = o3d.utility.Vector3dVector(filtered_points)

# Statistical outlier removal to clean the point cloud
obstacles, _ = obstacles.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Visualize filtered point cloud
filtered_points = np.asarray(obstacles.points)

# Define costmap parameters
resolution = 0.05  # meters per pixel
padding = 0.1      # obstacle padding in meters (for inflation)

# Determine costmap boundaries from point cloud
x_min, y_min = np.min(filtered_points[:, 0:2], axis=0) - padding
x_max, y_max = np.max(filtered_points[:, 0:2], axis=0) + padding

# Calculate costmap dimensions
width = int((x_max - x_min) / resolution)
height = int((y_max - y_min) / resolution)

# Create empty costmap (0 = free space, 100 = occupied, -1 = unknown)
costmap = np.zeros((height, width), dtype=np.int8)

# Project points onto 2D grid and mark as obstacles
for point in filtered_points:
    x, y = point[0:2]
    # Convert world coordinates to grid coordinates
    grid_x = int((x - x_min) / resolution)
    grid_y = int((y - y_min) / resolution)
    
    # Ensure within bounds
    if 0 <= grid_x < width and 0 <= grid_y < height:
        costmap[height - 1 - grid_y, grid_x] = 100  # Flip y-axis for image coordinates

# Apply simple obstacle inflation (dilation)
obstacle_mask = (costmap == 100)
inflation_size = int(padding / resolution)
inflated_obstacles = ndimage.binary_dilation(obstacle_mask, iterations=inflation_size)
costmap[inflated_obstacles] = 99  # Inflated area
# Create and save PGM file
pgm_file = "costmap.pgm"
with open(pgm_file, 'wb') as f:
    # PGM header
    f.write(b"P5\n")
    f.write(f"# Created from point cloud\n".encode())
    f.write(f"{width} {height}\n".encode())
    f.write(b"255\n")
    



    # Convert costmap to correct format
    output_array = np.zeros((height, width), dtype=np.uint8)
    output_array[costmap == 0] = 254    # Free space
    output_array[costmap == 99] = 0   # Inflation area
    output_array[costmap == 100] = 0    # Obstacle
    # image = Image.fromarray(output_array)
    # image = image.rotate(90)
    # output_array = np.array(image)
    # Write binary data
    f.write(output_array.tobytes())

# Create and save YAML metadata
yaml_file = "costmap.yaml"
with open(yaml_file, 'w') as f:
    f.write(f"image: {pgm_file}\n")
    f.write(f"resolution: {resolution}\n")
    f.write(f"origin: [{x_min}, {y_min}, 0.0]\n")
    f.write(f"occupied_thresh: 0.65\n")
    f.write(f"free_thresh: 0.196\n")
    f.write(f"negate: 0\n")

# Visualize the costmap
plt.figure(figsize=(10, 10))
plt.imshow(output_array, cmap='gray', origin='lower')
plt.colorbar(label='Cost')
plt.title('Generated Costmap')
plt.savefig("costmap_visualization.png")
print(f"Costmap saved as {pgm_file} with metadata in {yaml_file}")

