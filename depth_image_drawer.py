# read depth image from file and convert to point cloud

import numpy as np
import open3d as o3d
# from open3d.open3d.geometry import voxel_down_sample,estimate_normals

# Load the depth image data
depth_image = np.load('depth_image_drawer.npy')
print(depth_image.shape)

# Create a point cloud from the depth image
pcd = o3d.geometry.PointCloud()

# depth convert from mm to meters
depth_image = depth_image

masked_depth_image = o3d.geometry.Image(depth_image)

# intrinsic:[398.63726806640625, 0.0, 323.7835388183594, 0.0, 398.63726806640625, 241.52117919921875, 0.0, 0.0, 1.0]
intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, 398.63726806640625, 398.63726806640625, 323.7835388183594, 241.52117919921875)

# create a point cloud from the depth image
pcd = o3d.geometry.PointCloud.create_from_depth_image(masked_depth_image, intrinsic)

# scale the point cloud by 0.001 to get the correct size
# pcd.scale(0.001, center=pcd.get_center())

# filter the point cloud
# remove the points that are farther than 0.5 meters
# cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)

# downsample the point cloud to 4096 points
pcd = pcd.farthest_point_down_sample(4096)

# visualize the point cloud
# o3d.visualization.draw_geometries([pcd])




# save the point cloud to ply file
o3d.io.write_point_cloud('real_small_drawer.ply', pcd)

