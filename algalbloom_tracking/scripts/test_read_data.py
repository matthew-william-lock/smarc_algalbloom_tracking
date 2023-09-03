from smarc_algal_bloom_tracking.util import read_mat_data

grid, idx = read_mat_data(1618610399, include_time=False,scale_factor=1, base_path="/home/jfgf/catkin_ws/src/smarc_algal_bloom_tracking/data")
print(grid.grid[0][0], " - ", grid.grid[1])