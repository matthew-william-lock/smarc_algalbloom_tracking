import h5py as h5
import numpy as np
import pandas as pd
import bagpy
from bagpy import bagreader

b = bagreader('../output/Rosbag_11Aug.bag')

traj = b.message_by_topic(topic='/sam/dr/lat_lon')
chl = b.message_by_topic(topic='/sam/algae_tracking/chlorophyll_sampling')
grad = b.message_by_topic(topic='/sam/algae_tracking/gradient')
lat_lon_offset = b.message_by_topic(topic='/sam/algae_tracking/lat_lon_offset')

traj_data = pd.read_csv(traj)
chl_data = pd.read_csv(chl)
grad_data = pd.read_csv(grad)
lat_lon_offset_data = pd.read_csv(lat_lon_offset)

# Get numpy arrays
traj_data_np = traj_data.values
traj_time = traj_data_np[:, 0]
traj_lat = traj_data_np[:, 1]
traj_lon = traj_data_np[:, 2]

# Get Chl Data
chl_data_np = chl_data.values
chl_time = chl_data_np[:, 0]
chl_sample = chl_data_np[:, -1].reshape(-1, 1)
chl_sample_lat = chl_data_np[:, -3]
chl_sample_lon = chl_data_np[:, -2]

grad_data_np = grad_data.values
grad_time = grad_data_np[:, 0]
grad_sample = np.hstack((
    grad_data_np[:, -2].reshape(-1, 1), grad_data_np[:, -1].reshape(-1, 1)))
grad_sample_lat = grad_data_np[:, -4]
grad_sample_lon = grad_data_np[:, -3]

traj = np.hstack((traj_lon.reshape(-1, 1), traj_lat.reshape(-1, 1)))

with h5.File('../output/scaled_mission.m5', 'r') as f:
    lon_m5 = f["lon"][()]
    lat_m5 = f["lat"][()]
    chl_m5 = f["chl"][()]

# Save H5 file
print("Trajectory shape: ", traj.shape)
print("Chlorophyl Sample Measured shape: ", chl_sample.shape)
print("Chlorophyl Time shape: ", chl_time.shape)
print("Gradient Sample shape: ", grad_sample.shape)

with h5.File("../output/experiment_11Aug.h5", 'w') as f:
    f.create_dataset("traj", data=traj)
    # Comes from data file
    f.create_dataset("chl", data=chl_m5)
    f.create_dataset("lon", data=lon_m5)
    f.create_dataset("lat", data=lat_m5)
    f.create_dataset("time", data=chl_time)
    f.create_dataset("measurement_vals", data=chl_sample)
    f.create_dataset("grad_vals", data=grad_sample)
    f.attrs.create("t_idx", data=3)
    f.attrs.create("delta_ref", data=7.45)
    f.attrs.create("meas_period", data=10.0)
