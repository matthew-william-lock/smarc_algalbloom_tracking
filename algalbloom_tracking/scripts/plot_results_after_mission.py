from argparse import ArgumentParser
import h5py as h5
import numpy as np
import geopy.distance
import matplotlib.pyplot as plt

import gp4aes.plotter.mission_plotter as plot_mission
# import smarc_algal_bloom_tracking.plotter as plot_mission

# Setup plotting style
plt.style.reload_library()
plt.style.use(['science', 'no-latex'])
plt.rcParams.update({'xtick.labelsize': 20,
                    'ytick.labelsize': 20,
                    'axes.titlesize': 20,
                    'axes.labelsize': 20,
                    'legend.fontsize': 20,
                    'legend.frameon' : True
                    })

# Read runtime arguments
def parse_args():
    parser = ArgumentParser()
    parser.add_argument("path", type=str, help="Path to the HDF5 file containing the processed data.")
    parser.add_argument("--pdf", action='store_true', help="Save plots in pdf format")
    parser.add_argument("--prefix",  type=str, help="Name used as prefix when saving plots.")
    parser.add_argument('-z','--zoom', nargs='+', help='Zoom on a particlar region of the map [x0,y0,width,height]', \
        required=False,type=lambda s: [float(item) for item in s.split(',')])
    parser.add_argument('-t','--time', nargs='+', help='Specify the time range in hours for plotting', \
        required=False,type=lambda s: [float(item) for item in s.split(',')])
    return parser.parse_args()

# Parse in arguments
args = parse_args()

# Read h5 file
with h5.File(args.path, 'r') as f:
    lon = f["lon"][()]
    lat = f["lat"][()]
    chl = f["chl"][()]
    # time = f["time"][()]
    position = f["traj"][()]
    measurements = f["measurement_vals"][()]
    gradient = f["grad_vals"][()]
    chl_ref = f.attrs["delta_ref"]
    #time_step = f.attrs["time_step"]
    meas_per = f.attrs["meas_period"]
    t_idx = f.attrs["t_idx"]

# Fixing parameters
meas_per = int(meas_per)

# Set plot format
extension = 'png'
if args.pdf:
    extension = 'pdf'

# Set prefix for plot names
plot_name_prefix = ""
if args.prefix:
    plot_name_prefix = args.prefix + "-"

# Trim zeros and last entry so that all meas/grads are matched
position = position[~np.all(position == 0, axis=1)]
time_step = 1

############################################ PRINTS
# Attributes and length os variables
print("delta_ref :", chl_ref)
print("time_step :", time_step)
print("meas_per :", meas_per)
# print('len(position) ', len(position[:, 0])-1, ' len(grad) ', len(gradient[:, 0]), ' len(measurements) ', len(measurements))

print("Measurements shape: ", measurements.shape, " - Grad shape: ", gradient.shape)
print("Measurements shape: ", measurements, " - Grad shape: ", gradient)

# Average speed
distances_between_samples = np.array([])
for i in range(0,len(position[:, 0])-2):
    distance = geopy.distance.geodesic((position[i,1],position[i,0]), (position[i+1,1],position[i+1,0])).m
    distances_between_samples = np.append(distances_between_samples,distance)
print("Average speed: {} m/s".format(np.mean(distances_between_samples)))

############################################ PLOTS

# Call plotter class
plotter = plot_mission.Plotter(position, lon, lat, chl, gradient, measurements, chl_ref, meas_per, time_step)
    
# Create zoom 1 square
lat_start1 = 61.525
lat_end1 = 61.56
lon_start1 = 21.1
lon_end1 = 21.17

#a) Mission overview
fig_trajectory = plotter.mission_overview(lon_start1,lon_end1,lat_start1,lat_end1)
fig_trajectory.savefig("plots/{}{}.{}".format(plot_name_prefix, "big_map",extension),bbox_inches='tight')

######################## ZOOM 1
# Create zoom 1 time
zoom_start1 = 0
zoom_end1 = 366

#c) Gradient zoom1
fig_gradient = plotter.gradient_comparison(zoom_start1, zoom_end1)
fig_gradient.savefig("plots/{}{}.{}".format(plot_name_prefix, "gradient",extension),bbox_inches='tight')

#d) Chl zoom1
fig_chl = plotter.chl_comparison()
fig_chl.savefig("plots/{}{}.{}".format(plot_name_prefix, "measurements",extension),bbox_inches='tight', dpi=300)

# # Previous zoom square
# # lon_start2 = 21.142
# # lon_end2 = 21.152
# # lat_start2 = 61.534  
# # lat_end2 = 61.539
# # Create zoom 2 square
# lon_start2 = 21.113
# lon_end2 = 21.123
# lat_start2 = 61.527 
# lat_end2 = 61.532
# # Create zoom 3 square
# lon_start3 = 21.154
# lon_end3 = 21.164
# lat_start3 = 61.547
# lat_end3 = 61.552

# #b) Zoom1 map with gradient
# fig_zoom_gradient = plotter.zoom1(lon_start2,lon_end2,lat_start2,lat_end2,lon_start3,lon_end3,lat_start3,lat_end3)
# fig_zoom_gradient.savefig("plots/{}{}.{}".format(plot_name_prefix, "zoom1_map",extension),bbox_inches='tight')

# ######################## ZOOM 2
# # Create zoom 2 time
# zoom_start2 = 15890
# zoom_end2 = 16920

# #f) Control law zoom2
# fig_control = plotter.control_input(zoom_start2, zoom_end2)
# fig_control.savefig("plots/{}{}.{}".format(plot_name_prefix, "control",extension),bbox_inches='tight')

# #e) Zoom 2 map with control law 
# fig_zoom_control = plotter.zoom2(lon_start2,lon_end2,lat_start2,lat_end2)
# fig_zoom_control.savefig("plots/{}{}.{}".format(plot_name_prefix, "zoom2_map",extension),bbox_inches='tight', dpi=300)

# ######################## ZOOM 3
# # Create zoom 3 time 
# zoom_start3 = 21400
# zoom_end3 = 22440

# #h) Control law zoom 3
# fig_control = plotter.control_input(zoom_start3, zoom_end3)
# fig_control.savefig("plots/{}{}.{}".format(plot_name_prefix, "control",extension),bbox_inches='tight')

# #g) Zoom 3 map with control law 
# fig_zoom_control = plotter.zoom2(lon_start3,lon_end3,lat_start3,lat_end3)
# fig_zoom_control.savefig("plots/{}{}.{}".format(plot_name_prefix, "zoom2_map",extension),bbox_inches='tight', dpi=300)

plt.show()
