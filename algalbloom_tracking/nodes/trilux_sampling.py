#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

import numpy as np
import signal
import rospy
from std_msgs.msg import Header
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt

# From other files
from smarc_algal_bloom_tracking.util import read_mat_data_offset, save_mission
from smarc_algal_bloom_tracking.publishers import publish_offset

# Msgs
from smarc_algal_bloom_tracking.msg import ChlorophyllSample
from ros_trilux_msgs.msg import Measurement

fig,ax = plt.subplots()

# Constants
GRADIENT_TOPIC = '/sam/algae_tracking/gradient'
VITUAL_POSITION_TOPIC = '/sam/algae_tracking/vp'
LIVE_WP_BASE_TOPIC = 'sam/smarc_bt/live_wp/'
WAPOINT_TOPIC=LIVE_WP_BASE_TOPIC+'wp'

class substance_sampler_node(object):

    def __init__(self):
        """ Init the sampler"""

        # Parameters
        self.update_period = rospy.get_param('~sampling_time')                  # Update period

        # Determine if data needs to be scaled
        self.delta_ref = rospy.get_param('~delta_ref')

        # Init values     
        self.init = False

        # Real position
        self.lat = None
        self.lon = None
        
        # Publishers and subscribers
        self.dr_sub                     = rospy.Subscriber('~gps_topic', NavSatFix, self.lat_lon__cb,queue_size=2)
        self.trilux_sub                = rospy.Subscriber('~trilux_topic', Measurement, self.trilux_cb,queue_size=1)
        
        self.chlorophyll_publisher      = rospy.Publisher('/sam/algae_tracking/measurement', ChlorophyllSample, queue_size=1)
        self.lat_lon_offset_publisher   = rospy.Publisher('/sam/algae_tracking/lat_lon_offset', GeoPointStamped, queue_size=2)
        self.map_offset_publisher       = rospy.Publisher('/sam/algae_tracking/map_offset', GeoPointStamped, queue_size=2)

        # Plotting
        self.grid_plotted = False

    def lat_lon__cb(self,fb):

        # Determine the offset of the GPS
        if not self.init:
            self.init = True

        # Get position
        if fb.latitude > 1e-6 and fb.longitude > 1e-6:
            self.lat = fb.latitude
            self.lon = fb.longitude
        else:
            rospy.logwarn("#PROBLEM# Received Zero GPS coordinates!")

    def trilux_cb(self,fb):
        
        # Get measurement
        measurement = fb.chlorophyll_a
        self.publish_sample(sample_value = measurement)

    def publish_sample(self, sample_value):
        """ Publish Chlorophyll Sample"""

        # Publish sample message
        sample = ChlorophyllSample()
        sample.header = Header()
        sample.header.stamp = rospy.Time.now()
        sample.lat = self.lat
        sample.lon = self.lon
        sample.sample = sample_value

        # Publish message
        rospy.loginfo('[Trilux Sampling] Publishing sample : {} at {},{}'.format(sample.sample,sample.lat,sample.lon))
        self.chlorophyll_publisher.publish(sample)

    def run_node(self):
        """ Start sampling """

        rate = rospy.Rate(float(1)/self.update_period)
        while not rospy.is_shutdown():
            self.publish_sample()
            rate.sleep()

    def close_node(self,signum, frame):
        """
        Kill node and save data
        """
        
        rospy.logwarn("Closing node")
        out_path = rospy.get_param('~output_data_path')

        # Get all relevant ros params
        self.all_params = rospy.get_param_names()
        self.tracking_params = [a for a in self.all_params if "sam_gp4aes_controller" in a]
        self.sampler_params = [a for a in self.all_params if "substance_sampler" in a]

        # track_params
        track_params= {}
        for key in self.tracking_params:
            track_params[key] = rospy.get_param(key)

        # sample_params
        sample_params = {}
        for key in self.sampler_params:
            print("key: ", key)
            sample_params[key] = None
            print("Param: ", rospy.get_param(key))
            sample_params[key] = rospy.get_param(key)
        
        #print('meas_per at simulated_chl_sampler is ', self.update_period)

        try :
            save_mission(out_path=out_path,grid=self.grid,meas_per=self.update_period,sample_params=sample_params,track_params=track_params, t_idx=self.t_idx)
            rospy.logwarn("Data saved!")
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Failed to save data")

        exit(1)

if __name__ == '__main__':

    rospy.init_node("substance_sampler")
    sampler = substance_sampler_node()

    # Attach exit handler
    signal.signal(signal.SIGINT, sampler.close_node)

    sampler.run_node()
    rospy.signal_shutdown()
        