#! /usr/bin/env python3
import numpy as np
import rospy
import signal

# Smarc imports
from std_msgs.msg import Bool
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix
from smarc_msgs.msg import GotoWaypoint, GotoWaypointActionResult, ChlorophyllSample, AlgaeFrontGradient
from smarc_msgs.srv import LatLonToUTM

# GP4AES imports
import gp4aes.estimator.GPR as gpr
import gp4aes.controller.front_tracking as controller

# Publishers & utils
from smarc_algal_bloom_tracking.publishers import publish_waypoint
from smarc_algal_bloom_tracking.util import save_raw_mission_data

class FrontTracking(object):

    def __init__(self):
        self.delta_ref = rospy.get_param('~delta_ref')                                     # target chlorophyll value
        self.wp_distance = rospy.get_param('~wp_distance')                                 # wp_distance [m]
        self.n_meas = rospy.get_param('~n_measurements')                                   # number of samples before estimation
        self.speed = rospy.get_param('~speed')                                             # waypoint following speed [m/s]
        self.travel_rpm = rospy.get_param('~travel_rpm')                                   # waypoint target rotation speed
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance')                   # waypoint tolerance [m]
        self.range = rospy.get_param('~range')                                             # estimation circle radius [m]
        self.kernel_params = rospy.get_param('~kernel_params')                             # kernel parameters obtained through training
        self.kernel = rospy.get_param('~kernel')                                           # name of the kernel to use: RQ or MAT
        self.std = rospy.get_param('~std')                                                 # measurement noise
        self.alpha_seek = rospy.get_param('~alpha_seek')                                   # Controller param to seek front
        self.alpha_follow = rospy.get_param('~alpha_follow')                               # Controller param to follow front
        
        # Vehicle Controller
        self.ctl_rate = rospy.Rate(50)

        # Init variables
        self.measurement = None
        self.position_measurement = None
        self.position = None
        self.waypoint_reached = True
        self.gps_lat_offset = None
        self.gps_lon_offset = None

        # Call subscribers, publishers, services
        self.set_subscribers_publishers()
        self.set_services()

    ###############################################
    #           Callbacks Region                  #
    ###############################################
    def position__cb(self, fb):  
        """        
        Latlon topic subscriber callback:
        Update virtual position of the robot using dead reckoning
        """
        if self.gps_lat_offset is None or self.gps_lon_offset is None:
            return

        if fb.latitude > 1e-6 and fb.longitude > 1e-6:
            if self.position is None:
                self.position = np.array([[fb.longitude - self.gps_lon_offset, fb.latitude - self.gps_lat_offset]])
            else:
                self.position = np.append(self.position, np.array([[fb.longitude - self.gps_lon_offset, fb.latitude - self.gps_lat_offset]]), axis=0)
        else:
            rospy.logwarn("#PROBLEM# Received Zero GPS coordinates in Tracker!")


    def measurement__cb(self, fb): 
        """
        Callback when a sensor reading is received

        The sensor reading should be appended to the list of sensor readings, along with the associated
        lat lon position where the reading was taken.
        """

        # TODO : Make the measurement a service so that controller can set measurement period

        # read values (the sensor is responsible for providing the Geo stamp i.e. lat lon co-ordinates)
        position_measurement = np.array([[fb.lon, fb.lat]])
        sample = fb.sample

        # Save the measurement if not Nan
        if np.isnan(sample):
            print("Warning: NaN value measured.")
            if self.measurement is not None:
                self.measurement = np.append(self.measurement,self.measurement[-1])  # Avoid plots problems
                self.position_measurement = np.append(self.position_measurement,position_measurement, axis=0)
        else:
            if self.measurement is not None:
                self.measurement = np.append(self.measurement,sample)
                self.position_measurement = np.append(self.position_measurement,position_measurement, axis=0)
            else:
                self.measurement = np.array([sample])
                self.position_measurement = position_measurement
        pass

    def gps_offset__cb(self, msg):
        """
        Call for GPS offset, given by substance sampler
        """
        self.gps_lat_offset = msg.position.latitude
        self.gps_lon_offset = msg.position.longitude

    ###############################################
    #           End Callbacks Region              #
    ###############################################

    def set_subscribers_publishers(self):
        """
        Helper function to create all publishers and subscribers.
        """
        # Subscribers
        rospy.Subscriber('~measurement', ChlorophyllSample, self.measurement__cb)
        rospy.Subscriber('~gps', NavSatFix, self.position__cb)
        rospy.Subscriber('~gps_offset', GeoPointStamped, self.gps_offset__cb, queue_size=2)

        # Publishers
        self.enable_waypoint_pub = rospy.Publisher("~enable_live_waypoint", Bool, queue_size=1)
        self.waypoint_pub = rospy.Publisher("~live_waypoint", GotoWaypoint, queue_size=5)
        self.vp_pub = rospy.Publisher("~virtual_position", GeoPointStamped, queue_size=1)
        self.gradient_pub = rospy.Publisher("~gradient", AlgaeFrontGradient, queue_size=1)

    def set_services(self):
        """
        Helper function to create all services.
        """

        service = '/sam/dr/lat_lon_to_utm'
        try:
            rospy.wait_for_service(service, timeout=1)
        except:
            rospy.logwarn(str(service)+" service not found!")

        self.latlontoutm_service = rospy.ServiceProxy(service,LatLonToUTM)

    ############################################################################################################
    def run(self):

        ############ INIT functions
        # Dynamics
        dynamics = controller.Dynamics(self.alpha_seek, self.alpha_follow, self.delta_ref, self.wp_distance)
        # Gaussian Process
        est = gpr.GPEstimator(self.kernel, self.std, self.range, self.kernel_params)

        ############ Tunable parameters
        meas_filter_len = 3 
        alpha = 0.97
        weights_meas = [0.2, 0.3, 0.5]
        init_flag = True
        init_gradient = np.array([[1, 0]])

        ############ INIT vectors 
        gradient = np.empty((0,2))
        self.filtered_measurements = np.empty((0, 2))
        self.filtered_gradient = np.empty((0, 2))
        self.next_waypoint = np.array([[0, 0]])

        ############ Get the first measurement
        while self.measurement is None or self.position is None or self.gps_lat_offset is None or self.gps_lon_offset is None:
            rospy.logwarn("Waiting for valida data: ")
            rospy.logwarn("Measurement: {} - Position: {} - GPS Offset: {},{}".format(self.measurement, self.position, self.gps_lon_offset, self.gps_lat_offset))
            rospy.sleep(1)


        ########### MISSION CYCLE
        while not rospy.is_shutdown():
            
            ##### Init state - From beginning until at front
            if (len(self.filtered_measurements) < self.n_meas or self.measurement[-1] < 0.95*self.delta_ref) and init_flag is True:
                gradient = np.append(gradient, init_gradient[[0], :2] / np.linalg.norm(init_gradient[0, :2]), axis=0)
                self.filtered_gradient = np.append(self.filtered_gradient, gradient[[-1],:], axis=0)
                self.filtered_measurements = np.append(self.filtered_measurements,self.measurement[-1])

            ##### Main state - From reaching the front till the end 
            else:
                if init_flag is True:
                    print("Following the front...")
                    init_flag = False

                self.filtered_measurements = np.append(self.filtered_measurements, np.average(self.measurement[- meas_filter_len:], weights=weights_meas))

                # Estimate and filter gradient
                gradient_calculation = np.array(est.est_grad(self.position_measurement[-self.n_meas:, :].reshape((self.n_meas, 2)), self.filtered_measurements[-self.n_meas:].reshape((-1, 1)))).squeeze().reshape(-1, 2)
                gradient = np.append(gradient, gradient_calculation / np.linalg.norm(gradient_calculation), axis=0)
                self.filtered_gradient = np.append(self.filtered_gradient, self.filtered_gradient[[-1], :]*alpha + gradient[[-1], :]*(1-alpha), axis=0)

            print("measurement was", self.filtered_measurements[-1], "and gradient was", self.filtered_gradient[-1])
            
            ##### Always - Calculate next waypoint
            control = dynamics(self.filtered_measurements[-1], self.filtered_gradient[-1,:], include_time=False)
            self.next_waypoint = controller.next_position(self.position[-1, :],control)
            
            # Convert back waypoints to simulator coordinates
            self.next_waypoint[0, 0] = self.next_waypoint[0, 0] + self.gps_lon_offset
            self.next_waypoint[0, 1] = self.next_waypoint[0, 1] + self.gps_lat_offset
            self.waypoint_reached = False
            publish_waypoint(self.latlontoutm_service,self.next_waypoint,self.waypoint_pub,self.enable_waypoint_pub,self.travel_rpm,self.speed,self.waypoint_tolerance)

            if self.next_waypoint[0, 1] > 61.64:
                break

            self.ctl_rate.sleep()

    def close_node(self,signum, frame):
        """
        Stop following behaviour, save data output and close the node
        """
        rospy.logwarn("Closing node")
        rospy.logwarn("Attempting to end waypoint following")

        try :
            enable_waypoint_following = Bool()
            enable_waypoint_following.data = False
            self.enable_waypoint_pub.publish(enable_waypoint_following)
            rospy.logwarn("Waypoint following successfully disabled")
        except Exception as e:
            rospy.logwarn("Failed to disabled Waypoint following")

        out_path = rospy.get_param('~output_data_path')
        try :
            # Write data to file
            save_raw_mission_data(out_path=out_path, traj=self.position,measurements=self.filtered_measurements,grads=self.filtered_gradient,delta_ref=self.delta_ref)
            rospy.logwarn("Data saved!")

        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Failed to save data")

        exit(1)


if __name__ == '__main__':
    rospy.init_node('Front_Tracking')

    tracker = FrontTracking()

    signal.signal(signal.SIGINT, tracker.close_node)

    tracker.run()
    rospy.signal_shutdown()
