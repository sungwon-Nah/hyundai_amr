import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Patch
from geopy.distance import geodesic
from collections import deque
import pymap3d as pm

class TireForceVisualizer(Node):

    def __init__(self):
        super().__init__('tire_force_visualizer')
        self.f_z_est_sub = self.create_subscription(
            Float32MultiArray,
            '/F_z_est',
            self.f_z_est_callback,
            10
        )
        self.f_z_req_sub = self.create_subscription(
            Float32MultiArray,
            '/F_z_req',
            self.f_z_req_callback,
            10
        )
        self.navsat_sub = self.create_subscription(
            NavSatFix,
            '/ekf/llh_position',
            self.navsat_callback,
            10
        )
        self.fl_activate_sub = self.create_subscription(
            Bool,
            '/FL_activate',
            self.fl_activate_callback,
            10
        )
        self.fr_activate_sub = self.create_subscription(
            Bool,
            '/FR_activate',
            self.fr_activate_callback,
            10
        )
        self.rl_activate_sub = self.create_subscription(
            Bool,
            '/RL_activate',
            self.rl_activate_callback,
            10
        )
        self.rr_activate_sub = self.create_subscription(
            Bool,
            '/RR_activate',
            self.rr_activate_callback,
            10
        )

        self.f_z_est = [0.0, 0.0, 0.0, 0.0]
        self.f_z_req = [0.0, 0.0, 0.0, 0.0]
        self.activate = [False, False, False, False]
        self.position_history = deque(maxlen=200)  # Store last 200 positions
        self.vehicle_position = None

        # Set up the plot
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 10))
        self.axs = self.axs.flatten()
        self.axs_titles = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']
        for ax, title in zip(self.axs, self.axs_titles):
            ax.set_aspect('equal')
            ax.set_title(title)
            ax.set_xlim(-20000, 20000)
            ax.set_ylim(-20000, 20000)

        # Set up vehicle footprint plot
        self.footprint_fig, self.footprint_ax = plt.subplots()
        self.footprint_ax.set_title('Vehicle Footprint')
        self.footprint_ax.set_xlim(-5000, 5000)
        self.footprint_ax.set_ylim(-5000, 5000)
        self.vehicle_patch = Patch(color='green', alpha=0.5, label='Vehicle Position')

        # Create proxy artists for the legend
        self.est_patch = Patch(color='blue', alpha=1.0, label='F_z_est')
        self.req_patch = Patch(color='red', alpha=1.0, label='F_z_req')
        self.past_position_patch = Patch(color='blue', alpha=0.5, label='Past Position')
        self.current_position_patch = Patch(color='red', alpha=1.0, label='Current Position')

        plt.ion()  # Turn on interactive mode
        plt.show()

    def f_z_est_callback(self, msg):
        self.f_z_est = msg.data
        self.update_plot()

    def f_z_req_callback(self, msg):
        self.f_z_req = msg.data
        self.update_plot()

    def navsat_callback(self, msg):
        # self.vehicle_position = (msg.latitude, msg.longitude)
        n, e, d = pm.geodetic2ned(msg.latitude, msg.longitude, 0, 36.3660857, 127.3636824, 0) #kaist = 36.3660857, 127.3636824)
        self.vehicle_position = (n, e)
        # print(self.vehicle_position)
        self.position_history.append(self.vehicle_position)
        self.update_footprint_plot()

    def fl_activate_callback(self, msg):
        self.activate[0] = msg.data
        self.update_plot()

    def fr_activate_callback(self, msg):
        self.activate[1] = msg.data
        self.update_plot()

    def rl_activate_callback(self, msg):
        self.activate[2] = msg.data
        self.update_plot()

    def rr_activate_callback(self, msg):
        self.activate[3] = msg.data
        self.update_plot()

    def update_plot(self):
        for ax in self.axs:
            ax.clear()

        for i, (ax, est, req, activate) in enumerate(zip(self.axs, self.f_z_est, self.f_z_req, self.activate)):
            est_circle = Circle((0, 0), est, color='blue', alpha=1.0)
            req_circle = Circle((0, 0), req, color='red', alpha=1.0)
            ax.add_patch(est_circle)
            ax.add_patch(req_circle)
            ax.set_title(self.axs_titles[i])
            ax.set_xlim(-20000, 20000)
            ax.set_ylim(-20000, 20000)

            # Add legend to each subplot
            ax.legend(handles=[self.est_patch, self.req_patch])

            # Check if the wheel is activated and change the background color
            if activate:
                ax.set_facecolor((1, 1, 0, 0.3))
            else:
                ax.set_facecolor('white')

        plt.draw()
        plt.pause(0.01)

    def update_footprint_plot(self):
        self.footprint_ax.clear()
        if self.position_history:
            lats, lons = zip(*self.position_history)
            self.footprint_ax.plot(lons[:-1], lats[:-1], 'bo-', alpha=0.5, label='Past Position')  # Past positions in blue with transparency
            self.footprint_ax.plot(lons[-1], lats[-1], 'ro', alpha=1.0, label='Current Position')  # Current position in red without transparency

            # Add legend
            self.footprint_ax.legend(handles=[self.past_position_patch, self.current_position_patch])

        self.footprint_ax.set_title('Vehicle Position')
        if self.position_history:
            min_lon = min(lons) - 100
            max_lon = max(lons) + 100
            min_lat = min(lats) - 100
            max_lat = max(lats) + 100
            self.footprint_ax.set_xlim(min_lon, max_lon)
            self.footprint_ax.set_ylim(min_lat, max_lat)
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    visualizer = TireForceVisualizer()
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
