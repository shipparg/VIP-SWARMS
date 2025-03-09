import airsimneurips
import numpy as np
import math
import time
from scipy.interpolate import CubicSpline

# Connect to AirSim
client = airsimneurips.MultirotorClient()

# Constant for number of waypoints
NUM_WAYPOINTS = 10  # You can adjust this based on the requirement

def gatePos():
    """ Retrieve gate positions and sort them in numerical order. """
    all_objects = client.simListSceneObjects(name_regex='.*')
    gates = [obj for obj in all_objects if 'Gate' in obj]
    pos_dict = {}
    for gate_name in gates:
        gate_position = client.simGetObjectPose(gate_name).position
        gate_number = int(gate_name[4:6])  # Extracting gate number safely
        pos_dict[gate_number] = gate_position
    sorted_dict = {k: pos_dict[k] for k in sorted(pos_dict)}
    return sorted_dict

def enable():
    """ Enable API control and arm the drone. """
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    client.simStartRace()
    client.takeoffAsync(vehicle_name='drone_1').join()

# PID Controller for Position Control
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

def generate_spline(posDict):
    """ Generate spline trajectories between gates. """
    time = [0, 1]  # Stand-in for x
    testwp = {}

    for gate_number in range(len(posDict) - 1):
        gate1_pos = posDict[gate_number]
        gate2_pos = posDict[gate_number + 1]

        # Generate cubic splines for each axis (x, y, z)
        trajX = CubicSpline(time, [gate1_pos.x_val, gate2_pos.x_val])
        trajY = CubicSpline(time, [gate1_pos.y_val, gate2_pos.y_val])
        trajZ = CubicSpline(time, [gate1_pos.z_val, gate2_pos.z_val])

        # Generate waypoints along the trajectory
        timeWaypoints = np.linspace(0, 1, NUM_WAYPOINTS)
        waypoints = []

        for t in timeWaypoints:
            x_val = trajX(t)
            y_val = trajY(t)
            z_val = trajZ(t)
            waypoints.append([x_val, y_val, z_val])

        testwp[gate_number] = waypoints
    return testwp

def inGateSphere(position: airsimneurips.Vector3r, radius=2.5):
    """ Check if the drone is inside the target gate's sphere """
    dronePose = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position

    dx = dronePose.x_val - position.x_val
    dy = dronePose.y_val - position.y_val
    dz = dronePose.z_val - position.z_val
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    return distance <= radius

def follow_spline(path_points):
    """ Move drone along the spline using PID control to refine position. """
    dt = 0.1  # 100ms loop time
    pid_x = PIDController(kp=1.5, ki=0.05, kd=0.2, dt=dt)
    pid_y = PIDController(kp=1.5, ki=0.05, kd=0.2, dt=dt)
    pid_z = PIDController(kp=1.2, ki=0.05, kd=0.1, dt=dt)

    for waypoint in path_points:
        target = airsimneurips.Vector3r(waypoint[0], waypoint[1], waypoint[2])
        print(f"Navigating to {target}")

        pid_x.integral = pid_y.integral = pid_z.integral = 0
        pid_x.previous_error = pid_y.previous_error = pid_z.previous_error = 0

        while True:
            current = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
            error_x = target.x_val - current.x_val
            error_y = target.y_val - current.y_val
            error_z = target.z_val - current.z_val

            # Compute corrections using PID controllers
            vx = pid_x.update(error_x)
            vy = pid_y.update(error_y)
            vz = pid_z.update(error_z)

            # Move the drone toward the target
            client.moveByVelocityAsync(vx, vy, vz, dt, vehicle_name="drone_1")
            time.sleep(dt)

            if inGateSphere(target):
                break

    print("Spline path completed!")

def main():
    enable()
    gate_positions = gatePos()

    if len(gate_positions) < 2:
        print("Not enough gates detected for a spline path.")
        return

    # Convert dictionary values to a list of positions
    gate_positions_list = list(gate_positions.values())

    # Generate spline trajectory
    spline_path = generate_spline(gate_positions)

    # Move along the spline with PID control
    print("Following the spline path with PID control...")
    for gate_number, waypoints in spline_path.items():
        follow_spline(waypoints)

    print("Race completed!")

main()
