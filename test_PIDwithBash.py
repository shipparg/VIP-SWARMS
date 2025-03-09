import airsimneurips
import numpy as np
import math
import time
from scipy.interpolate import CubicSpline
import argparse

# Connect to AirSim
client = airsimneurips.MultirotorClient()

# Constant for number of waypoints
NUM_WAYPOINTS = 10
DATA = {}
VELOCITY = 5

def parse_args():
    parser = argparse.ArgumentParser(description='PID Control Parameters')
    parser.add_argument('kp', type=float, help='Proportional gain')
    parser.add_argument('ki', type=float, help='Integral gain')
    parser.add_argument('kd', type=float, help='Derivative gain')
    return parser.parse_args()

def gatePos():
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
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    client.simStartRace()
    client.takeoffAsync(vehicle_name='drone_1').join()
    # start_position = airsimneurips.Vector3r(0.0, 0.0, 0.8)
    # start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    # new_pose = airsimneurips.Pose(start_position, start_rotation)
    # client.simSetVehiclePose(new_pose, ignore_collison=True)

def generate_spline(posDict):
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


def checkSphere(position: airsimneurips.Vector3r, radius=2.5):
    currentPos = client.getMultirotorState(vehicle_name='drone_1').kinematics_estimated.position

    # we find the distance between the gate and the current pos
    dx = currentPos.x_val - position.x_val
    dy = currentPos.y_val - position.y_val
    dz = currentPos.z_val - position.z_val
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if distance > radius:
        return False
    else:
        return True


def controlTerms(target, current, integral, previous_error, kp, ki, kd, dt):
    # Calculate error
    error = target - current
    # now find all i,d,p terms
    P = kp * error
    integral += error * dt
    I = ki * integral
    derivative = (error - previous_error) / dt
    D = kd * derivative

    output = P + I + D
    return output, integral, error


def follow_spline(path_points, kpX, kiX, kdX, kpY, kiY, kdY, kpZ, kiZ, kdZ):
    dt = 0.1  # 100ms loop time

    # PID vars for each axis
    integral_x = integral_y = integral_z = 0
    prev_error_x = prev_error_y = prev_error_z = 0

    # list of errors for the x-axis:
    errorX = []  # errors for a particular VELOCITY

    for waypoint in path_points:
        target = airsimneurips.Vector3r(waypoint[0], waypoint[1], waypoint[2])
        print("moving to: ", target)

        while not checkSphere(target):
            current = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position

            # find terms for each axis
            x_term, integral_x, previous_error_x = controlTerms(target.x_val, current.x_val, integral_x, prev_error_x,
                                                                kpX, kiX, kdX, dt)
            y_term, integral_y, previous_error_y = controlTerms(target.y_val, current.y_val, integral_y, prev_error_y,
                                                                kpY, kiY, kdY, dt)
            z_term, integral_z, previous_error_z = controlTerms(target.z_val, current.z_val, integral_z, prev_error_z,
                                                                kpZ, kiZ, kdZ, dt)

            errorX.append(x_term)

            client.moveToPositionAsync(current.x_val + x_term, current.y_val + y_term, current.z_val + z_term,
                                       VELOCITY, vehicle_name='drone_1')
            time.sleep(dt)

    DATA[VELOCITY] = errorX
    print(errorX)


def main():
    # enable()
    args = parse_args()
    gate_positions = gatePos()

    # spline stuff
    spline_path = generate_spline(gate_positions)

    for gate_number, waypoints in spline_path.items():
        follow_spline(waypoints, args.kp, args.ki, args.kd, args.kp, args.ki, args.kd, args.kp, args.ki, args.kd)

    print("completed!")
    client.simResetRace()
    # client.reset()
main()
