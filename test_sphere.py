# test implementation of simple gate to gate movement
import airsimneurips

client = airsimneurips.MultirotorClient()
client.confirmConnection()
print('connection confirmed')

# client.simLoadLevel('Soccer_Field_Easy')

def gatePos():
    all_objects = client.simListSceneObjects(name_regex='.*')
    gates = [obj for obj in all_objects if 'Gate' in obj]
    pos_dict = {}
    for gate_name in gates:
        gate_position = client.simGetObjectPose(gate_name).position
        pos_dict.update({gate_name: gate_position})
    return pos_dict


def enable():
    client.enableApiControl(vehicle_name="drone_1")  # Enable API control for the drone
    client.arm(vehicle_name="drone_1")  # Arm the drone so it can take off
    client.simStartRace()
    client.takeoffAsync(vehicle_name='drone_1').join()


# here we try to set up the co-ordinates for the sphere and the corresponding co-ordinates
def sphere(pos_dict):
    sphereCods = {}
    radius = 2  # radius of the sphere (how do we pick the radius?)
    for gate_name in pos_dict.keys():
        new_pos = client.simGetObjectPose(gate_name).position
        x_min = new_pos.x_val - radius
        x_max = new_pos.x_val + radius
        y_min = new_pos.y_val - radius
        y_max = new_pos.y_val + radius
        z_min = new_pos.z_val - radius
        z_max = new_pos.z_val + radius
        sphereCods[gate_name] = [x_min, x_max, y_min, y_max, z_min, z_max]
    return sphereCods


def simplemove(gateInfo):  # gateInfo is just posDict[key]
    client.moveToPositionAsync(gateInfo.x_val, gateInfo.y_val, gateInfo.z_val,
                               5)  # we need to dynamically set this vel value at a later date


def sphere_condition(sphere_info, gate_name):
    # check whether the drone is within the sphere or not
    # if so, we need to immediately change focus to the next target by updating movePosAsync
    current_pos = client.getMultirotorState(vehicle_name='drone_1').kinematics_estimated.position
    if sphere_info[gate_name][0] < current_pos.x_val < sphere_info[gate_name][1]:
        if sphere_info[gate_name][2] < current_pos.y_val < sphere_info[gate_name][3]:
            if sphere_info[gate_name][4] < current_pos.z_val < sphere_info[gate_name][5]:
                return True
    else:
        return False


def sphere_movement(posDict, s):
    # first move the drone to the first gate
    # then check for the sphere condition
    # if true, set movement goal to the next gate
    # if false, continue
    for gate_name in posDict.keys():
        simplemove(posDict[gate_name])  # first we make it move to the gate_name specified
        # then we check if it is within the minmax co-ords of the gate
        # we need to keep checking for sphere_condition until it passes.
        # when it passes, we immediately go back to the for loop
        while not sphere_condition(s, gate_name):
            pass


if __name__ == '__main__':
    enable()
    gateInfo = gatePos()  # returns the gate positions
    # print(gateInfo.keys())
    sphereData = sphere(gateInfo)
    # print(sphereData.keys())
    sphere_movement(gateInfo, sphereData)
    print("move commands complete")
