# test implementation of simple gate to gate movement
import airsimneurips

client = airsimneurips.MultirotorClient()
client.confirmConnection()
print('connection confirmed')
# client.simLoadLevel('Soccer_Field_Easy')

all_objects = client.simListSceneObjects(name_regex='.*')
gates = [obj for obj in all_objects if 'Gate' in obj]
pos_dict = {}
for gate_name in gates:
    gate_position = client.simGetObjectPose(gate_name).position
    pos_dict.update({gate_name: gate_position})
# print(pos_dict)

client.enableApiControl(vehicle_name="drone_1")  # Enable API control for the drone
client.arm(vehicle_name="drone_1")  # Arm the drone so it can take off
client.simStartRace()
client.takeoffAsync(vehicle_name='drone_1').join()

linVel = []
linAcc = []
for names in pos_dict.keys():
    gate_pos = pos_dict[names]
    client.moveToPositionAsync(gate_pos.x_val, gate_pos.y_val, gate_pos.z_val, 5, vehicle_name='drone_1').join()
    print('Finished gate ', names)
    kinematics = client.getMultirotorState(vehicle_name='drone_1')
    linVel_mag = kinematics.kinematics_estimated.linear_velocity.x_val ** 2 + kinematics.kinematics_estimated.linear_velocity.y_val ** 2 + kinematics.kinematics_estimated.linear_velocity.z_val ** 2
    linVel.append(linVel_mag)
    linAcc_mag = kinematics.kinematics_estimated.linear_acceleration.x_val ** 2 + kinematics.kinematics_estimated.linear_acceleration.y_val ** 2 + kinematics.kinematics_estimated.linear_acceleration.z_val ** 2
    linAcc.append(linAcc_mag)

# client.landAsync(vehicle_name='drone_1').join()
print(linVel)
print(linAcc)





