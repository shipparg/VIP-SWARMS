from __future__ import print_function

from .utils import *
from .types import *
from .baseline import *
import msgpackrpc
import numpy as np
import msgpack
import time
import math
import logging
from copy import deepcopy 

class MultirotorClient(object):
    """
    
    """
    
    def __init__(self, ip = "127.0.0.1", port = 41451, timeout_value = 3600):
        """
        Args:
            ip (str, optional): Description
            port (int, optional): Description
            timeout_value (int, optional): Description
        """
        self.client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout = timeout_value, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')
        self.race_tier = None
        self.level_name = None
        self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS = 10
        
    # -----------------------------------  Common vehicle APIs ---------------------------------------------
    def reset(self):
        """ This resets the vehicle to its original starting state. Note that you must call enableApiControl() and arm() again after calling reset.
        """
        self.client.call('reset')

    def ping(self):
        """ If connection is established then this call will return true otherwise it will be blocked until timeout.
        Returns:
            bool: True if ping was succesfull
        """
        return self.client.call('ping')

    def enableApiControl(self, vehicle_name = ''):
        """Enables API control for vehicle with name vehicle_name
        
        Args:
            vehicle_name (str, optional): Name of vehicle to enable API control for
        
        Returns:
            bool: Success
        """
        return self.client.call('enableApiControl', True, vehicle_name)

    def disableApiControl(self, vehicle_name = ''):
        """Disables API control for vehicle with name vehicle_name
        
        Args:
            vehicle_name (str, optional): Name of vehicle to disable API control for
        
        Returns:
            bool: Success
        """
        return self.client.call('enableApiControl', False, vehicle_name)

    def isApiControlEnabled(self, vehicle_name = ''):
        """Returns true if API control is established. If false (which is default) then API calls would be ignored. After a successful call to enableApiControl, the isApiControlEnabled should return true.
        
        Args:
            vehicle_name (str, optional): Name of vehicle
        
        Returns:
            bool: If API is enabled 
        """
        return self.client.call('isApiControlEnabled', vehicle_name)

    def arm(self, vehicle_name = ''):
        """Arms vehicle corresponding to vehicle_name
        
        Args:
            vehicle_name (str): Name of vehicle
        
        Returns:
            bool: Success
        
        """
        return self.client.call('armDisarm', True, vehicle_name)

    def disarm(self, vehicle_name = ''):
        """Disarms vehicle corresponding to vehicle_name.
        
        Args:
            vehicle_name (str): Name of vehicle
        
        Returns:
            bool: Success
        
        """
        return self.client.call('armDisarm', False, vehicle_name)

    def simPause(self):
        """
        Pauses sim
        """
        self.client.call('simPause', True)

    def simUnPause(self):
        """
        Resumes / unpauses sim
        """
        self.client.call('simPause', False)

    def simIsPaused(self):
        """
        Returns:
            bool: Returns True if simulator is paused.
        """
        return self.client.call("simIsPaused")

    def simContinueForTime(self, duration):
        """
        Args:
            duration (float): Unpauses simulator if paused, runs it for desired duration (seconds), then pauses it. 
        """
        self.client.call('simContinueForTime', duration)

    def confirmConnection(self):
        """
        Checks state of connection every 1 sec and reports it in Console so user can see the progress for connection.
        """
        if self.ping():
            print("Connected!")
        else:
             print("Ping returned false!")

     # Race APIs
    def simStartRace(self, tier=1):
        """ Starts an instance of a race in your given level, if valid."""
        self.race_tier = tier
        if (self.level_name == 'Final_Tier_1'):
            self.race_tier = 1
        if (self.level_name == 'Final_Tier_2'):
            self.race_tier = 2
        if (self.level_name == 'Final_Tier_3'):
            self.race_tier = 3

        if self.race_tier == 2:
            self.client.call('simStartRace', self.race_tier)
            return
        else:
            client_instance = self.__class__()
            competitor = BaselineRacer(client_instance, viz_traj=False, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0])
            competitor.level_name = self.level_name
            competitor.initialize_drone()
            competitor.gate_poses_ground_truth = [self.__internalRandomGoalZone(gate) for gate in sorted(self.simListSceneObjects(".*[Gg]ate.*"))]
            if self.level_name == 'Soccer_Field_Medium':
                curr_pose = deepcopy(competitor.gate_poses_ground_truth[19])
                curr_pose.position.x_val = (competitor.gate_poses_ground_truth[19].position.x_val + competitor.gate_poses_ground_truth[20].position.x_val) / 2
                competitor.gate_poses_ground_truth.insert(20, curr_pose)

            if self.level_name == 'Building99_Hard':
                competitor.gate_poses_ground_truth.insert(3, Pose(Vector3r((-21.49-41.89)/2.0, (-5.44-1.84)/2, (1.51+1.82)/2)))

            if self.level_name == 'ZhangJiaJie_Medium':
                num_gates = len(competitor.gate_poses_ground_truth)
                last_gate_position = deepcopy(competitor.gate_poses_ground_truth[-1].position)
                second_last_gate_position = deepcopy(competitor.gate_poses_ground_truth[-2].position)
                competitor.gate_poses_ground_truth.insert(num_gates-1, Pose(Vector3r((last_gate_position.x_val+second_last_gate_position.x_val)/2.0, 
                                                                                (last_gate_position.y_val+second_last_gate_position.y_val)/2.0,
                                                                                (last_gate_position.z_val+second_last_gate_position.z_val)/2.0 )))


            self.client.call('simStartRace', self.race_tier)
            competitor.run_in_thread()

    def __internalRandomGoalZone(self, gate_name):
        gate_pose = self.__internalGetObjectPose(gate_name)
        x_rand = (2*np.random.random() - 1) * 0.50 
        z_rand = (2*np.random.random() - 1) * 0.50
        nominal_dims = self.simGetNominalGateInnerDimensions()
        gate_scale = self.simGetObjectScaleInternal(gate_name)
        inner_dims = Vector3r(gate_scale.x_val * nominal_dims.x_val, 0, gate_scale.z_val * nominal_dims.z_val)
        useful_zone = [inner_dims.x_val/2, inner_dims.z_val/2]
        offset = Quaternionr(x_rand*useful_zone[0], 0, z_rand*useful_zone[1], 0)
        rotated_position = offset.rotate(gate_pose.orientation)
        return Pose(gate_pose.position + Vector3r(rotated_position.x_val, rotated_position.y_val,rotated_position.z_val))

    def simResetRace(self):
        """
        Resets a current race: moves players to start positions, timer and penalties reset.
        """
        self.client.call('simResetRace')

    def simDisableRaceLog(self):
        """
        Disables race log
        """
        self.client.call('simDisableRaceLog')

    def simIsRacerDisqualified(self, vehicle_name):
        """
        Args:
            vehicle_name (str): Name of the multirotor to send this command to 
        
        Returns:
            bool: True if vehicle_name is disqualified. False if not
        """
        return self.client.call('simGetDisqualified', vehicle_name)

    def simGetLastGatePassed(self, vehicle_name):
        """
        
        Args:
            vehicle_name (str): Name of the multirotor to send this command to 
        
        Returns:
            int: index of last gate passed by vehicle_name
        """
        return self.client.call('simGetLastGatePassed', vehicle_name)

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImage(self, camera_name, image_type, vehicle_name = ''):
        """
        Args:
            camera_name (string): name of camera to get image for
            image_type (ImageType): Description
            vehicle_name (str): Name of the multirotor to send this command to. 
                Note that the vehicle_name must have camera_name in its section in the settings.json file
        
        Returns:
            TYPE: Description
        """
        # todo: in future remove below, it's only for compatibility to pre v1.2
        camera_name = str(camera_name)

        # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
        result = self.client.call('simGetImage', camera_name, image_type, vehicle_name)
        if (result == "" or result == "\0"):
            return None
        return result

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImages(self, requests, vehicle_name = ''):
        """
        Args:
            requests (ImageRequest): Description
            vehicle_name (str): Name of the multirotor to send this command to 
        
        Returns:
            TYPE: Description
        """
        responses_raw = self.client.call('simGetImages', requests, vehicle_name)
        return [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]

    def simGetCollisionInfo(self, vehicle_name = ''):
        """
        Args:
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            CollisionInfo: Description
        """
        return CollisionInfo.from_msgpack(self.client.call('simGetCollisionInfo', vehicle_name))

    def simSetVehiclePose(self, pose, ignore_collison, vehicle_name = ''):
        """
        
        Args:
            pose (TYPE): Description
            ignore_collison (TYPE): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call('simSetVehiclePose', pose, ignore_collison, vehicle_name)

    def simGetVehiclePose(self, vehicle_name = ''):
        """
        
        Args:
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            Pose: pose of desired vehicle
        """
        pose = self.client.call('simGetVehiclePose', vehicle_name)
        return Pose.from_msgpack(pose)

    def simGetObjectPose(self, object_name):
        """
        Returns true pose for tier 1 races
        Returns noisy pose for tier 2 and 3 races

        Args:
            object_name (str): Name of the object to get pose for
        
        Returns:
            Pose: pose of desired object, NanPose if object not found
        """
        if self.race_tier is None:
            pose = self.client.call('simGetObjectPose', object_name, True)
        elif self.race_tier == 1:
            pose = self.client.call('simGetObjectPose', object_name, False)
        else:
            pose = self.client.call('simGetObjectPose', object_name, True)
        return Pose.from_msgpack(pose)

    def __internalGetObjectPose(self, object_name):
        pose_msgpack = self.client.call('simGetObjectPose', object_name, False)
        pose = Pose.from_msgpack(pose_msgpack)
        counter = 0
        while (math.isnan(pose.position.x_val) or math.isnan(pose.position.y_val) or math.isnan(pose.position.z_val)) and (counter < self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS):
            print(f"DEBUG:  position is nan, retrying...")
            counter += 1
            pose_msgpack = self.client.call('simGetObjectPose', object_name, False)
            pose = Pose.from_msgpack(pose_msgpack)
        assert not math.isnan(pose.position.x_val), f"ERROR: pose.position.x_val is still {pose.position.x_val} after {counter} trials"
        assert not math.isnan(pose.position.y_val), f"ERROR: pose.position.y_val is still {pose.position.y_val} after {counter} trials"
        assert not math.isnan(pose.position.z_val), f"ERROR: pose.position.z_val is still {pose.position.z_val} after {counter} trials"

        return pose

    def simSetObjectPose(self, object_name, pose, teleport = True):
        """
        Args:
            object_name (string): Name of object to move. 
                Note that you can get all objects with simListSceneObjects. 
            pose (Pose): desired Pose in world NED frame
            teleport (bool, optional): Description
        
        Returns:
            TYPE: Description
        """
        return self.client.call('simSetObjectPose', object_name, pose, teleport)

    def simGetNominalGateInnerDimensions(self):
        """
        - Return the dimensions of the drone racing gate cavity, with scale (width=1.0, thickness=1.0, height=1.0)
        - Use this API in conjunction with simGetObjectScale(), simSetObjectScale(), simSetObjectPose() to generate arbitrary sized checkered gates with arbitrary poses.  

        Returns:
            Vector3r: width, thickness, height of the gate inner cavity in meters. 
            See https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/28 for gate coordinate system
        """
        return Vector3r(1.6, 0.2, 1.6) 

    def simGetNominalGateOuterDimensions(self):
        """
        - Return the outer dimensions of the drone racing gate, with scale (width=1.0, thickness=1.0, height=1.0)
        - Use this API in conjunction with simGetObjectScale(), simSetObjectScale(), simSetObjectPose() to generate arbitrary sized checkered gates with arbitrary poses.  

        Returns:
            Vector3r: width, thickness, height of the gate inner cavity in meters. 
            See https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/28 for gate coordinate system
        """
        return Vector3r(3.2 / 1.5, 0.2, 3.2 / 1.5) 

    def simGetObjectScale(self, object_name):
        """

        Args:
            object_name (str): Name of the object to poll

        Returns:
            Vector3r: scale vector of desired object
        """
        scale = self.client.call('simGetObjectScale', object_name)
        return Vector3r.from_msgpack(scale)

    def simGetObjectScaleInternal(self, object_name):
        scale = self.client.call('simGetObjectScaleInternal', object_name)
        return Vector3r.from_msgpack(scale)

    def simSetObjectScale(self, object_name, scale_vector):
        """

        Args:
            object_name (str): Name of the object to poll
            scale (Vector3r): desired scale of the object
        """
        return self.client.call('simSetObjectScale', object_name, scale_vector)

    def simListSceneObjects(self, name_regex = '.*'):
        """Returns list of all objects in the scene 
        
        Args:
            name_regex (str, optional): Description
        
        Returns:
            List[strings]: List of all objects in the scene 
        """
        return self.client.call('simListSceneObjects', name_regex)

    def simLoadLevel(self, level_name):
        """Loads desired level
        
        Args:
            level_name (str): Description
        
        Returns:
            TYPE: Description
        """
        self.level_name = level_name
        return self.client.call('simLoadLevel', level_name)

    def simSetSegmentationObjectID(self, mesh_name, object_id, is_name_regex = False):
        """
        
        Args:
            mesh_name (str): Description
            object_id (int): Description
            is_name_regex (bool, optional): Description
        
        Returns:
            bool: Success
        """
        return self.client.call('simSetSegmentationObjectID', mesh_name, object_id, is_name_regex)

    def simGetSegmentationObjectID(self, mesh_name):
        """
        
        Args:
            mesh_name (str): Description
        
        Returns:
            int: segmentation ID
        """
        return self.client.call('simGetSegmentationObjectID', mesh_name)

    def simPrintLogMessage(self, message, message_param = "", severity = 0):
        """Prints the specified message in the simulator's window. 
        | If message_param is also supplied then its printed next to the message and in that case if this API is called with same message value but different message_param again then previous line is overwritten with new line (instead of API creating new line on display). 
        | For example, simPrintLogMessage("Iteration: ", to_string(i)) keeps updating same line on display when API is called with different values of i. 
        | The valid values of severity parameter is 0 to 3 inclusive that corresponds to different colors.
        
        Args:
            message (TYPE): Description
            message_param (str, optional): Description
            severity (int, optional): Description
        
        Returns:
            TYPE: Description
        """
        return self.client.call('simPrintLogMessage', message, message_param, severity)

    def simGetCameraInfo(self, camera_name, vehicle_name = ''):
        """
        
        Args:
            camera_name (TYPE): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            CameraInfo: Description
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        return CameraInfo.from_msgpack(self.client.call('simGetCameraInfo', str(camera_name), vehicle_name))

    def simSetCameraOrientation(self, camera_name, orientation, vehicle_name = ''):
        """Sets camera's orientation to desired quaternion
        
        Args:
            camera_name (str): Description
            orientation (Quaternionr): Desired orientation quaternion
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        self.client.call('simSetCameraOrientation', str(camera_name), orientation, vehicle_name)

    def simGetGroundTruthKinematics(self, vehicle_name = ''):
        """
        
        Args:
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            TYPE: Description
        """
        kinematics_state = self.client.call('simGetGroundTruthKinematics', vehicle_name)
        return KinematicsState.from_msgpack(kinematics_state)
    simGetGroundTruthKinematics.__annotations__ = {'return': KinematicsState}

    def cancelLastTask(self, vehicle_name = ''):
        """
        """
        self.client.call('cancelLastTask', vehicle_name)

    def waitOnLastTask(self, timeout_sec = float('nan')):
        """
        
        Args:
            timeout_sec (TYPE, optional): Description
        
        Returns:
            TYPE: Description
        """
        return self.client.call('waitOnLastTask', timeout_sec)

# -----------------------------------  Multirotor APIs ---------------------------------------------

    def takeoffAsync(self, timeout_sec = 20, vehicle_name = ''):
        """
        
        Args:
            timeout_sec (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('takeoff', timeout_sec, vehicle_name)  

    def landAsync(self, timeout_sec = 60, vehicle_name = ''):
        """
        
        Args:
            timeout_sec (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('land', timeout_sec, vehicle_name)   

    def goHomeAsync(self, timeout_sec = 3e+38, vehicle_name = ''):
        """
        
        Args:
            timeout_sec (float, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('goHome', timeout_sec, vehicle_name)

    # APIs for control
    def moveByRollPitchYawZAsync(self, roll, pitch, yaw, z, duration, vehicle_name = ''):
        """
        - z is given in local NED frame of the vehicle.  
        - Roll angle, pitch angle, and yaw angle set points are given in **radians**, in the body frame. 
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness. 

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor. 
            | Clockwise rotation about this axis defines a positive **roll** angle.    
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame. 

            - Y axis is along the **Left** direction of the quadrotor.  
            | Clockwise rotation about this axis defines a positive **pitch** angle.    
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame. 

            - Z axis is along the **Up** direction. 
            | Clockwise rotation about this axis defines a positive **yaw** angle. 
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane. 
        
        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw (float): Desired yaw angle, in radians.
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByRollPitchYawZ', roll, -pitch, -yaw, z, duration, vehicle_name)

    def moveByRollPitchYawThrottleAsync(self, roll, pitch, yaw, throttle, duration, vehicle_name = ''):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw angle are given in **radians**, in the body frame. 
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness. 

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor. 
            | Clockwise rotation about this axis defines a positive **roll** angle.    
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame. 

            - Y axis is along the **Left** direction of the quadrotor.  
            | Clockwise rotation about this axis defines a positive **pitch** angle.    
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame. 

            - Z axis is along the **Up** direction. 
            | Clockwise rotation about this axis defines a positive **yaw** angle. 
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane. 
                
        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw (float): Desired yaw angle, in radians.
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByRollPitchYawThrottle', roll, -pitch, -yaw, throttle, duration, vehicle_name)

    def moveByRollPitchYawrateThrottleAsync(self, roll, pitch, yaw_rate, throttle, duration, vehicle_name = ''):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw rate set points are given in **radians**, in the body frame. 
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness. 

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor. 
            | Clockwise rotation about this axis defines a positive **roll** angle.    
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame. 

            - Y axis is along the **Left** direction of the quadrotor.  
            | Clockwise rotation about this axis defines a positive **pitch** angle.    
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame. 

            - Z axis is along the **Up** direction. 
            | Clockwise rotation about this axis defines a positive **yaw** angle. 
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane. 
                
        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw_rate (float): Desired yaw rate, in radian per second.
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByRollPitchYawrateThrottle', roll, -pitch, -yaw_rate, throttle, duration, vehicle_name)

    def moveByRollPitchYawrateZAsync(self, roll, pitch, yaw_rate, z, duration, vehicle_name = ''):
        """
        - z is given in local NED frame of the vehicle.  
        - Roll angle, pitch angle, and yaw rate set points are given in **radians**, in the body frame. 
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness. 

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor. 
            | Clockwise rotation about this axis defines a positive **roll** angle.    
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame. 

            - Y axis is along the **Left** direction of the quadrotor.  
            | Clockwise rotation about this axis defines a positive **pitch** angle.    
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame. 

            - Z axis is along the **Up** direction. 
            | Clockwise rotation about this axis defines a positive **yaw** angle. 
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane. 
                
        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw_rate (float): Desired yaw rate, in radian per second.
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to 
       
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByRollPitchYawrateZ', roll, -pitch, -yaw_rate, z, duration, vehicle_name)

    def moveByAngleRatesZAsync(self, roll_rate, pitch_rate, yaw_rate, z, duration, vehicle_name = ''):
        """
        - z is given in local NED frame of the vehicle.  
        - Roll rate, pitch rate, and yaw rate set points are given in **radians**, in the body frame. 
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness. 

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor. 
            | Clockwise rotation about this axis defines a positive **roll** angle.    
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame. 

            - Y axis is along the **Left** direction of the quadrotor.  
            | Clockwise rotation about this axis defines a positive **pitch** angle.    
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame. 

            - Z axis is along the **Up** direction. 
            | Clockwise rotation about this axis defines a positive **yaw** angle. 
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane. 
        
        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByAngleRatesZ', roll_rate, -pitch_rate, -yaw_rate, z, duration, vehicle_name)

    def moveByAngleRatesThrottleAsync(self, roll_rate, pitch_rate, yaw_rate, throttle, duration, vehicle_name = ''):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll rate, pitch rate, and yaw rate set points are given in **radians**, in the body frame. 
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness. 

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor. 
            | Clockwise rotation about this axis defines a positive **roll** angle.    
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame. 

            - Y axis is along the **Left** direction of the quadrotor.  
            | Clockwise rotation about this axis defines a positive **pitch** angle.    
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame. 

            - Z axis is along the **Up** direction. 
            | Clockwise rotation about this axis defines a positive **yaw** angle. 
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane. 

        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByAngleRatesThrottle', roll_rate, -pitch_rate, -yaw_rate, throttle, duration, vehicle_name)

    def moveByVelocityAsync(self, vx, vy, vz, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        """
        
        Args:
            vx (float): desired velocity in world (NED) X axis 
            vy (float): desired velocity in world (NED) Y axis
            vz (float): desired velocity in world (NED) Z axis
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional): Description
            yaw_mode (YawMode, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByVelocity', vx, vy, vz, duration, drivetrain, yaw_mode, vehicle_name)

    def moveByVelocityZAsync(self, vx, vy, z, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        """
        
        Args:
            vx (float): desired velocity in world (NED) X axis 
            vy (float): desired velocity in world (NED) Y axis
            z (float): desired altitude in world (NED) Z axis
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional): Description
            yaw_mode (YawMode, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByVelocityZ', vx, vy, z, duration, drivetrain, yaw_mode, vehicle_name)

    def moveOnPathAsync(self, path, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), 
        lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        """
        
        Args:
            path (TYPE): Description
            velocity (TYPE): Description
            timeout_sec (float, optional): Description
            drivetrain (TYPE, optional): Description
            yaw_mode (TYPE, optional): Description
            lookahead (TYPE, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            TYPE: Description
        """
        return self.client.call_async('moveOnPath', path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)

    def clearTrajectory(self, vehicle_name = ''):
        """
        Clears, and stops following the current trajectory (see moveOnSpline() or moveOnSplineVelConstraintsAsyn,c if any. 
        
        Args:
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            TYPE: Description
        """

        self.client.call('clearTrajectory', vehicle_name)

    def setTrajectoryTrackerGains(self, gains=TrajectoryTrackerGains(), vehicle_name = ''):
        """
        - Sets trajectory tracker gains for moveOnSplineAsync, moveOnSplineVelConstraintsAsync. 
        - Must be called once before either of the moveOnSpline*() APIs is called
        
        Args:
            gains (TrajectoryTrackerGains): Pass TrajectoryTrackerGains() to set default values. Look at TrajectoryTrackerGains to set custom gains
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call('setTrajectoryTrackerGains', *((gains.to_list(),)+(vehicle_name,)))

    def setVelocityControllerGains(self, velocity_gains=VelocityControllerGains(), vehicle_name = ''):
        """
        - Sets velocity controller gains for moveByVelocityAsync().
        - This function should only be called if the default velocity control PID gains need to be modified.
        - Modifying the velocity controller gains will have an effect on the trajectory tracking behavior of moveOnSpline*() APIs, 
            as moveOnSpline*() APIs use a controller on the lines of the pure-pursuit approach, which is tracking the reference position and
            velocities of the reference trajectories, while minimizing cross-track errors in both position and velocity state, 
            by sending velocity commands (via moveByVelocityAsync()) in the backend.  
            If you change this, it might be worth playing with the gains of moveOnSpline() by using setTrajectoryTrackerGains()
        - Passing VelocityControllerGains() sets gains to default airsim values. 

        Args:
            velocity_gains (VelocityControllerGains): 
                - Correspond to the world X, Y, Z axes. 
                - Pass VelocityControllerGains() to reset gains to default recommended values.
                - Modifying velocity controller gains will have an affect on the behaviour of moveOnSplineAsync() and moveOnSplineVelConstraintsAsync(), as they both use velocity control to track the trajectory. 
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call('setVelocityControllerGains', *(velocity_gains.to_lists()+(vehicle_name,)))

    def setAngleLevelControllerGains(self, angle_level_gains=AngleLevelControllerGains(), vehicle_name = ''):
        """
        - Sets angle level controller gains (used by any API setting angle references - for ex: moveByRollPitchYawZAsync(), moveByRollPitchYawThrottleAsync(), etc)
        - Modifying these gains will also affect the behaviour of moveByVelocityAsync() and moveOnSpline*() APIs. 
            This is because the AirSim flight controller will track velocity setpoints by converting them to angle set points.  
        - This function should only be called if the default angle level control PID gains need to be modified.
        - Passing AngleLevelControllerGains() sets gains to default airsim values. 

        Args:
            angle_level_gains (AngleLevelControllerGains): 
                - Correspond to the roll, pitch, yaw axes, defined in the body frame. 
                - Pass AngleLevelControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call('setAngleLevelControllerGains', *(angle_level_gains.to_lists()+(vehicle_name,)))

    def setAngleRateControllerGains(self, angle_rate_gains=AngleRateControllerGains(), vehicle_name = ''):
        """
        - Modifying these gains will have an affect on *ALL* move*() APIs. 
            This is because any velocity setpoint is converted to an angle level setpoint which is tracked with an angle level controllers. 
            That angle level setpoint is itself tracked with and angle rate controller.  
        - This function should only be called if the default angle rate control PID gains need to be modified.

        Args:
            angle_rate_gains (AngleRateControllerGains): 
                - Correspond to the roll, pitch, yaw axes, defined in the body frame. 
                - Pass AngleRateControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call('setAngleRateControllerGains', *(angle_rate_gains.to_lists()+(vehicle_name,)))


    def setPositionControllerGains(self, position_gains=PositionControllerGains(), vehicle_name = ''):
        """
        Sets position controller gains for moveByPositionAsync.
        This function should only be called if the default position control PID gains need to be modified.

        Args:
            position_gains (PositionControllerGains): 
                - Correspond to the X, Y, Z axes. 
                - Pass PositionControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call('setPositionControllerGains', *(position_gains.to_lists()+(vehicle_name,)))

    def moveOnSplineAsync(self, waypoints, vel_max=15.0, acc_max=7.5, 
    		add_position_constraint=True, add_velocity_constraint=True, add_acceleration_constraint=False, 
    		viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0], 
    		replan_from_lookahead=True, replan_lookahead_sec=1.0, vehicle_name = ''):
        """
        - Fits a minimum jerk trajectory to the list of given 3D waypoints (specified by the waypoints parameter).
        - Uses ETHZ-ASL's `mav_trajectory_generation <https://github.com/ethz-asl/mav_trajectory_generation>`_ as the trajectory planning backend.
        - Tracks the references positions and velocities using a pure pursuit tracking controller. 
        - The gains of the pure pursuit tracking controller are set by setTrajectoryTrackerGains.
        - Reference yaws are allocated along the tangent of the trajectory. Hence the drone will always look at the direction along which it is flying, behaving like a 3D car.
        - Note: setTrajectoryTrackerGains() must be called once before calling moveOnSpline()

        Args:
            waypoints (list[Vector3r]): 
                - List of 3D waypoints, defined in local NED frame of the vehicle to track.
            vel_max (float, optional): 
                - Maximum magnitude of velocity along trajectory
            acc_max (float, optional):
                - Maximum magnitude of acceleration along trajectory
            add_position_constraint (bool, optional): 
                - Add a start constraint at current position, so that the planned trajectory is smooth if the drone is already moving. 
                - If replan_from_lookahead is False, and add_position_constraint is False, trajectory starts from the first element of the "waypoints" list param. 
                - If replan_from_lookahead is False, and add_position_constraint is True, a position constraint is added at current odometry, and so the trajectory starts from current position. 
                - If replan_from_lookahead is True, a position constraint trajectory is always added at look-ahead point regardless of the value of "add_position_constraint", and so the trajectory starts from the lookahead point. 
                - See below for the definition of "look-ahead point".   
            add_velocity_constraint (bool, optional): 
                - Should only be set to True if add_position_constraint is True. 
                - If replan_from_lookahead is True, a velocity constraint is added at current odometry. 
                - If replan_from_lookahead is True, a velocity constraint is added at lookahead point. 
            add_acceleration_constraint (bool, optional): 
                - Should only be set to True if add_velocity_constraint (and therefore, add_position_constraint) is True. 
                - If replan_from_lookahead is True, an acceleration constraint is added at current odometry. 
                - If replan_from_lookahead is True, an acceleration constraint is added at lookahead point. 
            viz_traj (bool, optional): 
                - set this to True to visualize trajectory in unreal window. 
                - Note that visualization would appear in the FPV image, so this should only be used for debugging. 
            viz_traj_color_rgba (list, optional): 
                - list of 4 floats from 0.0 to 1.0 that determines RGBA value of trajectory visualization. 
                - Example: viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0] corresponds to Red
            replan_from_lookahead(bool, optional):
                - If this is set to true, the trajectory will start from the "look-ahead point" associated with the trajectory the drone is currently following. 
                - The lookahead point is defined by the value of the replan_lookahead_sec paramater sent in the *previous* call to moveOnSpline. 
            replan_lookahead_sec(float, optional): 
                - Defines the lookahead point by sampling the current trajectory replan_lookahead_sec number of seconds ahead. 
                - If replan_from_lookahead is passed as True in the *next* call to moveOnSpline, the *next* call's trajectory will start from the lookahead point defined by the *current* call's replan_lookahead_sec
            vehicle_name (str, optional): 
                - Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async('moveOnSpline', waypoints, add_position_constraint, add_velocity_constraint, add_acceleration_constraint, 
        	vel_max, acc_max, viz_traj, viz_traj_color_rgba, replan_from_lookahead, replan_lookahead_sec, vehicle_name)

    def moveOnSplineVelConstraintsAsync(self, waypoints, velocity_constraints, vel_max=15.0, acc_max=7.5, 
    	add_position_constraint=True, add_velocity_constraint=True, add_acceleration_constraint=False, 
    	viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 0.4], 
    	replan_from_lookahead=True, replan_lookahead_sec=1.0, vehicle_name = ''):
        """
        - Fits a minimum jerk trajectory to the list of given 3D waypoints (specified by the waypoints parameter).
        - Also adds corresponding 3D velocity vector constraints (specified by the velocity_constraints parameter).
        - Uses ETHZ-ASL's `mav_trajectory_generation <https://github.com/ethz-asl/mav_trajectory_generation>`_ as the trajectory planning backend.
        - Tracks the references positions and velocities using a pure pursuit tracking controller. 
        - The gains of the pure pursuit tracking controller are set by setTrajectoryTrackerGains.
        - Reference yaws are allocated along the tangent of the trajectory. Hence the drone will always look at the direction along which it is flying, behaving like a 3D car.
        - Reference yaws are allocated along the tangent of the trajectory. 
        - Note: setTrajectoryTrackerGains() must be called once before calling moveOnSpline()
        
        Args:
            waypoints (list[Vector3r]): 
                - List of 3D waypoints, defined in local NED frame of the vehicle to track.
            velocity_constraints (list[Vector3r]): 
                - List of 3D velocity vector constraints, defined in local NED frame of the vehicle to track.
            vel_max (float, optional):
                - Maximum magnitude of velocity along trajectory
            acc_max (float, optional): 
            - Maximum magnitude of acceleration along trajectory
            add_position_constraint (bool, optional):
                - Add a start constraint at current position, so that the planned trajectory is smooth if the drone is already moving. 
                - If replan_from_lookahead is False, and add_position_constraint is False, trajectory starts from the first element of the "waypoints" list param. 
                - If replan_from_lookahead is False, and add_position_constraint is True, a position constraint is added at current odometry, and so the trajectory starts from current position. 
                - If replan_from_lookahead is True, a position constraint trajectory is always added at look-ahead point regardless of the value of "add_position_constraint", and so the trajectory starts from the lookahead point. 
                - See below for the definition of "look-ahead point".   
            add_velocity_constraint (bool, optional):
                - Should only be set to True if add_position_constraint is True. 
                - If replan_from_lookahead is True, a velocity constraint is added at current odometry. 
                - If replan_from_lookahead is True, a velocity constraint is added at lookahead point. 
            add_acceleration_constraint (bool, optional): 
                - Should only be set to True if add_velocity_constraint (and therefore, add_position_constraint) is True. 
                - If replan_from_lookahead is True, an acceleration constraint is added at current odometry. 
                - If replan_from_lookahead is True, an acceleration constraint is added at lookahead point. 
            viz_traj (bool, optional): 
                - set this to True to visualize trajectory in unreal window. 
                - Note that visualization would appear in the FPV image, so this should only be used for debugging. 
            viz_traj_color_rgba (list, optional): 
                - list of 4 floats from 0.0 to 1.0 that determines RGBA value of trajectory visualization. 
                - Example: viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0] corresponds to Red
            replan_from_lookahead(bool, optional):
                - If this is set to true, the trajectory will start from the "look-ahead point" associated with the trajectory the drone is currently following. 
                - The lookahead point is defined by the value of the replan_lookahead_sec paramater sent in the *previous* call to moveOnSpline. 
            replan_lookahead_sec(float, optional): 
                - Defines the lookahead point by sampling the current trajectory replan_lookahead_sec number of seconds ahead. 
                - If replan_from_lookahead is passed as True in the *next* call to moveOnSpline, the *next* call's trajectory will start from the lookahead point defined by the *current* call's replan_lookahead_sec
            vehicle_name (str, optional): 
                - Name of the multirotor to send this command to 

        Returns:
            bool: Success
        """
        return self.client.call_async('moveOnSplineVelConstraints', waypoints, velocity_constraints, 
        	add_position_constraint, add_velocity_constraint, add_acceleration_constraint, 
        	vel_max, acc_max, viz_traj, viz_traj_color_rgba, replan_from_lookahead, replan_lookahead_sec, vehicle_name)

    def moveToPositionAsync(self, x, y, z, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), 
        lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        """
        
        Args:
            x (float): Description
            y (float): Description
            z (float): Description
            velocity (float): Description
            timeout_sec (float, optional): Description
            drivetrain (DrivetrainType, optional): Description
            yaw_mode (YawMode, optional): Description
            lookahead (float, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async('moveToPosition', x, y, z, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)

    def moveToZAsync(self, z, velocity, timeout_sec = 3e+38, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        """
        
        Args:
            z (float): Description
            velocity (float): Description
            timeout_sec (float, optional): Description
            yaw_mode (YawMode, optional): Description
            lookahead (float, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async('moveToZ', z, velocity, timeout_sec, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)

    def moveToYawAsync(self, yaw, timeout_sec = 3e+38, margin = 5, vehicle_name = ''):
        """
        
        Args:
            yaw_rate (float): Desired yaw angle, in **degrees per second**.
            timeout_sec (float, optional): Description
            margin (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async('rotateToYaw', yaw, timeout_sec, margin, vehicle_name)

    def moveByYawRateAsync(self, yaw_rate, duration, vehicle_name = ''):
        """
        
        Args:
            yaw_rate (float): Desired yaw rate, in **degrees per second**.
            duration (float): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async('rotateByYawRate', yaw_rate, duration, vehicle_name)

    def hoverAsync(self, vehicle_name = ''):
        """
        
        Args:
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async('hover', vehicle_name)

    def plot_transform(self, pose_list, vehicle_name = ''):
        """
        Plots a transform for a requested list of poses

        Args:
            pose_list (list[Pose]): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call('plot_tf', pose_list, 10.0, vehicle_name) 
      
    # query vehicle state
    def getMultirotorState(self, vehicle_name = ''):
        """
        
        Args:
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            MultirotorState: Description
        """
        return MultirotorState.from_msgpack(self.client.call('getMultirotorState', vehicle_name))
    getMultirotorState.__annotations__ = {'return': MultirotorState}
