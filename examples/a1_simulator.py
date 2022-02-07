import pybullet
import pybullet_data
import numpy as np
import time


class A1Simulator:
    def __init__(self, path_to_urdf, time_step):
        self.path_to_urdf = path_to_urdf
        self.time_step = time_step
        self.calib_camera = False
        self.camera_distance = 0.0
        self.camera_yaw = 0.0
        self.camera_pitch = 0.0
        self.camera_target_pos = [0., 0., 0.]
        self.robot = None
        self.baseLinVel_prev = np.zeros(3)
        self.q = np.array([0, 0, 0.3181, 0, 0, 0, 1, 
                           0.0,  0.67, -1.3, 
                           0.0,  0.67, -1.3, 
                           0.0,  0.67, -1.3, 
                           0.0,  0.67, -1.3])
        self.qJ_ref = np.array([0.0,  0.67, -1.3, 
                                0.0,  0.67, -1.3, 
                                0.0,  0.67, -1.3, 
                                0.0,  0.67, -1.3])

    def set_urdf(self, path_to_urdf):
        self.path_to_urdf = path_to_urdf

    def set_camera(self, camera_distance, camera_yaw, camera_pitch, camera_target_pos):
        self.calib_camera = True
        self.camera_distance = camera_distance
        self.camera_yaw = camera_yaw
        self.camera_pitch = camera_pitch
        self.camera_target_pos = camera_target_pos

    def init(self, q=None):
        if q is not None:
            self.q = q
        pybullet.connect(pybullet.GUI)
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setTimeStep(self.time_step)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane = pybullet.loadURDF("plane.urdf")
        self.robot = pybullet.loadURDF(self.path_to_urdf,  
                                       useFixedBase=False, 
                                        useMaximalCoordinates=False)
        self.init_state(self.q[0:3], self.q[3:7], self.q[7:19])

    def step_simulation(self):
        pybullet.stepSimulation()
        time.sleep(self.time_step)

    def get_base_state(self):
        basePos, baseOrn = pybullet.getBasePositionAndOrientation(self.robot)
        baseLinVel, baseAngVel = pybullet.getBaseVelocity(self.robot)
        return np.array(basePos), np.array(baseOrn), np.array(baseLinVel), np.array(baseAngVel)

    def get_imu_state(self):
        basePos, baseOrn, baseLinVel, baseAngVel = self.get_base_state()
        baseLinAcc = (baseLinVel - self.baseLinVel_prev) / self.time_step
        self.baseLinVel_prev = baseLinVel
        return baseAngVel, baseLinAcc

    def get_joint_state(self):
        # joint angles
        qJ = np.zeros(12)
        # FL
        qJ[0] = pybullet.getJointState(self.robot, 7)[0]
        qJ[1] = pybullet.getJointState(self.robot, 9)[0]
        qJ[2] = pybullet.getJointState(self.robot, 10)[0]
        # FR
        qJ[3] = pybullet.getJointState(self.robot, 2)[0]
        qJ[4] = pybullet.getJointState(self.robot, 4)[0]
        qJ[5] = pybullet.getJointState(self.robot, 5)[0]
        # RF
        qJ[6] = pybullet.getJointState(self.robot, 17)[0]
        qJ[7] = pybullet.getJointState(self.robot, 19)[0]
        qJ[8] = pybullet.getJointState(self.robot, 20)[0]
        # RH
        qJ[9] = pybullet.getJointState(self.robot, 12)[0]
        qJ[10] = pybullet.getJointState(self.robot, 14)[0]
        qJ[11] = pybullet.getJointState(self.robot, 15)[0]
        # joint velocities
        dqJ = np.zeros(12)
        # FL
        dqJ[0] = pybullet.getJointState(self.robot, 7)[1]
        dqJ[1] = pybullet.getJointState(self.robot, 9)[1]
        dqJ[2] = pybullet.getJointState(self.robot, 10)[1]
        # FR
        dqJ[3] = pybullet.getJointState(self.robot, 2)[1]
        dqJ[4] = pybullet.getJointState(self.robot, 4)[1]
        dqJ[5] = pybullet.getJointState(self.robot, 5)[1]
        # RF
        dqJ[6] = pybullet.getJointState(self.robot, 17)[1]
        dqJ[7] = pybullet.getJointState(self.robot, 19)[1]
        dqJ[8] = pybullet.getJointState(self.robot, 20)[1]
        # RH
        dqJ[9] = pybullet.getJointState(self.robot, 12)[1]
        dqJ[10] = pybullet.getJointState(self.robot, 14)[1]
        dqJ[11] = pybullet.getJointState(self.robot, 15)[1]
        # joint torques
        tauJ = np.zeros(12)
        tauJ[0] = pybullet.getJointState(self.robot, 7)[3]
        tauJ[1] = pybullet.getJointState(self.robot, 9)[3]
        tauJ[2] = pybullet.getJointState(self.robot, 10)[3]
        # FR
        tauJ[3] = pybullet.getJointState(self.robot, 2)[3]
        tauJ[4] = pybullet.getJointState(self.robot, 4)[3]
        tauJ[5] = pybullet.getJointState(self.robot, 5)[3]
        # RF
        tauJ[6] = pybullet.getJointState(self.robot, 17)[3]
        tauJ[7] = pybullet.getJointState(self.robot, 19)[3]
        tauJ[8] = pybullet.getJointState(self.robot, 20)[3]
        # RH
        tauJ[9] = pybullet.getJointState(self.robot, 12)[3]
        tauJ[10] = pybullet.getJointState(self.robot, 14)[3]
        tauJ[11] = pybullet.getJointState(self.robot, 15)[3]
        return qJ, dqJ, tauJ

    def apply_torque_command(self, tauJ):
        mode = pybullet.TORQUE_CONTROL
        # LFL
        pybullet.setJointMotorControl2(self.robot, 7, controlMode=mode, force=tauJ[0])
        pybullet.setJointMotorControl2(self.robot, 9, controlMode=mode, force=tauJ[1])
        pybullet.setJointMotorControl2(self.robot, 10, controlMode=mode, force=tauJ[2])
        # FR 
        pybullet.setJointMotorControl2(self.robot, 2, controlMode=mode, force=tauJ[3])
        pybullet.setJointMotorControl2(self.robot, 4, controlMode=mode, force=tauJ[4])
        pybullet.setJointMotorControl2(self.robot, 5, controlMode=mode, force=tauJ[5])
        # RF
        pybullet.setJointMotorControl2(self.robot, 17, controlMode=mode, force=tauJ[6])
        pybullet.setJointMotorControl2(self.robot, 19, controlMode=mode, force=tauJ[7])
        pybullet.setJointMotorControl2(self.robot, 20, controlMode=mode, force=tauJ[8])
        # RH
        pybullet.setJointMotorControl2(self.robot, 12, controlMode=mode, force=tauJ[9])
        pybullet.setJointMotorControl2(self.robot, 14, controlMode=mode, force=tauJ[10])
        pybullet.setJointMotorControl2(self.robot, 15, controlMode=mode, force=tauJ[11])

    def apply_position_command(self, qJ):
        mode = pybullet.POSITION_CONTROL
        maxForce = 30
        Kp = 0.05
        Kd = 0.0001
        # LFL
        pybullet.setJointMotorControl2(self.robot, 7, controlMode=mode, targetPosition=qJ[0], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 9, controlMode=mode, targetPosition=qJ[1], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 10, controlMode=mode, targetPosition=qJ[2], positionGain=Kp, force=maxForce)
        # FR 
        pybullet.setJointMotorControl2(self.robot, 2, controlMode=mode, targetPosition=qJ[3], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 4, controlMode=mode, targetPosition=qJ[4], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 5, controlMode=mode, targetPosition=qJ[5], positionGain=Kp, force=maxForce)
        # RF
        pybullet.setJointMotorControl2(self.robot, 17, controlMode=mode, targetPosition=qJ[6], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 19, controlMode=mode, targetPosition=qJ[7], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 20, controlMode=mode, targetPosition=qJ[8], positionGain=Kp, force=maxForce)
        # RH
        pybullet.setJointMotorControl2(self.robot, 12, controlMode=mode, targetPosition=qJ[9], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 14, controlMode=mode, targetPosition=qJ[10], positionGain=Kp, force=maxForce)
        pybullet.setJointMotorControl2(self.robot, 15, controlMode=mode, targetPosition=qJ[11], positionGain=Kp, force=maxForce)

    def init_state(self, base_pos, base_orn, qJ):
        # Base
        pybullet.resetBasePositionAndOrientation(self.robot, base_pos, base_orn)
        # FR
        pybullet.resetJointState(self.robot, 2, qJ[3])
        pybullet.resetJointState(self.robot, 4, qJ[4])
        pybullet.resetJointState(self.robot, 5, qJ[5])
        # FL
        pybullet.resetJointState(self.robot, 7, qJ[0])
        pybullet.resetJointState(self.robot, 9, qJ[1])
        pybullet.resetJointState(self.robot, 10, qJ[2])
        # RR 
        pybullet.resetJointState(self.robot, 12, qJ[9])
        pybullet.resetJointState(self.robot, 14, qJ[10])
        pybullet.resetJointState(self.robot, 15, qJ[11])
        # RL 
        pybullet.resetJointState(self.robot, 17, qJ[6])
        pybullet.resetJointState(self.robot, 19, qJ[7])
        pybullet.resetJointState(self.robot, 20, qJ[8])