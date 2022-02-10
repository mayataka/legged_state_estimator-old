import a1_simulator
import numpy as np
import scipy
import legged_state_estimator 
from scipy.spatial.transform import Rotation


PATH_TO_URDF = "a1_description/urdf/a1.urdf"
time_step = 0.0025
sim = a1_simulator.A1Simulator(PATH_TO_URDF, time_step)

estimator_settings = legged_state_estimator.StateEstimatorSettings.a1_settings(PATH_TO_URDF, time_step)
estimator_settings.contact_estimator_settings.beta0 = -4.0
estimator_settings.contact_estimator_settings.beta1 = 0.25
estimator = legged_state_estimator.StateEstimator(estimator_settings)

sim.init()
base_pos, base_quat, base_lin_vel, base_ang_vel = sim.get_base_state()
estimator.init(base_pos=base_pos, base_quat=base_quat, base_lin_vel=base_lin_vel,
               imu_gyro_bias=np.zeros(3), imu_lin_accel_bias=np.zeros(3))
for i in range(10000):
    sim.step_simulation()
    if i%100 == 0:
        qJ_cmd = 0.01 * np.random.normal(12) + sim.qJ_ref
        sim.apply_position_command(qJ_cmd)
    base_ang_vel, base_lin_acc = sim.get_imu_state()
    qJ, dqJ, ddqJ, tauJ = sim.get_joint_state()
    estimator.update(imu_gyro_raw=base_ang_vel, imu_lin_accel_raw=base_lin_acc, 
                     qJ=qJ, dqJ=dqJ, ddqJ=ddqJ, tauJ=tauJ, f=[0, 0, 0, 0])
    base_pos, base_quat, base_lin_vel, base_ang_vel = sim.get_base_state()
    R_true = Rotation.from_quat(base_quat).as_matrix()
    diff = Rotation.from_matrix(R_true@estimator.base_rotation_estimate).as_quat()[0:3]
    print('base_pos error:', base_pos-estimator.base_position_estimate)
    print('base_orn error:', diff)
    print('base_lin_vel error:', base_lin_vel-estimator.base_linear_velocity_estimate)
    print('contact_probability:', estimator.contact_probability)
    print('contact_force_estimate:', estimator.contact_force_estimate)

    # print(estimator.base_quaternion_estimate)
    # print(estimator.base_linear_velocity_estimate)
    # print(estimator.base_angular_velocity_estimate)

# from scipy.spatial.transform import Rotation
# robot = legged_state_estimator.Robot(PATH_TO_URDF, est_settings.contact_frames)
# pos = np.zeros(3)
# sth = np.sin(np.pi/3)
# cth = np.cos(np.pi/3)
# # rot = np.array([[cth, -sth, 0],
# #                 [sth,  cth, 0],
# #                 [  0,    0, 1]])
# quat = Rotation.from_matrix(np.eye(3)).as_quat()
# lin_vel = np.zeros(3)
# ang_vel = np.array([0, 0, 1])
# robot.update_base_configuration(pos, quat, lin_vel, ang_vel, 1.0)
# # print(robot.base_position)
# print(Rotation.from_quat(robot.base_orientation).as_matrix())
# # print(robot.base_linear_velocity)

# rot = np.array([[cth, -sth, 0],
#                 [sth,  cth, 0],
#                 [  0,    0, 1]])
