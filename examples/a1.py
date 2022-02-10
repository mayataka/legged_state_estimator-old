import a1_simulator
import numpy as np
import scipy
import legged_state_estimator 


PATH_TO_URDF = "a1_description/urdf/a1.urdf"
time_step = 0.0025
sim = a1_simulator.A1Simulator(PATH_TO_URDF, time_step)

estimator_settings = legged_state_estimator.StateEstimatorSettings.a1_settings(PATH_TO_URDF, time_step)
estimator = legged_state_estimator.StateEstimator(estimator_settings)

sim.init()
for i in range(10000):
    sim.step_simulation()
    if i%1000 == 0:
        qJ = 0.025 * np.random.normal(12) + sim.qJ_ref
        sim.apply_position_command(qJ)
    base_ang_vel, base_lin_acc = sim.get_imu_state()
    qJ, dqJ, tauJ = sim.get_joint_state()
    estimator.update(imu_gyro_raw=base_ang_vel, imu_lin_accel_raw=base_lin_acc, 
                     qJ=qJ, dqJ=dqJ, tauJ=tauJ, f=np.zeros(4))
    # print(est.base_position_estimate)
    # print(est.base_orientation_estimate)
    # print(est.base_linear_velocity_estimate)

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
