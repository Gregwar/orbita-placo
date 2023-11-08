import placo
import time
import numpy as np
from placo_utils.visualization import robot_viz, robot_frame_viz
from placo_utils.tf import tf
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--joint", action="store_true")
args = parser.parse_args()

robot = placo.RobotWrapper("orbita/", placo.Flags.ignore_collisions)
robot.set_T_world_fbase(np.eye(4))
viz = robot_viz(robot)

solver = robot.make_solver()
solver.mask_fbase(True)

# Adding closure constraints
for k in [2, 3]:
    task = solver.add_relative_position_task(f"closing_br{k}_1", f"closing_br{k}_2", np.array([0, 0, 0]))
    task.configure(f"closing_br{k}", "hard")
    task.mask.set_axises("xy")

if args.joint:
    # Creating a task to control joint
    joints_task = solver.add_joints_task()
    joints_task.set_joints({
        "ring1": 0,
        "ring2": 0,
        "ring3": 0,
    })
else:
    # Creating a task to control orientation
    orientation_task = solver.add_orientation_task("effector", np.eye(3))

dt = 0.01
t = 0

while True:
    # Updating the task's target
    if args.joint:
        joints_task.set_joints({"ring1": np.sin(t)})
    else:
        R = tf.rotation_matrix(np.sin(t*3)*.4, [1, 0, 0])[:3, :3]
        orientation_task.R_world_frame = R

    # Solving kinematics
    robot.update_kinematics()
    solver.solve(True)

    # Displaying
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")

    t += dt
    time.sleep(dt)
