import mujoco
import numpy as np
from mujoco import viewer

model = mujoco.MjModel.from_xml_path('/home/aech7/robotis/robotis_mujoco_menagerie/robotis_tb3/scene_turtlebot3_waffle_pi.xml')
data = mujoco.MjData(model)

body_id = model.body('base').id
my_force = np.array([90, 0, 0])  
my_torque = np.array([0, 0, 0])
point_on_body = np.array([0, 0, 0])
force_applied = False
with viewer.launch_passive(model, data) as v:
    while v.is_running():
        data.xfrc_applied[:] = 0
        if not force_applied and data.time > 5.0:
            mujoco.mj_applyFT(model, data, my_force, my_torque, point_on_body, body_id, data.qfrc_applied)
            force_applied = True   
        mujoco.mj_step(model, data)
        v.sync()
