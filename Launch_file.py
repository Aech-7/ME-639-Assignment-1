import mujoco
import numpy as np
from mujoco import viewer

model = mujoco.MjModel.from_xml_path('/home/aech7/mujoco_projects/Inverted_pendulum.xml')
data = mujoco.MjData(model)
hinge_qpos= model.joint("hinge").qposadr[0]
data.qpos[hinge_qpos] = np.pi
viewer.launch(model, data)