import numpy as np

mass = 0.02
mass /= 12
w = 0.05
h = 0.04
d = 0.012

# Moment of Inertia for a cuboid
I = np.array(([mass*(h**2 + d**2), 0, 0],
             [0, mass*(w**2 + h**2), 0],
             [0, 0, mass*(w**2 + d**2)]))

print(f'ixx="{I[0,0]:.7f}" iyy="{I[1,1]:.7f}" izz="{I[2,2]:.7f}" ixy="0" ixz="0" iyz="0"')