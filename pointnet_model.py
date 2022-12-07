import numpy as np
import matplotlib.pyplot as plt
import scipy
import open3d as o3d

target_output = {
#     x1, y1, x2, y2, pitch, f1, f2
    "nugg_1": np.array([0.4133, 0.115, 0.4024, 0.1550, 0.524, 0.5, 0.75]),
    "nugg_2": np.array([0.4054, 0.094, 0.4535, 0.1271, 0.524, 0.5, 0.75]),
    "nugg_3": np.array([0.4156, 0.1164, 0.4132, 0.1621, 0.524, 0.5, 0.75]),
    "nugg_4": np.array([0.4017, 0.0874, 0.4305, 0.1325, 0.524, 0.5, 0.75]),
    "nugg_5": np.array([0.4115, 0.0840, 0.4457, 0.1250, 0.524, 0.5, 0.75])
}

for i in range(5):
    pcd = o3d.io.read_point_cloud(f'./data/train/nugg_{i+1}.pcd')
    print(pcd)
    xyz = np.asarray(pcd.points)
    center = np.mean(xyz, axis=0)
    print(f"Center: {center}")
    # idx = np.random.choice(np.arange(xyz.shape[0]), size=256)
    # xyz = xyz[idx]
    center = np.mean(xyz, axis=0)
    print(f"Center: {center}")

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2])

    # ax.set_xlim(0.35, 0.5)
    # ax.set_ylim(0.05, 0.2)
    # ax.set_zlim(0, 0.0125)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()