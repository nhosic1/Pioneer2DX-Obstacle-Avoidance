import matplotlib.pyplot as plt
# matplotlib.use('tkagg')
import numpy as np
import math

def plotReconstruction2D(points2D, camera_pose, left_limit, right_limit, z_limit, x_avoid, angle):
    
    X = [x[0] for x in points2D]
    Z = [z[1] for z in points2D]

    sgn = np.sign(angle)

    x_pom = 0.2/math.sin((math.radians(90 - abs(angle))))

    ax = plt.axes()
    ax.scatter(X, Z, c='blue', s=3)
    ax.scatter([camera_pose[0]], [camera_pose[1]], c='blue', s=30, marker="s")
    ax.plot([left_limit, right_limit], [z_limit, z_limit], c='red', linewidth=1)
    ax.plot([0, x_avoid], [0, z_limit], c='green', linewidth=2)
    # ax.quiver(0, 0, x_avoid, z_limit, angles='xy', scale_units='xy' ,scale=1, width=0.004, color='green')
    ax.plot([x_pom, x_avoid + x_pom], [0, z_limit], 'g--', linewidth=2)
    ax.plot([-x_pom, x_avoid - x_pom], [0, z_limit], 'g--', linewidth=2)  # Sirina putanje nije dobra

    camera_hfov = 1.047*0.85
    x_max = math.tan(camera_hfov/2)*1.5
    x_min = - math.tan(camera_hfov/2)*1.5
    ax.plot([0, x_max], [0, 1.5], 'r--', linewidth=1)
    ax.plot([0, x_min], [0, 1.5], 'r--', linewidth=1)

    ax.set_xlabel('x')
    ax.set_ylabel('z')
    plt.axis('scaled')
    plt.grid()

    plt.show()

def plotReconstruction3D(points3D, camera_pose, origin):
    
    X = [x[0] for x in points3D]
    Y = [y[1] for y in points3D]
    Z = [z[2] for z in points3D]

    ax = plt.axes(projection='3d')
    ax.scatter3D(X, Y, Z, c='black', s=1, depthshade=False)
    ax.scatter3D(camera_pose[0], camera_pose[1], camera_pose[2], c='blue', s=20, marker="s")
    ax.scatter3D(origin[0], origin[1], origin[2], c='red', s=20, marker="s")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()

def plotPoses(pose0, pose1, pose2, points3D):
    points3D = points3D[:3].T
    X = [x[0] for x in points3D]
    Y = [y[1] for y in points3D]
    Z = [z[2] for z in points3D]

    fig = plt.figure()

    ax = plt.axes(projection='3d')
    ax.scatter3D(X, Y, Z, c='black', s=1, depthshade=False)
    ax.scatter3D(pose0[0], pose0[1], pose0[2], c='blue', s=20, marker="s")
    ax.scatter3D(pose1[0], pose1[1], pose1[2], c='blue', s=20, marker="s")
    ax.scatter3D(pose2[0], pose2[1], pose2[2], c='blue', s=20, marker="s")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()

