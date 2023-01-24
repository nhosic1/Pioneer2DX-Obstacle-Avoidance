import matplotlib.pyplot as plt
# matplotlib.use('tkagg')
import numpy as np
import math

def plotReconstruction2D(points2D, camera_pose, tangent_point, x_avoid, angle):
    
    X = [x[0] for x in points2D]
    Z = [z[1] for z in points2D]

    sgn = np.sign(angle)
    x_pom = 0.2/math.sin((math.radians(90 - abs(angle))))

    ax = plt.axes()
    ax.scatter(X, Z, c='k', s=3)
    ax.scatter([camera_pose[0]], [camera_pose[1]], c='blue', s=65, marker="s")
    ax.scatter([tangent_point[0]], [tangent_point[1]], c='red', s=70, marker="o")
    # ax.plot([left_limit, right_limit], [z_limit, z_limit], c='red', linewidth=1)
    ax.plot([0, x_avoid], [0, tangent_point[1]], c='green', linewidth=2)
    # ax.quiver(0, 0, x_avoid, z_limit, angles='xy', scale_units='xy' ,scale=1, width=0.004, color='green')
    ax.plot([x_pom, x_avoid + x_pom], [0, tangent_point[1]], 'g--', linewidth=2)
    ax.plot([-x_pom, x_avoid - x_pom], [0, tangent_point[1]], 'g--', linewidth=2)  # Sirina putanje nije dobra
    camera_hfov = 1.047*0.85
    camera_hfov_full = 1.047
    x_max = math.tan(camera_hfov/2)*1.5
    x_min = - math.tan(camera_hfov/2)*1.5
    right_full = math.tan(camera_hfov_full/2)*1.5
    left_full = - math.tan(camera_hfov_full/2)*1.5
    ax.plot([0, x_max], [0, 1.5], 'b--', linewidth=1)
    ax.plot([0, x_min], [0, 1.5], 'b--', linewidth=1)
    ax.plot([0, right_full], [0, 1.5], 'k--', linewidth=1)
    ax.plot([0, left_full], [0, 1.5], 'k--', linewidth=1)

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

def plot_odom(X1, Y1, X2, Y2, X3, Y3, X4, Y4):
    ax = plt.axes()
    ax.scatter(X1, Y1, c='purple', s=1, label='1. metoda')
    ax.scatter(X2, Y2, c='b', s=1, label='2. metoda')
    ax.scatter(X3[600:], Y3[600:], c='limegreen', s=1, label='3. metoda')
    ax.scatter(X4[610:], Y4[610:], c='orange', s=1, label='4. metoda')
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    plt.grid()
    plt.axis('scaled')
    plt.xlim([-4, 1.5])
    plt.ylim([-2.5, 2.5])
    plt.legend(loc ="lower left", markerscale=4)
    plt.show()

def plot_single(X, Y):
    ax = plt.axes()
    ax.scatter(X, Y, c='limegreen', s=1, label='3. metoda')
    # ax.scatter(-2.470790, 1.140200, c='blue', s=6)
    # ax.scatter(2.285320, -0.612381, c='blue', s=6)
    # ax.scatter(-3.734230, -0.458001, c='blue', s=6)
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    plt.grid()
    plt.axis('scaled')
    # plt.xlim([-4, 1.5])
    # plt.ylim([-2.5, 2.5])
    plt.xlim([-5, 3])
    plt.ylim([-4, 3])
    plt.legend(loc ="lower left", markerscale=4)
    plt.show()
