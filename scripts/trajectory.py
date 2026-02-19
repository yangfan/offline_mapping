import sys
import numpy as np
import matplotlib.pyplot as plt

def draw_traj(kf_info):
    kfs = np.loadtxt(kf_info, skiprows=1)
    print(f"r: {kfs.shape[0]}, c: {kfs.shape[1]} ")

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    ax.plot(kfs[:, 7], kfs[:, 8], kfs[:, 9], 'r-', linewidth=1, label="LIO")
    ax.plot(kfs[:, 10], kfs[:, 11], kfs[:, 12], 'b-', linewidth=1, label="GNSS")
    ax.plot(kfs[:, 17], kfs[:, 18], kfs[:, 19], 'g-', linewidth=1, label="OPT1")
    ax.plot(kfs[:, 24], kfs[:, 25], kfs[:, 26], 'y-', linewidth=1, label="OPT2")
    ax.set_zlim([-10, 100])
    ax.set_aspect("equalxy")
    ax.set_title("Keyframe 3D position")
    ax.grid(True)
    ax.legend()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    lio = ax.scatter(kfs[:, 7], kfs[:, 8], s=1, c='r')
    gnss = ax.scatter(kfs[:, 10], kfs[:, 11], s=1, c='b')
    opt1 = ax.scatter(kfs[:, 17], kfs[:, 18], s=1, c='g')
    opt2 = ax.scatter(kfs[:, 24], kfs[:, 25], s=1, c='y')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Keyframe 2D position")
    ax.set_aspect("equal")
    ax.legend([lio, gnss, opt1, opt2], ["LIO", "GNSS", "OPT1", "OPT2"])

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Please provide keyframe txt.")
        exit(1)
    else:
        kf_info = sys.argv[1]
        draw_traj(kf_info)