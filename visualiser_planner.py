import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import sin, cos
import os


# =========================
# Robot parameters
# =========================

L1 = 0.3
L2 = 0.25
L3 = 0.15
LINK_RADIUS = 0.02

# analytic colors
analytic_link_colors = ["#00ff88", "#00ffaa", "#00ccff"]

# neural colors
neural_link_colors = ["#ff4444", "#ff8844", "#ffaa44"]

joint_colors = ["#ffffff","#ffaa00","#cc66ff","#00e6e6"]

ghost_arms = []


# =========================
# Aspect ratio fix
# =========================

def set_equal_aspect(ax):

    x_limits=ax.get_xlim3d()
    y_limits=ax.get_ylim3d()
    z_limits=ax.get_zlim3d()

    max_range=max(
        abs(x_limits[1]-x_limits[0]),
        abs(y_limits[1]-y_limits[0]),
        abs(z_limits[1]-z_limits[0])
    )/2

    mid_x=sum(x_limits)/2
    mid_y=sum(y_limits)/2
    mid_z=sum(z_limits)/2

    ax.set_xlim(mid_x-max_range,mid_x+max_range)
    ax.set_ylim(mid_y-max_range,mid_y+max_range)
    ax.set_zlim(mid_z-max_range,mid_z+max_range)


# =========================
# Load scene
# =========================

def load_scene():

    with open("scene.txt") as f:

        obs_line=list(map(float,f.readline().split()))
        target_line=list(map(float,f.readline().split()))

    obs={
        "x":obs_line[0],
        "y":obs_line[1],
        "z":obs_line[2],
        "r":obs_line[3]
    }

    target=np.array(target_line)

    return obs,target


obs,target=load_scene()


# =========================
# FK
# =========================

def fk(t1,t2,t3):

    x0,y0,z0=0,0,0
    x1,y1,z1=0,0,L1

    x2=L2*cos(t2)*cos(t1)
    y2=L2*cos(t2)*sin(t1)
    z2=L1+L2*sin(t2)

    x3=x2+L3*cos(t2+t3)*cos(t1)
    y3=y2+L3*cos(t2+t3)*sin(t1)
    z3=z2+L3*sin(t2+t3)

    return np.array([
        [x0,y0,z0],
        [x1,y1,z1],
        [x2,y2,z2],
        [x3,y3,z3]
    ])


# =========================
# Load trajectories
# =========================

def read_file(name):

    if not os.path.exists(name):
        return []

    data=[]
    with open(name) as f:
        for line in f:
            data.append(tuple(map(float,line.split())))

    return data


analytic_traj = read_file("trajectory_safe_analytic.txt")
neural_traj   = read_file("trajectory_safe_neural.txt")

if not analytic_traj or not neural_traj:
    print("Trajectory files missing")
    exit()


# =========================
# Draw obstacle
# =========================

def draw_obstacle():

    u=np.linspace(0,2*np.pi,50)
    v=np.linspace(0,np.pi,40)

    x=obs["x"]+obs["r"]*np.outer(np.cos(u),np.sin(v))
    y=obs["y"]+obs["r"]*np.outer(np.sin(u),np.sin(v))
    z=obs["z"]+obs["r"]*np.outer(np.ones_like(u),np.cos(v))

    ax.plot_surface(x,y,z,color="#555555",alpha=0.9)


# =========================
# Draw target
# =========================

def draw_target():

    ax.scatter(
        target[0],
        target[1],
        target[2],
        color="magenta",
        s=120
    )


# =========================
# Draw link
# =========================

def draw_link(p1,p2,r,color,alpha):

    v=p2-p1
    mag=np.linalg.norm(v)

    if mag==0:
        return

    v=v/mag

    not_v=np.array([1,0,0])
    if np.allclose(v,not_v):
        not_v=np.array([0,1,0])

    n1=np.cross(v,not_v)
    n1=n1/np.linalg.norm(n1)
    n2=np.cross(v,n1)

    t=np.linspace(0,mag,20)
    theta=np.linspace(0,2*np.pi,20)

    t,theta=np.meshgrid(t,theta)

    X=p1[0]+v[0]*t+r*np.sin(theta)*n1[0]+r*np.cos(theta)*n2[0]
    Y=p1[1]+v[1]*t+r*np.sin(theta)*n1[1]+r*np.cos(theta)*n2[1]
    Z=p1[2]+v[2]*t+r*np.sin(theta)*n1[2]+r*np.cos(theta)*n2[2]

    ax.plot_surface(X,Y,Z,color=color,alpha=alpha)


# =========================
# Draw robot
# =========================

def draw_robot(joints, colors, alpha):

    for i in range(3):

        draw_link(
            joints[i],
            joints[i+1],
            LINK_RADIUS,
            colors[i],
            alpha
        )

    for i in range(4):

        ax.scatter(
            joints[i,0],
            joints[i,1],
            joints[i,2],
            color="white",
            s=60
        )


# =========================
# Animation
# =========================

fig=plt.figure(figsize=(10,8))
ax=fig.add_subplot(111,projection="3d")

frame_index=0


def update(frame):

    global frame_index

    ax.clear()

    ax.set_facecolor("#111111")
    fig.patch.set_facecolor("#111111")

    ax.set_xlim(-0.6,0.6)
    ax.set_ylim(-0.6,0.6)
    ax.set_zlim(0,0.8)

    set_equal_aspect(ax)

    draw_obstacle()
    draw_target()

    i = frame_index % min(len(analytic_traj), len(neural_traj))

    analytic_joints = fk(*analytic_traj[i])
    neural_joints   = fk(*neural_traj[i])

    # analytic arm (GREEN)
    draw_robot(
        analytic_joints,
        analytic_link_colors,
        0.9
    )

    # neural arm (RED)
    draw_robot(
        neural_joints,
        neural_link_colors,
        0.6
    )

    ax.set_title(
        "Green: Analytical IK | Red: Neural IK",
        color="yellow"
    )

    frame_index += 1


ani = FuncAnimation(
    fig,
    update,
    interval=20
)

plt.show()
