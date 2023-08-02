import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

#Made by SohamTrips
#Feel free to tweak the values and enjoy the simulation!

G = 6.67430e-11
Mass_1 = 5.972e24
Mass_2 = 7.348e24

class Body:
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = position
        self.velocity = velocity

def gravitational_force(body1, body2):
    distance = np.linalg.norm(body2.position - body1.position)
    force_magnitude = G * body1.mass * body2.mass / distance**2
    force_direction = (body2.position - body1.position) / distance
    force = force_magnitude * force_direction
    return force

def range_kutta_integration(body, other_body, dt):
    k1v = gravitational_force(body, other_body) / body.mass
    k1x = body.velocity

    k2v = gravitational_force(body, other_body) / body.mass
    k2x = body.velocity + k1v * dt / 2

    k3v = gravitational_force(body, other_body) / body.mass
    k3x = body.velocity + k2v * dt / 2

    k4v = gravitational_force(body, other_body) / body.mass
    k4x = body.velocity + k3v * dt

    body.velocity += (k1v + 2*k2v + 2*k3v + k4v) * dt / 6

    body.position += (k1x + 2*k2x + 2*k3x + k4x) * dt / 6

dt = 60 * 60
total_time = 60 * 60 * 24 * 365
num_steps = int(total_time / dt)


body1 = Body(Mass_1, np.array([0.0, 0.0, 0.0]), np.array([0.0, 100.0, 0.0]))
body2 = Body(Mass_2, np.array([3.844e8, 0.0, 0.0]), np.array([0.0, 1200.0, 0.0]))

body1_x, body1_y, body1_z = [], [], []
body2_x, body2_y, body2_z = [], [], []

for _ in range(num_steps):
    range_kutta_integration(body1, body2, dt)
    body1_x.append(body1.position[0])
    body1_y.append(body1.position[1])
    body1_z.append(body1.position[2])
    
    range_kutta_integration(body2, body1, dt)
    body2_x.append(body2.position[0])
    body2_y.append(body2.position[1])
    body2_z.append(body2.position[2])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
body1_orbit, = ax.plot([], [], [], color='blue', linestyle='-', linewidth=0.8, label='Body 1 Orbit')
body2_orbit, = ax.plot([], [], [], color='orange', linestyle='-', linewidth=0.6, label='Body 2 Orbit')
body1_body, = ax.plot([], [], [], 'o', color='blue', markersize=15, label='Body 1')
body2_body, = ax.plot([], [], [], 'o', color='orange', markersize=15, label='Body 2')
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_zlabel('Z position')
ax.set_title('Two-Body Simulation')
ax.legend()
ax.grid(True)
ax.set_box_aspect([1, 1, 1])

plot_margin = 1.2e8
ax.set_xlim(min(body1_x + body2_x) - plot_margin, max(body1_x + body2_x) + plot_margin)
ax.set_ylim(min(body1_y + body2_y) - plot_margin, max(body1_y + body2_y) + plot_margin)
ax.set_zlim(min(body1_z + body2_z) - plot_margin, max(body1_z + body2_z) + plot_margin)

def init():
    body1_orbit.set_data([], [])
    body1_orbit.set_3d_properties([])
    body2_orbit.set_data([], [])
    body2_orbit.set_3d_properties([])
    body1_body.set_data([], [])
    body1_body.set_3d_properties([])
    body2_body.set_data([], [])
    body2_body.set_3d_properties([])
    return body1_orbit, body2_orbit, body1_body, body2_body

def update(frame):
    body1_orbit.set_data(body1_x[:frame], body1_y[:frame])
    body1_orbit.set_3d_properties(body1_z[:frame])
    body2_orbit.set_data(body2_x[:frame], body2_y[:frame])
    body2_orbit.set_3d_properties(body2_z[:frame])
    body1_body.set_data(body1_x[frame], body1_y[frame])
    body1_body.set_3d_properties(body1_z[frame])
    body2_body.set_data(body2_x[frame], body2_y[frame])
    body2_body.set_3d_properties(body2_z[frame])
    return body1_orbit, body2_orbit, body1_body, body2_body

ani = FuncAnimation(fig, update, frames=num_steps, init_func=init, interval=2, blit=True)

plt.show()