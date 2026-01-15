# planetary_motion_sim
# Planetary Motion Simulator

This project simulates the motion of multiple celestial bodies under their gravitational attraction using Newton's laws of gravitation. The motion is computed by numerically integrating the constant-acceleration equations of motion and visualising the resulting trajectories. Created on Matlab.

## Physical Model

The gravitational force vector $\\mathbf{F} = [F_x, F_y, F_z]$ applied on a body of mass $m_i$ located at $(x_i, y_i, z_i)$ by a body of mass $m_j$ located at $(x_j, y_j, z_j)$ is given by

$$
\\mathbf{F}_{ij} = G \\frac{m_i m_j}{|\\mathbf{r}|^3} \\, \\mathbf{r}
$$

where

$$
\\mathbf{r} =
\\begin{pmatrix}
x_j - x_i \\\\
y_j - y_i \\\\
z_j - z_i
\\end{pmatrix}
$$

The total force acting on the $i$-th body is obtained by summing the forces from all other bodies

$$
\\mathbf{F}_i = \\sum_{j \\neq i} \\mathbf{F}_{ij}
$$

## Numerical Integration

At each timestep, the velocity and position of each body are updated assuming constant acceleration over the timestep.

Velocity update:

$$
[u_i, v_i, w_i]_\text{new} = [u_i, v_i, w_i]_\text{old} + \frac{\mathbf{F}_i}{m_i} \, \Delta t
$$

(Analogous to the SUVAT equation: $v = u + at$)

Position update:

$$
[x_i, y_i, z_i]_\text{new} = [x_i, y_i, z_i]_\text{old} + [u_i, v_i, w_i]_\text{old} \, \Delta t + \frac{\mathbf{F}_i}{2 m_i} \, \Delta t^2
$$

(Analogous to the SUVAT equation: $S = ut + \frac{\mathbf{1}}{2} \ at^2$)

## Simulation Setup

The arrays containing the mass $m$ and the initial positions $(x, y, z)$ and velocities $(u, v, w)$ for five celestial bodies close to the Sun are provided in the file `space.mat`.

The simulation uses a timestep of $\\Delta t = 3600 \\, \\mathrm{s}$ (1 hour)
and is run for $26,280$ timesteps (3 years).

## Visualisation

At each timestep, the trajectory of each body is plotted using lines and the current position is shown using circular markers. The motion is visualised in the $x$ - $y$ plane to clearly show the orbital paths.
