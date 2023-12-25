## Overview

Our goal is to maximize the final altitude of a vertically launched rocket.

We can control the thrust of the rocket, and must take account of the rocket
mass, fuel consumption rate, gravity, and aerodynamic drag.

Let us consider the basic description of the model (for the full description,
including parameters for the rocket, see [COPS3](https://www.mcs.anl.gov/~more/cops/cops3.pdf)).

There are three state variables in our model:

* Velocity: $x_v(t)$
* Altitude: $x_h(t)$
* Mass of rocket and remaining fuel, $x_m(t)$

and a single control variable:

* Thrust: $u_t(t)$.

There are three equations that control the dynamics of the rocket:

 * Rate of ascent: $$\frac{d x_h}{dt} = x_v$$
 * Acceleration: $$\frac{d x_v}{dt} = \frac{u_t - D(x_h, x_v)}{x_m} - g(x_h)$$
 * Rate of mass loss: $$\frac{d x_m}{dt} = -\frac{u_t}{c}$$

where drag $D(x_h, x_v)$ is a function of altitude and velocity, gravity
$g(x_h)$ is a function of altitude, and $c$ is a constant.

These forces are defined as:

$$D(x_h, x_v) = D_c \cdot x_v^2 \cdot e^{-h_c \left( \frac{x_h-x_h(0)}{x_h(0)} \right)}$$
and
$$g(x_h) = g_0 \cdot \left( \frac{x_h(0)}{x_h} \right)^2$$

We use a discretized model of time, with a fixed number of time steps, $T$.

Our goal is thus to maximize $x_h(T)$.

````julia
import DirectOptimalControl as DOC
using JuMP
import Ipopt
using GLMakie
````

## Set solver configuration
Let us set first create an optimal control problem. The structure which stores all the data related to the
optimal control problem is called OCP (Optimal control problem).

````julia
OC = DOC.OCP()
````

Now we will set various parameters for the solver
* OC.tol : Sets up tolerence for the solver. In the intial run it is advisable to keep the tolerance height
* OC.mesh_iter_max : This is the maximum number of iterations that the solver takes
* OC.objective_sense: You can set two options here "Max" or "Min" depending on weather the objective is to be minimized or maximized

````julia
OC.tol = 1e-7
OC.mesh_iter_max = 10
OC.objective_sense = "Max"
````

Now it is time to select an optimizer. Let us select the Ipopt optimizer. The OC struct contains an JuMP model in its field
OC.model. We can assign any non-linear optimizer that JuMP supports. For this reason we needed to import JuMP. It is advisable to
initially keep the solver tolerance higher so that the optimizer converges.

````julia
set_optimizer(OC.model, Ipopt.Optimizer)
set_attribute(OC.model, "print_level", 0)
````

set_attribute(OC.model, "max_iter", 500)
set_attribute(OC.model, "tol", 1e-4)

# Define the models and cost functions
Now let us define the objectives and functions which make up the model

````julia
h0 = 1                      # Initial height
v0 = 0                      # Initial velocity
m0 = 1.0                    # Initial mass
mT = 0.6                    # Final mass
g0 = 1                      # Gravity at the surface
hc = 500                    # Used for drag
c = 0.5 * sqrt(g0 * h0)     # Thrust-to-fuel mass
Dc = 0.5 * 620 * m0 / g0    # Drag scaling
utmax = 3.5 * g0 * m0       # Maximum thrust
Tmax = 0.2                  # Number of seconds

x0 = [h0, v0, m0]

p = (g0 = g0, hc = hc, c = c, Dc = Dc, xh0 = h0, utmax = utmax, x0 = x0)

ns = 3
nu = 1
n = 1000
````

### System dynamics
Note that the dyn function which defines the dynamics must be in a particular format.
It must five inputs:
x : The state of system at time t
u : The input of system at time t
t : The time t
p : p is a named tuple of auzillary parameters required to define the fucntion
k : k is a named tuple containing kg and kp

````julia
D(xh, xv, p) = p.Dc*(xv^2)*exp(-p.hc*(xh - p.xh0)/p.xh0)
g(xh, p) = p.g0*(p.xh0/xh)^2
function dyn(x, u, t, p)
    xhn = x[2]
    xvn = (u[1] - D(x[1], x[2], p))/x[3] - g(x[1], p)
    xmn = -u[1]/p.c
    return [xhn, xvn, xmn]
end

function integralfun(x, u, t, p)
    return nothing
end
````

### Objective Function
The objective function consists of running cost and a fixed cost.
The running cost function also has syntax similar to the dynamics function.
For the rocket example there is no running cost involved so the running
cost function returns 0.

````julia
function L(x, u, t, p)
    return 0.0
end
````

The Final cost function involves the contribution of final state in the objective
Since we want to maximize the final height the function returns `xf[1]`. This is because
the first state denotes the height as per our definition of the heigth function.

````julia
function phi(xf, uf, tf, p)
    return xf[1]
end
````

### Integral functions
Some of the problems can have integral constraints associated with them in each phase
Since this problem does not have an integral constraint the `integralfun` will return `nothing`.

````julia
function integralfun(x, u, t, p)
    return nothing
end
````

### Path functions
Some of the problems can have path constraints associated with them in each phase
Since this problem does not have an path constraint the `pathfun` will return `nothing`.

````julia
function pathfun(x, u, t, p)
    return nothing
end

function integralfun(x, u, t, p)
    return nothing
end
````

# Assignement to Phase object
Each OCP must contain atleast one phase. The synatax for adding the phase
to an OCP is given by

````julia
ph = DOC.PH(OC)
````

Now let us assign the various functions defined above to the phase `ph` that we have
just created

````julia
ph.L = L      # Adding the running cost
ph.phi = phi  # Adding the final time cost
ph.dyn = dyn  # Add the dynamics
ph.integralfun = integralfun # Add the integral constraint function
ph.pathfun = pathfun # Add the integral constraint function
ph.n = n    # Number of points in initial mesh
ph.ns = ns  # State dimension
ph.nu = nu  # Input dimension
ph.nq = 0   # Dimension of the quadrature (integral) constraint
ph.nk = 0   # Dimension of the optimizable phase paramters
ph.np = 0   # Dimension of the path constraints
ph.p = p    # Auxillary parametrs named tuple
````

Let us select some of the options for the phase
* Collocation method: Two options ["hermite-simpson", "trapezoidal"]. Default is "hermite-simpson"
* Scale: Two options `true` or `false`

````julia
ph.collocation_method = "hermite-simpson"
ph.scale_flag = true
````

Now let us set the upper bounds and lower bounds on all the variables
pathconstaints and integral constraints

````julia
ph.limits.ll.u = [0.0]      # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.u = [p.utmax]  # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.x = [0.0, 0.0, 0.0] # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.x = [2.0, 2.0, 2.0] # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.xf = [0.3, 0, mT]      # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.xf = [2.0, 2.0, 2.0]   # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.xi = p.x0  # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.xi = p.x0  # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.ti = 0.0   # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.ti = 0.0   # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.tf = 0.2   # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.tf = 0.2   # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.dt = 0.0     # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.dt = 0.2     # Upper bounds on input. Vector of dimesion `ns`
````

ph.limits.ll.k = [] # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.k = [] # Upper bounds on input. Vector of dimesion `ns`
Add the boundary constraints

### Set intial values
There are two options here "Auto" and "Manual"

````julia
ph.set_initial_vals = "Auto"
ph.tau = range(start = 0, stop = 1, length = ph.n)
ph.xinit = ones(ph.ns, ph.n)
ph.uinit = ones(ph.nu, ph.n)
ph.tfinit = ph.limits.ll.tf
ph.tiinit = ph.limits.ll.ti
````

ph.kinit  = (ph.limits.ll.k + ph.limits.ul.k)/2

Specify initial value
OC.obj_llim = -2.0
OC.obj_ulim = 2.0

````julia
OC.psi_llim = [0.0]
OC.psi_ulim = [0.0]
````

OC.kg_llim = [0.0, 0.0, 0.0]
OC.kg_ulim = [1.0, 1.0, 1.0]

````julia
OC.nkg = 0     #  Number of global parameters  Optional parameter
OC.npsi = 1    # Optional parameter evnts
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    v1 = ph[1].u[:, end]

    return [v1;]
````

return nothing

````julia
end

OC.psi = psi
````

Call function to setup the JuMP model for solving optimal control problem

````julia
DOC.setup_mpocp(OC)
````

Solve for the control and state

````julia
DOC.solve_mpocp(OC)
````

DOC.solve(OC)

````julia
solution_summary(OC.model)
````

Display results

````julia
println("Objective Value: ", objective_value(OC.model))

f = Figure()
ax1 = Axis(f[1,1])
lines!(ax1, value.(ph.t), value.(ph.x[1,:]))
ax2 = Axis(f[2,1])
lines!(ax2, value.(ph.t), value.(ph.x[2,:]))
ax3 = Axis(f[1, 2])
lines!(ax3, value.(ph.t), value.(ph.x[3,:]))
ax4 = Axis(f[2, 2])
lines!(ax4,value.(ph.t), value.(ph.u[1,:]))
display(f)
````

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

