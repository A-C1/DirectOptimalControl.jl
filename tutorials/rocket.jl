# #### Goddard's rocket Model

# State variables:
# * Velocity: $x_v(t)$
# * Altitude: $x_h(t)$
# * Mass of rocket and remaining fuel, $x_m(t)$

# Control variables
# * Thrust: $u_t(t)$.

# Dynamics:
#  * Rate of ascent: $$\frac{d x_h}{dt} = x_v$$
#  * Acceleration: $$\frac{d x_v}{dt} = \frac{u_t - D(x_h, x_v)}{x_m} - g(x_h)$$
#  * Rate of mass loss: $$\frac{d x_m}{dt} = -\frac{u_t}{c}$$
# where drag $D(x_h, x_v)$ is a function of altitude and velocity, gravity
# $g(x_h)$ is a function of altitude, and $c$ is a constant.
# These forces are defined as:
# $$D(x_h, x_v) = D_c \cdot x_v^2 \cdot e^{-h_c \left( \frac{x_h-x_h(0)}{x_h(0)} \right)}$$
# $$g(x_h) = g_0 \cdot \left( \frac{x_h(0)}{x_h} \right)^2$$

# Outputs:

# Objective: 
# Maximize $x_h(T)$.

# State Constraints:
# For this probelem there are no state constraints

# Input Constraints:
# For this problem there are no input constraints

# Path Constrains:
# For this problem there are no path constraints

# Integral Constraints:
# For this problem there are no integral constraints

# #### Start code
# Include the necessary packages. `JuMP` is required to setup various configurations
# while `Ipopt` is the solver to be used. Technically all other nonlinear solvers available
# throught JuMP can be used but those have not yet been tested.
## include("../src/DirectOptimalControl.jl")
## import .DirectOptimalControl as DOC
import DirectOptimalControl as DOC
using JuMP
import Ipopt

# #### Set solver configuration
# Let us set first create an optimal control problem. The structure which stores all the data related to the 
# optimal control problem is called OCP (Optimal control problem).
OC = DOC.OCP()

# Now we will set various parameters for the solver
# * OC.tol : Sets up tolerence for the solver. In the intial run it is advisable to keep the tolerance high.
# * OC.mesh_iter_max : This is the maximum number of iterations that the solver takes
# * OC.objective_sense: You can set two options here "Max" or "Min" depending on weather the objective is to be minimized or maximized
OC.tol = 1e-7
OC.mesh_iter_max = 10
OC.objective_sense = "Max"

# Now it is time to select an optimizer. Let us select the Ipopt optimizer. The OC struct contains an `JuMP` model in its field
# `OC.model`. We can assign any non-linear optimizer that `JuMP` supports. For this reason we needed to import `JuMP`. It is advisable to
# initially keep the solver tolerance higher so that the optimizer converges. You can set all the solver specific options using the
# `JuMP` interface to aid the convergence of the solver.
set_optimizer(OC.model, Ipopt.Optimizer)
set_attribute(OC.model, "print_level", 0)
## set_attribute(OC.model, "max_iter", 500)
## set_attribute(OC.model, "tol", 1e-4)


# Each OCP must contain atleast one phase. The synatax for adding the phase to an OCP is given by
ph = DOC.PH(OC)

# #### Define the models and cost functions
# Now let us define the parameters and functions which make up the model
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

x0 = [h0, v0, m0]           # Initial state

ph.nk = 0                               # Number of auxillary phase parameters to be optimized
ph.k = @variable(OC.model, [1:ph.nk])   # Assigne them to field k in struct `ph`
OC.nkg = 0                              # Number of auxillary global parameters to be optimized
OC.kg = @variable(OC.model, [1:OC.nkg]) # Assign them to field k in struct `OC`


# #### Auxillary parameters
# Now we create a named tuple of various parameters which will be necessary while defining the problem
# Note that in addition to the constants defined above we can also pass two additional parameters `PH.kp` and
# `OC.kg`. These are additional `JuMP` variables which can be optimized if required. They will not
# be used in this example.
p = (g0 = g0, hc = hc, c = c, Dc = Dc, xh0 = h0, utmax = utmax, x0 = x0, kp = ph.k, kg = OC.kg )

ns = 3
nu = 1
n = 20

# #### System dynamics
# Note that the dyn function which defines the dynamics must be in a particular format.
# It must five inputs: 
# x : The state of system at time t
# u : The input of system at time t
# t : The time t
# p : p is a named tuple of auzillary parameters required to define the fucntion
D(xh, xv, p) = p.Dc*(xv^2)*exp(-p.hc*(xh - p.xh0)/p.xh0)
g(xh, p) = p.g0*(p.xh0/xh)^2
function dyn(x, u, t, p)
    xhn = x[2]
    xvn = (u[1] - D(x[1], x[2], p))/x[3] - g(x[1], p)
    xmn = -u[1]/p.c
    return [xhn, xvn, xmn]
end

# #### Objective Function
# The objective function consists of running cost and a fixed cost.
# The running cost function also has syntax similar to the dynamics function.
# For the rocket example there is no running cost involved so the running
# cost function returns 0.
function L(x, u, t, p)
    return 0.0
end

# The Final cost function involves the contribution of final state in the objective
# Since we want to maximize the final height the function returns `xf[1]`. This is because
# the first state denotes the height as per our definition of the heigth function.
function phi(xf, uf, tf, p)
    return xf[1]
end

# #### Integral functions
# Some of the problems can have integral constraints associated with them in each phase
# Since this problem does not have an integral constraint the `integralfun` will return `nothing`.
function integralfun(x, u, t, p)
    return nothing
end

# #### Path functions
# Some of the problems can have path constraints associated with them in each phase
# Since this problem does not have an path constraint the `pathfun` will return `nothing`.
function pathfun(x, u, t, p)
    return nothing
end

# #### Adding functions and parameters to a Phase 
# Now let us assign the various functions defined above to the phase `ph` that we have created
ph.L = L      # Adding the running cost
ph.phi = phi  # Adding the final time cost
ph.dyn = dyn  # Add the dynamics
ph.integralfun = integralfun # Add the integral constraint function
ph.pathfun = pathfun # Add the path constraint function
ph.n = n    # Number of points in initial mesh
ph.ns = ns  # State dimension
ph.nu = nu  # Input dimension
ph.nq = 0   # Dimension of the quadrature (integral) constraint
ph.np = 0   # Dimension of the path constraints
ph.p = p    # Auxillary parametrs named tuple

# Let us select some of the options for the phase 
# * Collocation method: Two options ["hermite-simpson", "trapezoidal"]. Default is "hermite-simpson"
# * Scale: Two options `true` or `false`
ph.collocation_method = "hermite-simpson"
ph.scale_flag = true


# Now let us set the upper bounds and lower bounds on all the variables
# pathconstaints and integral constraints. The upper and lower bounds are defined in the limits
# field of the `PH` structure. The limits feild contains `ll` structure which corrsponds to lower limits.
# The `ul` corresponds to upper bounds. The `ll` and `ul` structures both contain the JuMP variables
# on which we wanrt to apply upper and lower bounds. These are:
# `u` : Input
# `x` : State
# `xf` : Final State
# `xi`: Initial State
# `tf`: Final State
# `ti`: Initial time
# If we want a particular variable to have a fixwd value set both the upper limits and the lower limits
# to the same value
ph.limits.ll.u = [0.0]      # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.u = [p.utmax]  # Upper bounds on input. Vector of dimesion `nu`
ph.limits.ll.x = [0.0, 0.0, 0.0] # Lower bounds on state trajectory. Vector of dimension `ns`   
ph.limits.ul.x = [2.0, 2.0, 2.0] # Upper bounds on state trajectory. Vector of dimesion `ns`
ph.limits.ll.xf = [0.3, 0, mT]      # Lower bounds on final state. Vector of dimension `nu`
ph.limits.ul.xf = [2.0, 2.0, 2.0]   # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.xi = p.x0  # Lower bounds on initial state. Vector of dimension `ns`
ph.limits.ul.xi = p.x0  # Upper bounds on initial state. Vector of dimesion `ns`
ph.limits.ll.ti = 0.0   # Lower bounds on initial time. A Scalar.
ph.limits.ul.ti = 0.0   # Upper bounds on initial time. A Scalar.
ph.limits.ll.tf = 0.2   # Lower bounds on final time. A scalar.
ph.limits.ul.tf = 0.2   # Upper bounds on final time. A scalar.
ph.limits.ll.dt = 0.0     # Lower bounds on time interval. A scalar.
ph.limits.ul.dt = 0.2     # Upper bounds on time interval. A scalar 
ph.limits.ll.path = [] # Lower bounds on path constraint. Vector of dimension `nu`
ph.limits.ul.path = [] # Upper bounds on  path constraint. Vector of dimesion `ns`
ph.limits.ll.integral = [] # Lower bounds on integral constraint. Vector of dimension `nu`
ph.limits.ul.integral = [] # Upper bounds on integral contsraint. Vector of dimesion `ns`
ph.limits.ll.k = [] # Lower bounds on phase parameters. Vector of dimension `nu`
ph.limits.ul.k = [] # Upper bounds on phase parameters. Vector of dimesion `ns`

# #### Set intial values
# There are two options here "Auto" and "Manual". If "Auto" option is selected one need not specify tau and other init values
ph.set_initial_vals = "Auto"
ph.tau = range(start = 0, stop = 1, length = ph.n)
ph.xinit = ones(ph.ns, ph.n)
ph.uinit = ones(ph.nu, ph.n)
ph.tfinit = ph.limits.ll.tf
ph.tiinit = ph.limits.ll.ti
ph.kinit  = (ph.limits.ll.k + ph.limits.ul.k)/2

# #### Specify global parameters
# This problem only contains a single phase. However, in problems with multiple phases there are 
# parameters which are global to all the phases. We specify it here.
# ### Set limits on objective function
# It has been observed that it is better to not set it as in keep the value false. However, an option is provided to set upper
# and lower values for the objective function
OC.set_obj_lim = false
OC.obj_llim = -2.0
OC.obj_ulim = 2.0

# #### Setting up global parameters
# * OC.nkg: Number of global parameters
# * OC.kg_llim: Lower bound on global parameters
# * OC.kg_ulim: Upper bound on global parameters
# Note that these have to be passed to the function through the auxillary parameters tupple `p`
OC.kg_llim = []
OC.kg_ulim = []

# #### Setting up the event function
# *`OC.npsi`: Number of constraints in event function
# *`OC.psi_llim`: Lower bound on constraint function
# *`OC.psi_ulim`: Upper bound on constraint function
# *`OC.psi` : Function which contains the event constraints
OC.npsi = 1    
OC.psi_llim = [0.0]
OC.psi_ulim = [0.0]
# Not the format of he event function `psi`. It takes the OCP object as input.
# The OCP object has all the phases stored in it in the field `OC.ph`.
# Each phase has state variables which can be accessed by `ph.x` and input
# variable which can be accessed by `ph.u`. In this problem we want that
# `u` must have a zero value at end of the phase.
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    v1 = ph[1].u[:, end]

    return [v1;]
end
OC.psi = psi

# Callback function can be used to log variables
# Set it to return nothing if you do not want to log variables in mesh iterations
ph.callback_nt = (tau_hist = Vector{Float64}[],err_hist = Vector{Float64}[])
function callback_fun(ph::DOC.PH)
    push!(ph.callback_nt.tau_hist, deepcopy(ph.tau))
    push!(ph.callback_nt.err_hist, deepcopy(ph.error))
end
ph.callback_fun = callback_fun


# Call function to setup the JuMP model for solving optimal control problem
DOC.setup_mpocp(OC)
# Solve for the control and state
DOC.solve_mpocp(OC)
# DOC.solve(OC)
solution_summary(OC.model)

# Display results
println("Objective Value: ", objective_value(OC.model))

## using GLMakie
## f = Figure() 
## ax1 = Axis(f[1,1])
## lines!(ax1, value.(ph.t), value.(ph.x[1,:]))
## ax2 = Axis(f[2,1])
## lines!(ax2, value.(ph.t), value.(ph.x[2,:]))
## ax3 = Axis(f[1, 2])
## lines!(ax3, value.(ph.t), value.(ph.x[3,:]))
## ax4 = Axis(f[2, 2])
## lines!(ax4,value.(ph.t), value.(ph.u[1,:]))
## display(f)
 
## fth = Figure()
## axth = Axis(fth[1,1])
## n = length(ph.callback_nt.tau_hist)
## for i = 1:n
##     ni = length(ph.callback_nt.tau_hist[i])
##     tau = ph.callback_nt.tau_hist[i]
##     scatter!(axth,tau,i*ones(ni))
## end
 
## feh = Figure()
## axeh = Axis(feh[1,1])
## n = length(ph.callback_nt.err_hist)
## for i = 3:n
##     ni = length(ph.callback_nt.err_hist[i])
##     err = ph.callback_nt.err_hist[i]
##     tau = ph.callback_nt.tau_hist[i]
##     lines!(axeh,tau[1:end-1],err)
## end
