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

# Input Constraints:

# Path Constrains:

# Integral Constraints:

# Include the necessary packages. `JuMP` is required to setup various configurations
# while `Ipopt` is the solver to be used. Technically all other nonlinear solvers available
# throught JuMP can be used but those have not yet been tested.

## include("../src/DirectOptimalControl.jl")
## import .DirectOptimalControl as DOC

import DirectOptimalControl as DOC
using JuMP
import Ipopt

# Set solver configuration
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
set_attribute(OC.model, "max_iter", 500)
## set_attribute(OC.model, "tol", 1e-3)


# Each OCP must contain atleast one phase. The synatax for adding the phase
# to an OCP is given by
ph = DOC.PH(OC)

# #### Define the models and cost functions
# Now let us define the objectives and functions which make up the model
cft2m = 0.3048;
cft2km = cft2m/1000;
cslug2kg = 14.5939029;

# Provide Auxiliary Data for Problem 
Re = 6371203.92 # Equatorial Radius of Earth (m)
S = 249.9091776 # Vehicle Reference Area (mˆ2)
cl = [-0.2070 1.6756] # Parameters for Lift Coefficient
cdd = [0.0785 -0.3529 2.0400] # Parameters for Drag Coefficient
b = [0.07854 -0.061592 0.00621408] # Parameters for Heat Rate Model
H = 7254.24; # Density Scale Height (m)
al = [-0.20704 0.029244]; # Parameters for Heat Rate Model
rho0 = 1.225570827014494; # Sea Level Atmospheric Density (kg/mˆ3)
mu = 3.986031954093051e14; # Earth Gravitational Parameter (mˆ3/sˆ2)
mass = 92079.2525560557; # Vehicle Mass (kg)

p = (Re = Re, S = S, cl = cl, cd = cdd, b = b, H = H, al = al, rho0 = rho0, mu = mu, mass = mass)

# Boundary Conditions 
t0 = 0
alt0 = 79248
rad0 = alt0+Re
altf = +24384
radf = altf + Re
lon0 = 0
lat0 = 0
speed0 = +7802.88
speedf = +762
fpa0 = -1*pi/180
fpaf = -5*pi/180
azi0 = +90*pi/180
azif = -90*pi/180

# Limits on Variables 
tfMin = 0; tfMax = 3000;
radMin = Re; radMax = rad0;
lonMin = -pi; lonMax = -lonMin;
latMin = -70*pi/180; latMax = -latMin;
speedMin = 10; speedMax = 45000;
fpaMin = -80*pi/180; fpaMax = 80*pi/180;
aziMin = -180*pi/180; aziMax = 180*pi/180;
aoaMin = -90*pi/180; aoaMax = -aoaMin;
bankMin = -90*pi/180; bankMax = 1*pi/180;

# Provide Guess of Solution 
tGuess = [0; 1000];
radGuess = [rad0; radf];
lonGuess = [lon0; lon0+10*pi/180];
latGuess = [lat0; lat0+10*pi/180];
speedGuess = [speed0; speedf];
fpaGuess = [fpa0; fpaf];
aziGuess = [azi0; azif];
aoaGuess = [0; 0];
bankGuess = [0; 0];

# #### Auxillary parameters
# Now we create a named tuple of various parameters which will be necessary while defining the problem
# Note that in addition to the constants defined above we also add two additional parameters `PH.kp` and
# `OC.kg`. These are additional `JuMP` variables which can be optimized if required. They will not
# be used in this example.

ns = 6
nu = 2
n = 20

# #### System dynamics
# Note that the dyn function which defines the dynamics must be in a particular format.
# It must five inputs: 
# x : The state of system at time t
# u : The input of system at time t
# t : The time t
# p : p is a named tuple of auzillary parameters required to define the fucntion
# k : k is a named tuple containing kg and kp

function dyn(x, u, t, p)
    rad = x[1]; lon = x[2]; lat = x[3]; v = x[4]; fpa = x[5]; azi = x[6]
    aoa = u[1]; bank = u[2]

    cd0 = p.cd[1]
    cd1 = p.cd[2]
    cd2 = p.cd[3]
    cl0 = p.cl[1]
    cl1 = p.cl[2]
    mu = p.mu
    rho0 = p.rho0
    H = p.H
    S = p.S
    mass = p.mass
    altitude = rad - p.Re
    CD = cd0 + cd1 * aoa + cd2 * aoa^2
    rho = rho0 * exp(-altitude / H)
    CL = cl0 + cl1 * aoa
    q = 0.5 * rho * v^2
    D = q * S * CD / mass
    L = q * S * CL / mass
    gravity = mu/rad^2

    raddot = v*sin(fpa)
    londot = v*cos(fpa)*sin(azi)/(rad*cos(lat))
    latdot = v*cos(fpa)*cos(azi)/rad
    vdot = -D-gravity*sin(fpa)
    fpadot = (L*cos(bank)-cos(fpa)*(gravity-v^2/rad))/v;
    azidot = (L*sin(bank)/cos(fpa)+v^2*cos(fpa)*sin(azi)*tan(lat)/rad)/v;

    return [raddot, londot, latdot, vdot, fpadot, azidot]
end

# Objective Function
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
    return xf[3]
end

# Integral functions
# Some of the problems can have integral constraints associated with them in each phase
# Since this problem does not have an integral constraint the `integralfun` will return `nothing`.
function integralfun(x, u, t, p)
    return nothing
end

# Path functions
# Some of the problems can have path constraints associated with them in each phase
# Since this problem does not have an path constraint the `pathfun` will return `nothing`.
function pathfun(x, u, t, p)
    v = x[4]; rad = x[1];
    aoa = u[1]; bank = u[2]

    return nothing
end

function integralfun(x, u, t, p)
    return nothing
end

# # Assignement to Phase object

# Now let us assign the various functions defined above to the phase `ph` that we have
# just created
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

# Let us select some of the options for the phase 
# * Collocation method: Two options ["hermite-simpson", "trapezoidal"]. Default is "hermite-simpson"
# * Scale: Two options `true` or `false`
ph.collocation_method = "hermite-simpson"
ph.scale_flag = true


# Now let us set the upper bounds and lower bounds on all the variables
# pathconstaints and integral constraints. The upper and lower bounds are defined in the limits
# field of the PH structure. The limits feild contains `ll` structure which corrsponds to lower limits.
# The `ul` corresponds to upper bounds. The `ll` and `ul structures` both contain the JuMP variables
# on which we wanrt to apply upper and lower bounds. These are:
# `u` : Input
# `x` : State
# `xf` : Final State
# `xi`: Initial State
# `tf`: Final State
# `ti`: Initial time
# If we want a particular variable to have a fixwd value set both the upper limits and the lower limits
# to the same value
ph.limits.ll.u = [aoaMin, bankMin]      # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.u = [aoaMax, bankMax]  # Upper bounds on input. Vector of dimesion `nu`
ph.limits.ll.x =  [radMin, lonMin, latMin, speedMin, fpaMin, aziMin] # Lower bounds on state trajectory. Vector of dimension `ns`   
ph.limits.ul.x =  [radMax, lonMax, latMax, speedMax, fpaMax, aziMax] # Upper bounds on state trajectory. Vector of dimesion `ns`
ph.limits.ll.xf =  [radf, lonMin, latMin, speedf, fpaf, aziMin];    # Lower bounds on final state. Vector of dimension `nu`
ph.limits.ul.xf =  [radf, lonMax, latMax, speedf, fpaf, aziMax] # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.xi = [rad0, lon0, lat0, speed0, fpa0, azi0]  # Lower bounds on initial state. Vector of dimension `ns`
ph.limits.ul.xi = [rad0, lon0, lat0, speed0, fpa0, azi0]  # Upper bounds on initial state. Vector of dimesion `ns`
ph.limits.ll.ti = t0   # Lower bounds on initial time. A Scalar.
ph.limits.ul.ti = t0   # Upper bounds on initial time. A Scalar
ph.limits.ll.tf = tfMin   # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.tf = tfMax   # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.dt = tfMin     # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.dt = tfMax     # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.k = [] # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.k = [] # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.path = [] # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.path = [] # Upper bounds on input. Vector of dimesion `ns`

# ### Set intial values
# There are two options here "Auto" and "Manual". If "Auto" option is selected one need not specify tau and other init values
ph.set_initial_vals = "Auto"
# ph.tau = range(start = 0, stop = 1, length = ph.n)
# ph.xinit = ones(ph.ns, ph.n)
# ph.uinit = ones(ph.nu, ph.n)
# ph.tfinit = ph.limits.ll.tf
# ph.tiinit = ph.limits.ll.ti
# ph.kinit  = (ph.limits.ll.k + ph.limits.ul.k)/2

# ## Specify global parameters
# This problem only contains a single phase. However, in problems with multiple phases there are 
# parameters which are global to all the phases. We specify it here.
# ### Set limits on objective function
# It has been observed that it is better to not set it as in keep the value false. However, an option is provided to set upper
# and lower values for the objective function
OC.set_obj_lim = false
OC.obj_llim = -2.0
OC.obj_ulim = 2.0

# ## Setting up global parameters
# * OC.nkg: Number of global parameters
# * OC.kg_llim: Lower bound on global parameters
# * OC.kg_ulim: Upper bound on global parameters
# Note that these have to be passed to the function through the auxillary parameters tupple `p`
OC.nkg = 0    
OC.kg_llim = []
OC.kg_ulim = []

# ## Setting up the event function
# *`OC.npsi`: Number of constraints in event function
# *`OC.psi_llim`: Lower bound on constraint function
# *`OC.psi_ulim`: Upper bound on constraint function
# *`OC.psi` : Function which contains the event constraints
OC.npsi = 0    
OC.psi_llim = []
OC.psi_ulim = []
# Not the format of he event function `psi`. It takes the OCP object as input.
# The OCP object has all the phases stored in it in the field `OC.ph`.
# Each phase has state variables which can be accessed by `ph.x` and input
# variable which can be accessed by `ph.u`.
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    return nothing
end
OC.psi = psi


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
## for i = 1:2:ns
##     ax = Axis(f[i,1])
##     lines!(ax, value.(ph.t), value.(ph.x[i,:]))
##     if i+1 <= ns
##         ax = Axis(f[i,2])
##         lines!(ax, value.(ph.t), value.(ph.x[i+1,:]))
##     end
## end
## #     ax2 = Axis(f[2,1])
## # lines!(ax2, value.(ph.t), value.(ph.x[2,:]))
## # ax3 = Axis(f[1, 2])
## # lines!(ax3, value.(ph.t), value.(ph.x[3,:]))
## # ax4 = Axis(f[2, 2])
## # lines!(ax4,value.(ph.t), value.(ph.u[1,:]))
## display(f)

