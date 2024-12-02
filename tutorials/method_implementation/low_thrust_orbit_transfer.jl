# #### Goddard's rocket Model

# State variables:

# Control variables

# Dynamics:
# $$g(x_h) = g_0 \cdot \left( \frac{x_h(0)}{x_h} \right)^2$$

# Outputs:

# Objective: 

# State Constraints:

# Input Constraints:

# Path Constrains:

# Integral Constraints:

# #### Start code
# Include the necessary packages. `JuMP` is required to setup various configurations
# while `Ipopt` is the solver to be used. Technically all other nonlinear solvers available
# throught JuMP can be used but those have not yet been tested.
include("../../src/DirectOptimalControl.jl")
import .DirectOptimalControl as DOC
# import DirectOptimalControl as DOC
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
OC.tol = 1e-2
OC.mesh_iter_max = 10
OC.objective_sense = "Max"

# Now it is time to select an optimizer. Let us select the Ipopt optimizer. The OC struct contains an `JuMP` model in its field
# `OC.model`. We can assign any non-linear optimizer that `JuMP` supports. For this reason we needed to import `JuMP`. It is advisable to
# initially keep the solver tolerance higher so that the optimizer converges. You can set all the solver specific options using the
# `JuMP` interface to aid the convergence of the solver.
# set_optimizer(OC.model, Ipopt.Optimizer)
# set_attribute(OC.model, "print_level", 0)
## set_attribute(OC.model, "max_iter", 500)
## set_attribute(OC.model, "tol", 1e-4)


# Each OCP must contain atleast one phase. The synatax for adding the phase to an OCP is given by
ph = DOC.Phase(OC)

# #### Define the models and cost functions
# Now let us define the parameters and functions which make up the model
T = 4.446618e-3 # [lb]
Isp = 450 # [s]
mu = 1.407645794e16 # [ftˆ3/sˆ2]
gs = 32.174 # [ft/sˆ2]
Re = 20925662.73 # [ft]
J2 = 1082.639e-6
J3 = -2.565e-6
J4 = -1.608e-6
p0 = 21837080.052835 # [ft]
f0 = 0
g0 = 0
h0 = -0.25396764647494
k0 = 0
L0 = pi
w0 = 1
tau = -25;
t0 = 0;
tf = 90000;

t0 = 0; tmin = t0; tfmin = 50000; tmax = 100000;
p0 = 21837080.052835; pf = 40007346.015232;
f0 = 0;
g0 = 0;
h0 = -0.25396764647494;
k0 = 0;
L0 = pi;
w0 = 1;

pmin = 20000000; pmax = 60000000;
fmin = -1; fmax = +1;
gmin = -1; gmax = +1;
hmin = -1; hmax = +1;
kmin = -1; kmax = +1;
Lmin = L0; Lmax = 9*2*pi;
wmin = 0.1; wmax = w0;
urmin = -1; urmax = +1;
utmin = -1; utmax = +1;
uhmin = -1; uhmax = +1;
taumin = -50; taumax = 0;

x0 = [p0, f0, g0, h0, k0, L0, w0]           # Initial state

# #### Auxillary parameters
# Now we create a named tuple of various parameters which will be necessary while defining the problem
# Note that in addition to the constants defined above we can also pass two additional parameters `PH.kp` and
# `OC.kg`. These are additional `JuMP` variables which can be optimized if required. They will not
# be used in this example.
p = (Isp = Isp, mu = mu, gs =gs, T = T, Re = Re, J2 = J2, J3 = J3, J4 = J4, x0 = x0)

ns = 7
nu = 3
n = 10

# #### System dynamics
# Note that the dyn function which defines the dynamics must be in a particular format.
# It must five inputs: 
# x : The state of system at time t
# u : The input of system at time t
# t : The time t
# p : p is a named tuple of auzillary parameters required to define the fucntion
function cross(a, b)
    return [a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]]
end

function dyn(x, u, t, p)
    # Extract auxiliary data for problem
    ap = p
    Isp = ap.Isp # [s]
    mu = ap.mu # [ftˆ3/sˆ2]
    gs = ap.gs # [ft/sˆ2]
    T = ap.T # [lb]
    Re = ap.Re # [ft]
    J2 = ap.J2
    J3 = ap.J3
    J4 = ap.J4

    # Extract state, control, and parameter for problem
    p = x[1]
    f = x[2]
    g = x[3]
    h = x[4]
    k = x[5]
    L = x[6]
    w = x[7]
    ur = u[1]
    ut = u[2]
    uh = u[3]
    tau = 1.0

    # Gravitational disturbing acceleration
    q = 1 + f * cos(L) + g * sin(L)
    r = p / q
    alpha2 = h * h - k * k
    chi = sqrt(h * h + k * k)
    s2 = 1 + chi * chi
    rX = (r / s2) * (cos(L) + alpha2 * cos(L) + 2 * h * k * sin(L))
    rY = (r / s2) * (sin(L) - alpha2 * sin(L) + 2 * h * k * cos(L))
    rZ = (2 * r / s2) * (h * sin(L) - k * cos(L))
    rVec = [rX, rY, rZ]
    rMag = sqrt(rX^2 + rY^2 + rZ^2)
    rXZMag = sqrt(rX^2 + rZ^2)
    vX = -(1 / s2) * sqrt(mu / p) * (sin(L) + alpha2 * sin(L) - 2 * h * k * cos(L) + g - 2 * f * h * k + alpha2 * g)
    vY = -(1 / s2) * sqrt(mu / p) * (-cos(L) + alpha2 * cos(L) + 2 * h * k * sin(L) - f + 2 * g * h * k + alpha2 * f)
    vZ = (2 / s2) * sqrt(mu / p) * (h * cos(L) + k * sin(L) + f * h + g * k)
    vVec = [vX, vY, vZ]
    rCrossv = cross(rVec, vVec)
    rCrossvMag = sqrt(rCrossv[1]^2 + rCrossv[2]^2 + rCrossv[3]^2)
    rCrossvCrossr = cross(rCrossv, rVec)
    ir1 = rVec[1] / rMag
    ir2 = rVec[2] / rMag
    ir3 = rVec[3] / rMag
    ir = [ir1, ir2, ir3]
    it1 = rCrossvCrossr[1] / (rCrossvMag * rMag)
    it2 = rCrossvCrossr[2] / (rCrossvMag * rMag)
    it3 = rCrossvCrossr[3] / (rCrossvMag * rMag)
    it = [it1, it2, it3]
    ih1 = rCrossv[1] / rCrossvMag
    ih2 = rCrossv[2] / rCrossvMag
    ih3 = rCrossv[3] / rCrossvMag
    ih = [ih1, ih2, ih3]
    enir = ir3
    enirir1 = enir * ir1
    enirir2 = enir * ir2
    enirir3 = enir * ir3
    enenirir1 = 0 - enirir1
    enenirir2 = 0 - enirir2
    enenirir3 = 1 - enirir3
    enenirirMag = sqrt(enenirir1^2 + enenirir2^2 + enenirir3^2)
    in1 = enenirir1 / enenirirMag
    in2 = enenirir2 / enenirirMag
    in3 = enenirir3 / enenirirMag
    # Geocentric latitude
    sinphi = rZ / rXZMag
    cosphi = sqrt(1 - sinphi^2)
    # Legendre polynomials
    P2 = (3 * sinphi^2 - 2) / 2
    P3 = (5 * sinphi^3 - 3 * sinphi) / 2
    P4 = (35 * sinphi^4 - 30 * sinphi^2 + 3) / 8
    dP2 = 3 * sinphi
    dP3 = (15 * sinphi - 3) / 2
    dP4 = (140 * sinphi^3 - 60 * sinphi) / 8
    # Oblate earth perturbations
    sumn = (Re / r)^2 * dP2 * J2 + (Re / r)^3 * dP3 * J3 + (Re / r)^4 * dP4 * J4
    sumr = (2 + 1) * (Re / r)^2 * P2 * J2 + (3 + 1) * (Re / r)^3 * P3 * J3 + (4 + 1) * (Re / r)^4 * P4 * J4
    ∆gn = -(mu * cosphi / (r^2)) * sumn
    ∆gr = -(mu / (r^2)) * sumr
    ∆gnin1 = ∆gn * in1
    ∆gnin2 = ∆gn * in2
    ∆gnin3 = ∆gn * in3
    ∆grir1 = ∆gr * ir1
    ∆grir2 = ∆gr * ir2
    ∆grir3 = ∆gr * ir3
    ∆g1 = ∆gnin1 - ∆grir1
    ∆g2 = ∆gnin2 - ∆grir2
    ∆g3 = ∆gnin3 - ∆grir3
    Deltag1 = ir[1] * ∆g1 + ir[2] * ∆g2 + ir[3] * ∆g3
    Deltag2 = it[1] * ∆g1 + it[2] * ∆g2 + it[3] * ∆g3
    Deltag3 = ih[1] * ∆g1 + ih[2] * ∆g2 + ih[3] * ∆g3
    # Thrust acceleration
    DeltaT1 = ((gs * T * (1 + 0.01 * tau)) / w) * ur
    DeltaT2 = ((gs * T * (1 + 0.01 * tau)) / w) * ut
    DeltaT3 = ((gs * T * (1 + 0.01 * tau)) / w) * uh
    # Total acceleration
    Delta1 = Deltag1 + DeltaT1
    Delta2 = Deltag2 + DeltaT2
    Delta3 = Deltag3 + DeltaT3
    # Differential equations of motion
    dp = (2 * p / q) * sqrt(p / mu) * Delta2
    df = sqrt(p / mu) * sin(L) * Delta1 + sqrt(p / mu) * (1 / q) * ((q + 1) * cos(L) + f) * Delta2 - sqrt(p / mu) * (g / q) * (h * sin(L) - k * cos(L)) * Delta3
    dg = -sqrt(p / mu) * cos(L) * Delta1 + sqrt(p / mu) * (1 / q) * ((q + 1) * sin(L) + g) * Delta2 + sqrt(p / mu) * (f / q) * (h * sin(L) - k * cos(L)) * Delta3
    dh = sqrt(p / mu) * (s2 * cos(L) / (2 * q)) * Delta3
    dk = sqrt(p / mu) * (s2 * sin(L) / (2 * q)) * Delta3
    dL = sqrt(p / mu) * (1 / q) * (h * sin(L) - k * cos(L)) * Delta3 + sqrt(mu * p) * ((q / p)^2)
    dw = -(T * (1 + 0.01 * tau) / Isp)

    return [dp, df, dg, dh, dk, dL, dw]
end

# function dyn(x, u, t, p)
#     # Extract auxiliary data for problem
#     ap = p
#     Isp = ap.Isp # [s]
#     mu = ap.mu # [ftˆ3/sˆ2]
#     gs = ap.gs # [ft/sˆ2]
#     T = ap.T # [lb]
#     Re = ap.Re # [ft]
#     J2 = ap.J2
#     J3 = ap.J3
#     J4 = ap.J4

#     # Extract state, control, and parameter for problem
#     p = x[1]
#     f = x[2]
#     g = x[3]
#     h = x[4]
#     k = x[5]
#     L = x[6]
#     w = x[7]
#     ur = u[1]
#     ut = u[2]
#     uh = u[3]
#     tau = ap.kp[1]

#     # Gravitational disturbing acceleration
#     q = 1 + f * cos(L) + g * sin(L)
#     r = p / q
#     alpha2 = h * h - k * k
#     chi = sqrt(h * h + k * k)
#     s2 = 1 + chi * chi
#     rX = (r / s2) * (cos(L) + alpha2 * cos(L) + 2 * h * k * sin(L))
#     rY = (r / s2) * (sin(L) - alpha2 * sin(L) + 2 * h * k * cos(L))
#     rZ = (2 * r / s2) * (h * sin(L) - k * cos(L))
#     rVec = [rX, rY, rZ]
#     rMag = sqrt(rX^2 + rY^2 + rZ^2)
#     rXZMag = sqrt(rX^2 + rZ^2)
#     vX = -(1 / s2) * sqrt(mu / p) * (sin(L) + alpha2 * sin(L) - 2 * h * k * cos(L) + g - 2 * f * h * k + alpha2 * g)
#     vY = -(1 / s2) * sqrt(mu / p) * (-cos(L) + alpha2 * cos(L) + 2 * h * k * sin(L) - f + 2 * g * h * k + alpha2 * f)
#     vZ = (2 / s2) * sqrt(mu / p) * (h * cos(L) + k * sin(L) + f * h + g * k)
#     vVec = [vX, vY, vZ]
#     rCrossv = cross(rVec, vVec)
#     rCrossvMag = sqrt(rCrossv[1]^2 + rCrossv[2]^2 + rCrossv[3]^2)
#     rCrossvCrossr = cross(rCrossv, rVec)
#     ir1 = rVec[1] / rMag
#     ir2 = rVec[2] / rMag
#     ir3 = rVec[3] / rMag
#     ir = [ir1, ir2, ir3]
#     it1 = rCrossvCrossr[1] / (rCrossvMag * rMag)
#     it2 = rCrossvCrossr[2] / (rCrossvMag * rMag)
#     it3 = rCrossvCrossr[3] / (rCrossvMag * rMag)
#     it = [it1, it2, it3]
#     ih1 = rCrossv[1] / rCrossvMag
#     ih2 = rCrossv[2] / rCrossvMag
#     ih3 = rCrossv[3] / rCrossvMag
#     ih = [ih1, ih2, ih3]
#     enir = ir3
#     enirir1 = enir * ir1
#     enirir2 = enir * ir2
#     enirir3 = enir * ir3
#     enenirir1 = 0 - enirir1
#     enenirir2 = 0 - enirir2
#     enenirir3 = 1 - enirir3
#     enenirirMag = sqrt(enenirir1^2 + enenirir2^2 + enenirir3^2)
#     in1 = enenirir1 / enenirirMag
#     in2 = enenirir2 / enenirirMag
#     in3 = enenirir3 / enenirirMag
#     # Geocentric latitude
#     sinphi = rZ / rXZMag
#     cosphi = sqrt(1 - sinphi^2)
#     # Legendre polynomials
#     P2 = (3 * sinphi^2 - 2) / 2
#     P3 = (5 * sinphi^3 - 3 * sinphi) / 2
#     P4 = (35 * sinphi^4 - 30 * sinphi^2 + 3) / 8
#     dP2 = 3 * sinphi
#     dP3 = (15 * sinphi - 3) / 2
#     dP4 = (140 * sinphi^3 - 60 * sinphi) / 8
#     # Oblate earth perturbations
#     sumn = (Re / r)^2 * dP2 * J2 + (Re / r)^3 * dP3 * J3 + (Re / r)^4 * dP4 * J4
#     sumr = (2 + 1) * (Re / r)^2 * P2 * J2 + (3 + 1) * (Re / r)^3 * P3 * J3 + (4 + 1) * (Re / r)^4 * P4 * J4
#     ∆gn = -(mu * cosphi / (r^2)) * sumn
#     ∆gr = -(mu / (r^2)) * sumr
#     ∆gnin1 = ∆gn * in1
#     ∆gnin2 = ∆gn * in2
#     ∆gnin3 = ∆gn * in3
#     ∆grir1 = ∆gr * ir1
#     ∆grir2 = ∆gr * ir2
#     ∆grir3 = ∆gr * ir3
#     ∆g1 = ∆gnin1 - ∆grir1
#     ∆g2 = ∆gnin2 - ∆grir2
#     ∆g3 = ∆gnin3 - ∆grir3
#     Deltag1 = ir[1] * ∆g1 + ir[2] * ∆g2 + ir[3] * ∆g3
#     Deltag2 = it[1] * ∆g1 + it[2] * ∆g2 + it[3] * ∆g3
#     Deltag3 = ih[1] * ∆g1 + ih[2] * ∆g2 + ih[3] * ∆g3
#     # Thrust acceleration
#     DeltaT1 = ((gs * T * (1 + 0.01 * tau)) / w) * ur
#     DeltaT2 = ((gs * T * (1 + 0.01 * tau)) / w) * ut
#     DeltaT3 = ((gs * T * (1 + 0.01 * tau)) / w) * uh
#     # Total acceleration
#     Delta1 = Deltag1 + DeltaT1
#     Delta2 = Deltag2 + DeltaT2
#     Delta3 = Deltag3 + DeltaT3
#     # Differential equations of motion
#     dp = (2 * p / q) * sqrt(p / mu) * Delta2
#     df = sqrt(p / mu) * sin(L) * Delta1 + sqrt(p / mu) * (1 / q) * ((q + 1) * cos(L) + f) * Delta2 - sqrt(p / mu) * (g / q) * (h * sin(L) - k * cos(L)) * Delta3
#     dg = -sqrt(p / mu) * cos(L) * Delta1 + sqrt(p / mu) * (1 / q) * ((q + 1) * sin(L) + g) * Delta2 + sqrt(p / mu) * (f / q) * (h * sin(L) - k * cos(L)) * Delta3
#     dh = sqrt(p / mu) * (s2 * cos(L) / (2 * q)) * Delta3
#     dk = sqrt(p / mu) * (s2 * sin(L) / (2 * q)) * Delta3
#     dL = sqrt(p / mu) * (1 / q) * (h * sin(L) - k * cos(L)) * Delta3 + sqrt(mu * p) * ((q / p)^2)
#     dw = -(T * (1 + 0.01 * tau) / Isp)

#     return [dp, df, dg, dh, dk, dL, dw]
# end
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
    wf = xf[7]
    return wf
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
    ur = u[1]
    ut = u[2]
    uh = u[3]
    return ur^2+ut^2+uh^2-1
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
ph.np = 1   # Dimension of the path constraints
ph.p = p    # Auxillary parametrs named tuple

ph.nk = 1                               # Number of auxillary phase parameters to be optimized
OC.nkg = 0                              # Number of auxillary global parameters to be optimized

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
ph.limits.ll.u =  [urmin,utmin,uhmin]      # Lower bounds on input. Vector of dimension `nu`
ph.limits.ul.u =   [urmax,utmax,uhmax] # Upper bounds on input. Vector of dimesion `nu`
ph.limits.ll.x = [pmin, fmin, gmin, hmin, kmin, Lmin, wmin] # Lower bounds on state trajectory. Vector of dimension `ns`   
ph.limits.ul.x = [pmax, fmax, gmax, hmax, kmax, Lmax, wmax] # Upper bounds on state trajectory. Vector of dimesion `ns`
ph.limits.ll.xf =   [pf,fmin,gmin,hmin,kmin,Lmin,wmin]     # Lower bounds on final state. Vector of dimension `nu`
ph.limits.ul.xf =   [pf,fmax,gmax,hmax,kmax,Lmax,wmax]   # Upper bounds on input. Vector of dimesion `ns`
ph.limits.ll.xi = p.x0  # Lower bounds on initial state. Vector of dimension `ns`
ph.limits.ul.xi = p.x0  # Upper bounds on initial state. Vector of dimesion `ns`
ph.limits.ll.ti = t0   # Lower bounds on initial time. A Scalar.
ph.limits.ul.ti = t0   # Upper bounds on initial time. A Scalar.
ph.limits.ll.tf = tfmin   # Lower bounds on final time. A scalar.
ph.limits.ul.tf = tmax   # Upper bounds on final time. A scalar.
ph.limits.ll.dt = tfmin     # Lower bounds on time interval. A scalar.
ph.limits.ul.dt = tmax     # Upper bounds on time interval. A scalar 
ph.limits.ll.path = [0.0] # Lower bounds on path constraint. Vector of dimension `nu`
ph.limits.ul.path = [0.0] # Upper bounds on  path constraint. Vector of dimesion `ns`
ph.limits.ll.integral = [] # Lower bounds on integral constraint. Vector of dimension `nu`
ph.limits.ul.integral = [] # Upper bounds on integral contsraint. Vector of dimesion `ns`
ph.limits.ll.k = [taumin] # Lower bounds on phase parameters. Vector of dimension `nu`
ph.limits.ul.k = [taumax] # Upper bounds on phase parameters. Vector of dimesion `ns`

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
OC.npsi = 4    
OC.psi_llim = [0.73550320568829^2,0.61761258786099^2,0,-3]
OC.psi_ulim = [0.73550320568829^2,0.61761258786099^2,0,0];
# Not the format of he event function `psi`. It takes the OCP object as input.
# The OCP object has all the phases stored in it in the field `OC.ph`.
# Each phase has state variables which can be accessed by `ph.x` and input
# variable which can be accessed by `ph.u`. In this problem we want that
# `u` must have a zero value at end of the phase.
function psi(ocp::DOC.OCP)
    (;ph) = ocp
    xf = ph[1].xf

    ff = xf[2]
    gf = xf[3]
    hf = xf[4]
    kf = xf[5]

    return [ff^2+gf^2, hf^2+kf^2, ff*hf+gf*kf, gf*hf-kf*ff]
end
OC.psi = psi

# # Callback function can be used to log variables
# # Set it to return nothing if you do not want to log variables in mesh iterations
# ph.callback_nt = (tau_hist = Vector{Float64}[],err_hist = Vector{Float64}[])
# function callback_fun(ph::DOC.PH)
#     push!(ph.callback_nt.tau_hist, deepcopy(ph.tau))
#     push!(ph.callback_nt.err_hist, deepcopy(ph.error))
# end
# ph.callback_fun = callback_fun


# Call function to setup the JuMP model for solving optimal control problem
DOC.setup_mpocp(OC)
# Solve for the control and state
DOC.solve_mpocp(OC)
# DOC.solve(OC)
solution_summary(OC.model)

# Display results
println("Objective Value: ", objective_value(OC.model))

using GLMakie
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
