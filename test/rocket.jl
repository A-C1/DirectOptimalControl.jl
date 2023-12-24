import DirectOptimalControl as DOC
using JuMP
import Ipopt
using GLMakie

# Set solver configuration
OC = DOC.OCP()
OC.tol = 1e-7
OC.mesh_iter_max = 10
OC.objective_sense = "Max"
set_optimizer(OC.model, Ipopt.Optimizer)
set_attribute(OC.model, "print_level", 0)
# set_attribute(OC.model, "max_iter", 500)
# set_attribute(OC.model, "tol", 1e-4)


# Define the models and cost functions
h0 = 1                      # Initial height
v0 = 0                      # Initial velocity
m0 = 1.0                    # Initial mass
mT = 0.6                    # Final mass
g0 = 1                      # Gravity at the surface
hc = 500                    # Used for drag
c = 0.5 * sqrt(g0 * h0)    # Thrust-to-fuel mass
Dc = 0.5 * 620 * m0 / g0  # Drag scaling
utmax = 3.5 * g0 * m0    # Maximum thrust
Tmax = 0.2                  # Number of seconds

x0 = [h0, v0, m0]

p = (g0 = g0, hc = hc, c = c, Dc = Dc, xh0 = h0, utmax = utmax, x0 = x0)

ns = 3
nu = 1
n = 50

# System dynamics
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

# Objective Function
# Running cost
function L(x, u, t, p)
    return 0.0
end

function phi(xf, uf, tf, p)
    return xf[1]
end



#------------------------------------------------------
# Phase 1
ph = DOC.PH(OC)
ph.L = L
ph.phi = phi
ph.dyn = dyn
ph.integralfun = integralfun
ph.collocation_method = "hermite-simpson"
ph.scale_flag = false

ph.n = n
ph.ns = ns
ph.nu = nu
ph.nq = 0
# ph.nk = 3

# Auxillary parameters
ph.p = p

ph.limits.ll.u = [0.0]
ph.limits.ul.u = [p.utmax]
ph.limits.ll.x = [0.3, 0.0, mT]
ph.limits.ul.x = [2.0, 2.0, 2.0]
ph.limits.ll.xf = [0.3, 0, mT]
ph.limits.ul.xf = [2.0, 2.0, 2.0]
ph.limits.ll.xi = p.x0
ph.limits.ul.xi = p.x0
ph.limits.ll.ti = 0.0
ph.limits.ul.ti = 0.0
ph.limits.ll.tf = 0.2
ph.limits.ul.tf = 0.2
ph.limits.ll.dt = 0.0
ph.limits.ul.dt = 0.2
# ph.limits.ll.k = [0.0, 0.0, 0.0]
# ph.limits.ul.k = [0.2, 0.2, 0.2]
# Add the boundary constraints

# Set intial values
ph.set_initial_vals = "Auto"
ph.tau = range(start = 0, stop = 1, length = ph.n)
ph.xinit = ones(ph.ns, ph.n)
ph.uinit = ones(ph.nu, ph.n)
ph.tfinit = ph.limits.ll.tf
ph.tiinit = ph.limits.ll.ti
# ph.kinit  = (ph.limits.ll.k + ph.limits.ul.k)/2

# Specify initial value

OC.obj_llim = -2.0
OC.obj_ulim = 2.0
OC.psi_llim = [0.0]
OC.psi_ulim = [0.0]
# OC.kg_llim = [0.0, 0.0, 0.0]
# OC.kg_ulim = [1.0, 1.0, 1.0]


# OC.nkg = 2     #  Number of global parameters  Optional parameter
OC.npsi = 1    # Optional parameter evnts
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    v1 = ph[1].u[:, end]

    return [v1;]
    # return nothing
end

OC.psi = psi


# Call function to setup the JuMP model for solving optimal control problem
DOC.setup_mpocp(OC)
# Solve for the control and state
# DOC.solve_mpocp(OC)
DOC.solve(OC)
solution_summary(OC.model)

# Display results
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

