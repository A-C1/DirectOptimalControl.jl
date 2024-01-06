## include("../src/DirectOptimalControl.jl")
## import .DirectOptimalControl as DOC

import .DirectOptimalControl as DOC
import Ipopt
using JuMP


OC = DOC.OCP()
OC.tol = 1e-7
OC.mesh_iter_max = 30
OC.objective_sense = "Min"
set_optimizer(OC.model, Ipopt.Optimizer)
## set_attribute(OC.model, "max_iter", 500)
## set_attribute(OC.model, "tol", 1e-3)
set_attribute(OC.model, "print_level", 0)


t0 = 0;
tf = 10000;
x0 = 1.5;
xf = 1;
xMin = -50;
xMax = +50;
uMin = -50;
uMax = +50;

ns = 1
nu = 1
n = 10

# System dynamics
# Function must always return a vector``
function dyn(x, u, t, p)
    return [-x[1]^3 + u[1]]
end

# Objective Function
# Running cost
function L(x, u, t, p)
    return 0.5*(x[1]^2 + u[1]^2)
end

function phi(xf, uf, tf, p)
    return xf[1]^2 + uf[1]^2
end

integralfun(x, u, t, p) = nothing



# Phase 1
ph = DOC.PH(OC)
ph.L = L
ph.phi = phi
ph.dyn = dyn
ph.integralfun = integralfun

ph.n = n
ph.ns = ns
ph.nu = nu
ph.p = (k1 = 1, k2 = 2)

ph.limits.ll.u = [uMin]
ph.limits.ul.u = [uMax]
ph.limits.ll.x =[xMin]
ph.limits.ul.x = [xMax]
ph.limits.ll.xf = [xf]
ph.limits.ul.xf = [xf]
ph.limits.ll.xi = [x0]
ph.limits.ul.xi = [x0]
ph.limits.ll.ti = t0
ph.limits.ul.ti = t0
ph.limits.ll.tf = tf
ph.limits.ul.tf = tf
ph.limits.ll.dt = tf-t0
ph.limits.ul.dt = tf-t0

ph.collocation_method = "hermite-simpson"
ph.set_initial_vals = "Auto"
ph.scale_flag = true

# Add the boundary constraints
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    return nothing
end

OC.psi = psi
DOC.setup_mpocp(OC)
## DOC.solve_mpocp(OC)
DOC.solve(OC)

# Solve for the control and state
solution_summary(OC.model)

# Display results
println("Min value: ", objective_value(OC.model))

## using GLMakie
## f1, ax1, l1 = lines(value.(ph.t), value.(ph.x[1,:]))
## f2, ax2, l2 = lines(value.(ph.t), value.(ph.u[1,:]))

