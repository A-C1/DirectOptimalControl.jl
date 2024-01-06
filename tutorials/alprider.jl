## include("../src/DirectOptimalControl.jl")
## import .DirectOptimalControl as DOC

import DirectOptimalControl as DOC
import Ipopt
using JuMP

OC = DOC.OCP()
OC.tol = 1e-5
OC.mesh_iter_max = 10
OC.objective_sense = "Min"
set_optimizer(OC.model, Ipopt.Optimizer)
set_attribute(OC.model, "print_level", 0)
set_attribute(OC.model, "max_iter", 500)
set_attribute(OC.model, "tol", 1e-6)


x0 = [2, 1, 2, 1]
xf = [2, 3, 1, -2]
t0 = 0
tf = 20

p = (x0 = x0, xf = xf, t0 = t0, tf = tf)

ns = 4
nu = 2
n = 500
np = 1

# System dynamics
pf(t, a, b) = exp(-b*(t-a)^2)
function dyn(x, u, t, p)
    x1 = -10x[1] + u[1] + u[2]
    x2 = -2x[2] + u[1] + 2u[2]
    x3 = -3x[3] + 5x[4] + u[1] - u[2]
    x4 = 5x[3] - 3x[4] + u[1] + 3u[2]

    return [x1, x2, x3, x4]
end

# Objective Function
# Running cost
function L(x, u, t, p)
    return 100*(x[1]^2 + x[2]^2 + x[3]^2 + x[4]^2) + 0.01(u[1]^2 + u[2]^2)
end

function phi(xf, uf, tf, p)
    return 0.0
end

function integralfun(x, u, t, p)
    return nothing
end

# Path function
function pathfun(x, u, t, p)
    v1 = x[1]^2 + x[2]^2 + x[3]^2 + x[4]^2 - (3*pf(t, 3.0, 12.0) + 3*pf(t, 6.0, 10.0) + 3*pf(t, 10.0, 6.0) + 8*pf(t, 15.0, 4.0) + 0.01 )
    return [v1]
end



# Phase 1
ph = DOC.PH(OC)
ph.L = L
ph.phi = phi
ph.dyn = dyn
ph.integralfun = integralfun
ph.pathfun = pathfun
ph.collocation_method = "hermite-simpson"
ph.scale_flag = true

ph.n = n
ph.ns = ns
ph.nu = nu
ph.np = np
ph.nq = 0
ph.p = p

ph.limits.ll.u = -1e3*[1.0, 1.0]
ph.limits.ul.u = 1e3*[1.0, 1.0]
ph.limits.ll.x = -1e1*[1.0, 1.0, 1.0, 1.0]
ph.limits.ul.x = 1e1*[1.0, 1.0, 1.0, 1.0]
ph.limits.ll.xf = xf
ph.limits.ul.xf = xf
ph.limits.ll.xi = x0
ph.limits.ul.xi = x0
ph.limits.ll.ti = t0
ph.limits.ul.ti = t0
ph.limits.ll.tf = tf
ph.limits.ul.tf = tf
ph.limits.ll.dt = tf-t0
ph.limits.ul.dt = tf-t0
ph.limits.ll.path = [0]
ph.limits.ul.path = [10000]

ph.set_initial_vals = "Auto"

# Add the boundary constraints
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    return nothing
end

OC.psi = psi
OC.psi_llim = []
OC.psi_ulim = []
OC.set_obj_lim = false
OC.obj_llim = 1e6
OC.obj_ulim = -1e6

# Callback function can be used to log variables
# Set it to return nothing if you do not want to log variables in mesh iterations
ph.callback_nt = (tau_hist = Vector{Float64}[],err_hist = Vector{Float64}[])
function callback_fun(ph::DOC.PH)
    push!(ph.callback_nt.tau_hist, deepcopy(ph.tau))
    push!(ph.callback_nt.err_hist, deepcopy(ph.error))
end
ph.callback_fun = callback_fun

DOC.setup_mpocp(OC)
DOC.solve_mpocp(OC)
## DOC.solve(OC)

solution_summary(OC.model)

# Display results
println("Objective Value: ", objective_value(OC.model))


## using GLMakie
## f1, ax1, l1 = lines(value.(ph.t), value.(ph.x[1,:]))
## f2, ax2, l2 = lines(value.(ph.t), value.(ph.x[2,:]))
## f3, ax3, l3 = lines(value.(ph.t), value.(ph.x[3,:]))
## f4, ax4, l4 = lines(value.(ph.t), value.(ph.x[4,:]))
## fu1 = lines(value.(ph.t), value.(ph.u[1,:]))
## fu2 = lines(value.(ph.t), value.(ph.u[2,:]))

