include("../src/DirectOptimalControl.jl")
import .DirectOptimalControl as NOC

import Ipopt
using GLMakie
using JuMP


OC = NOC.OCP()
OC.tol = 1e-4
OC.mesh_iter_max = 5
OC.min = false
set_optimizer(OC.model, Ipopt.Optimizer)
# set_attribute(OC.model, "max_iter", 500)
# set_attribute(OC.model, "tol", 1e-4)


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
n = 1000

# System dynamics
D(xh, xv, p) = p.Dc*(xv^2)*exp(-p.hc*(xh - p.xh0)/p.xh0)
g(xh, p) = p.g0*(p.xh0/xh)^2
function dyn(x, u, t, p)
    h = x[1]
    phi = x[2]
    h1 = 
    phi2 = 
    theta3 = 
    v4 = 
    gamma5 =
    psi6 =

    return [h1, phi2, theta3, v4, gamma4, psi6]
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
ph1 = NOC.PH(OC)
ph1.L = L
ph1.phi = phi
ph1.dyn = dyn

ph1.n = n
ph1.tau = range(start = 0, stop = 1, length = ph1.n)
ph1.ns = ns
ph1.nu = nu
ph1.p = p

ph1.limits.ll.u = [0.0]
ph1.limits.ul.u = [p.utmax]
ph1.limits.ll.x = [0.3, 0.0, mT]
ph1.limits.ul.x = [2.0, 2.0, 2.0]
ph1.limits.ll.xf = [0.3, 0, mT]
ph1.limits.ul.xf = [2.0, 2.0, 2.0]
ph1.limits.ll.xi = p.x0
ph1.limits.ul.xi = p.x0
ph1.limits.ll.ti = 0.0
ph1.limits.ul.ti = 0.0
ph1.limits.ll.tf = 0.2
ph1.limits.ul.tf = 0.2
ph1.limits.ll.dt = 0.0
ph1.limits.ul.dt = 0.2

# Add the boundary constraints
function psi(ocp::NOC.OCP)
    (;ph) = ocp

    # Phase 1
    v1 = ph[1].ti - 0.0
    v2 = ph[1].u[:, end]
    v3 = ph[1].xi - ph[1].p.x0
    v4 = ph[1].tf - 0.2


    return [v2;]
    # return nothing
end

OC.psi = psi
# c = NOC.add_phase(ph1, OC)
NOC.solve(OC)
# Solve for the control and state
solution_summary(OC.model)

# Display results
println("Min time: ", objective_value(OC.model))

# x, u, dt, oc = NOC.solve(OC)

f1, ax1, l1 = lines(value.(ph1.t), value.(ph1.x[1,:]))
f2, ax2, l2 = lines(value.(ph1.t), value.(ph1.x[2,:]))
f3, ax3, l3 = lines(value.(ph1.t), value.(ph1.x[3,:]))
f4, ax4, l4 = lines(value.(ph1.t), value.(ph1.u[1,:]))
display(f1)
display(f2)
display(f3)

