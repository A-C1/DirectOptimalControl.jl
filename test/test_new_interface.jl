include("../src/DirectOptimalControl.jl")
import .DirectOptimalControl as DOC


# Define dimensions of optimal control problem
d = DOC.Dims()
d.n = 50
d.ns = 3
d.nu = 1
d.nq = 1
d.np = 1
d.nk = 0


# Define functions for optimal control problem
p = (v = 1.0, umax = 1.0)

function dyn(x, u, t, p)
    xn = p.v*cos(x[3])
    yn = p.v*sin(x[3])
    thetan = u[1]

    return [xn, yn, thetan]
end

function L(x, u, t, p)
    return 1.0
end

function phi(xn, un, t, p)
    return 0.0
end

function integral_fun(x, u, t, p)
    return [1.0]
end

function path_fun(x, u, t, p) 
    return [(x[1] - 3)^2 + (x[2] - 3)^2 - 9]
end

funcs = DOC.Funs(L, phi, dyn, path_fun, integral_fun)

# Define limits
limits = DOC.Limits()
limits.ll.ti = 0.0
limits.ul.ti = 0.0
limits.ll.dt = 0.0 
limits.ul.dt = 40.0
limits.ll.tf = 0.0
limits.ul.tf = 40.0
limits.ll.xf = [10.0, 10.0, 0.0]
limits.ul.xf = [10.0, 10.0, 0.0]
limits.ll.xi = [0.0, 0.0, -3.14]
limits.ul.xi = [0.0, 0.0, -3.14]
limits.ll.x = [-20.0, -20.0, -3.5]
limits.ul.x = [20.0, 20.0, 3,5]
limits.ll.u = [-1.0]
limits.ul.u = [1.0]
limits.ll.integral = [30.0]
limits.ul.integral = [40.0]
limits.ll.path = [0.0]
limits.ul.path = [150.0]

cp = DOC.ConfigParams("trapezoidal", true, true)
ph = DOC.Phase(ocp, p, funcs, d, limits, cp)

# Test all the functions
# DOC.add_var_constraints(ph.uvars, ph.dims, ph.limits, ocp.model)
# DOC.add_quadrature_constraints(ph, ocp.model)
# DOC.add_path_constraints(ph, ocp.model)
# DOC.trapezoidal(ph, d, ocp.model)

DOC.add_phase(ph, ocp)
DOC.add_objective(ocp)

DOC.optimize!(ocp.model)

using GLMakie
f = Figure()
ax = Axis(f[1,1], autolimitaspect = 1)
lines!(ax, DOC.value.(ph.uvars.x[1,:]), DOC.value.(ph.uvars.x[2,:]), color = :blue)
arc!(ax, Point2f(3.0, 3.0), 3.0, 0.0, 2π; color =:red)



 
