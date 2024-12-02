include("../src/DirectOptimalControl.jl")

import .DirectOptimalControl as DOC

# Create OCP
ocp = DOC.OCP()

# Define dimensions of optimal control problem
d = DOC.Dims()
d.n = 100
d.ns = 3
d.nu = 1
d.nq = 0
d.np = 0
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
    return nothing
end

function path_fun(x, u, t, p)
    return nothing
end

funcs = DOC.Funs(L, phi, dyn, path_fun, integral_fun)

# Define limits
limits = DOC.Limits()
limits.ll.ti = 0.0
limits.ul.ti = 0.0
limits.ll.dt = 0.0 
limits.ul.dt = 10.0
limits.ll.tf = 0.0
limits.ul.tf = 10.0
limits.ll.xf = [0.0, 0.0, 0.0]
limits.ul.xf = [0.0, 0.0, 0.0]
limits.ll.xi = [5.0, 5.0, 0.0]
limits.ul.xi = [5.0, 5.0, 0.0]
limits.ll.x = [-10.0, -10.0, -10.0]
limits.ul.x = [10.0, 10.0, 10.0]
limits.ll.u = [-1.0]
limits.ul.u = [1.0]
limits.ll.integral = []
limits.ul.integral = []
limits.ll.path = []
limits.ul.path = []
limits.ll.path = []
limits.ul.path = []

cp = DOC.ConfigParams("trapezoidal", true, true)

ph = DOC.Phase(ocp, p, funcs, d, limits, cp)

# Test all the functions
DOC.add_var_constraints(ph.uvars, ph.dims, ph.limits, ocp.model)
DOC.add_quadrature_constraints(ph, ocp.model)
DOC.add_path_constraints(ph, ocp.model)


println("TryNow...")



 
