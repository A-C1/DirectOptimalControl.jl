"""
hermite_simpson(ph, model)
Add hermite_simpson collocation constraints to model of a particular phase. 
This is a fourth order implicit method.
"""
# function hermite_simpson(ph::PH, model::Model)
#     (;dyn, n, ns, nu, p, x, u, t, h) = ph
#     for j in 1:n-1
#         ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

#         # @infiltrate

#         f1 = dyn(x[1:ns, j], u[1:nu, j], t[j], p)
#         f2 = dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p)
#         xb = (x[1:ns, j] + x[1:ns, j+1])/2 + h[j]*(f1 - f2)/8
#         tb = t[j] + h[j]/2 
#         fb = dyn(xb, ub, tb, p)

#         dx = h[j] * (f1 + 4*fb + f2) / 6

#         ph.collocation_constraints[j] = @constraint(model, x[1:ns, j+1] .== x[1:ns, j] + dx)
#     end
# end

function hermite_simpson(ph::Phase, model::Model)
    (;dyn, n, ns, nu, p, x, u, t, h) = ph
    ph.f1 = @variable(model, [1:ns, 1:n-1])
    ph.f2 = @variable(model, [1:ns, 1:n-1])
    ph.xb = @variable(model, [1:ns, 1:n-1])
    ph.fb = @variable(model, [1:ns, 1:n-1])
    ph.dx = @variable(model, [1:ns, 1:n-1])
    for j in 1:n-1
        ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

        @constraint(model, ph.f1[1:ns, j] .== dyn(x[1:ns, j], u[1:nu, j], t[j], p))
        @constraint(model, ph.f2[1:ns, j] .== dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p))
        @constraint(model, ph.xb[1:ns,j] .== (x[1:ns, j] + x[1:ns, j+1])/2 + h[j]*(ph.f1[1:ns,j] - ph.f2[1:ns,j])/8)
        tb = t[j] + h[j]/2 
        @constraint(model, ph.fb[1:ns,j] .== dyn(ph.xb[1:ns,j], ub, tb, p))

        @constraint(model, ph.dx[1:ns, j] .== h[j] * (ph.f1[1:ns, j] + 4*ph.fb[1:ns, j] + ph.f2[1:ns,j])/6)

        ph.collocation_constraints[j] = @constraint(model, x[1:ns, j+1] .== x[1:ns, j] + ph.dx[1:ns, j])
    end
end


"""
    trapezoidal(ph, model)

Adds trapezoidal collocation to model.
"""
# function trapezoidal(ph::PH, model::Model)
#     (;dyn, n, ns, nu, p, x, u, t, h) = ph
#     for j in 1:n-1
#         f1 = dyn(x[1:ns, j], u[1:nu, j], t[j], p)
#         f2 = dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p)

#         ph.collocation_constraints[j] = @constraint(model, x[1:ns, j+1] .== x[1:ns, j] + h[j] * (f1 + f2) / 2)
#     end
# end

function trapezoidal(ph::Phase, model::Model)
    (;dyn, n, ns, nu, p, x, u, t, h) = ph
    ph.f1 = @variable(model, [1:ns, 1:n-1])
    ph.f2 = @variable(model, [1:ns, 1:n-1])
    for j in 1:n-1
        @constraint(model, ph.f1[1:ns, j] == dyn(x[1:ns, j], u[1:nu, j], t[j], p))
        @constraint(model, ph.f2[1:ns, j] == dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p))

        ph.collocation_constraints[j] = @constraint(model, x[1:ns, j+1] .== x[1:ns, j] + h[j] * (ph.f1[1:ns, j] + ph.f2[1:ns, j]) / 2)
    end
end


"""
euler(ph, Model)
"""
function euler(ph::Phase, model::Model)
    (;dyn, n, ns, nu, p, x, u, t, h) = ph
    for j in 1:n-1
        f1 = dyn(x[1:ns, j], u[1:nu, j], t[j], p)

        @constraint(model, x[1:ns, j+1] == x[1:ns, j] + h[j] * f1)
    end
end