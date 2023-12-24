"""
    RK4(ph, model)
    Adds the collocaion constraints to the model using RK4 method
"""
function rk4(ph::PH, model::Model)
    (;dyn, n, ns, nu, p, x, u, t, h) = ph
    for j in 1:n-1
        ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

        k1 = dyn(x[1:ns, j], u[1:nu,j], t[j], p)
        k2 = dyn(x[1:ns, j] + h[j] * k1 / 2, ub, t[j], p)
        k3 = dyn(x[1:ns, j] + h[j] * k2 / 2, ub, t[j], p)
        k4 = dyn(x[1:ns, j] + h[j] * k3, u[1:nu,j+1], t[j], p)

        dx = h[j] * (k1 + 2 * k2 + 2 * k3 + k4) / 6

        @constraint(model, x[1:ns, j+1] == x[1:ns, j] + dx)
    end
end

"""
hermite_simpson(ph, model)
Add hermite_simpson collocation constraints to model of a particular phase. 
This is a fourth order implicit method. Preferred to be used.
"""
function hermite_simpson(ph::PH, model::Model)
    (;dyn, n, ns, nu, p, x, u, t, h) = ph
    for j in 1:n-1
        ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

        # @infiltrate

        f1 = dyn(x[1:ns, j], u[1:nu, j], t[j], p)
        f2 = dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p)
        xb = (x[1:ns, j] + x[1:ns, j+1])/2 + h[j]*(f1 - f2)/8
        tb = t[j] + h[j]/2 
        fb = dyn(xb, ub, tb, p)

        dx = h[j] * (f1 + 4*fb + f2) / 6

        ph.collocation_constraints[j] = @constraint(model, x[1:ns, j+1] .== x[1:ns, j] + dx)
    end
end

"""
trapezoidal(ph, Model)
"""
function trapezoidal(ph::PH, model::Model)
    (;dyn, n, ns, nu, p, x, u, t, h) = ph
    for j in 1:n-1
        f1 = dyn(x[1:ns, j], u[1:nu, j], t[j], p)
        f2 = dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p)

        @constraint(model, x[1:ns, j+1] == x[1:ns, j] + h[j] * (f1 + f2) / 2)
    end
end

"""
euler(ph, Model)
"""
function euler(ph::PH, model::Model)
    (;dyn, n, ns, nu, p, x, u, t, h) = ph
    for j in 1:n-1
        f1 = dyn(x[1:ns, j], u[1:nu, j], t[j], p)

        @constraint(model, x[1:ns, j+1] == x[1:ns, j] + h[j] * f1)
    end
end