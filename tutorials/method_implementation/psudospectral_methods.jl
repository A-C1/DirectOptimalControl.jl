using JuMP
using Ipopt
using FastGaussQuadrature
using Polynomials
using GLMakie

# d: Number of points for interpolation. Degree of interpolating polynomial = d-1
d = 4
x, w = gausslegendre(d-1)

# Collocation points
tau_root = [0; (1.0 .+ x) ./ 2]

# Coefficients of the collocation equation
C = zeros(d, d)

# Coefficients of continuity equation
D = zeros(d)

# Coefficients of quadrature function
B = zeros(d)

# Construct polynomial basis
Lj::Polynomial = Polynomial([1])
for j in 1:d
    # Construct lagrange polynomials to get the polynomial basis at the collocation point
    global tau_root, C, D, B, Lj
    Lj = Polynomial([1])
    for r in 1:d
        if r != j
            Lj = Lj*Polynomial([-tau_root[r], 1]) / (tau_root[j] - tau_root[r])
        end
    end

    # Evaluate the polynomial at the end of interval
    D[j] = Lj(1.0)

    # Evaluate the time derivatives at the collocation point
    Ljd = derivative(Lj)
    for r = 1:d
        C[j, r] = Ljd(tau_root[r])
    end

    # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
    LjI = integrate(Lj)
    B[j] = LjI(1.0)
end

# Time Horizon
T = 10
N = 1000
h = T/N
time = range(start = 0, step = 0.5, length = N)

ns = 2
nu = 1

# Declare Model variables
dc = Model(Ipopt.Optimizer)
@variable(dc, x[1:ns, 1:d, 1:N], start = 0)
@variable(dc, -1 <= u[1:nu, 1:N] <= 1, start = 0)

# State constraints
@constraint(dc, x[1, 1:d, 1:N] .>= -0.25)

# Initial state
@constraint(dc, x[1:ns, 1, 1] .== [0, 1])

# System model
dyn(x, u) = [((1 - x[2]^2)*x[1] - x[2] + u[1]), x[1]]

# Objective function
L(x, u) = x[1]^2 + x[2]^2 + u[1]^2

pdot = zeros(ns)
J = 0
Xend = zeros(ns)

for i in 1:N-1 
    global pdot, J, Xend
    Xend = x[1:ns, 1, i]*D[1]
    for r = 2:d
        pdot = zeros(ns)
        for j in 1:d 
            pdot = pdot + C[j, r] * x[1:ns, j, i]
        end
        @constraint(dc, pdot .== h*dyn(x[1:ns, r, i], u[1:nu, i]))
        J = J + h*B[r]*L(x[1:ns, r, i], u[1:nu, i])
        Xend = Xend + x[1:ns, r, i]*D[r]
    end
    @constraint(dc, Xend .== x[1:ns, 1, i+1])
end

@objective(dc, Min, J)

optimize!(dc)

un = u[1:nu, :]
xn = x[1:ns,1,:]

f1, ax1, s1 = stairs(time, value.(un)[1, :], color = :blue, label = "u")
lines!(ax1, time, value.(xn)[1, :], color = :black, label = "x1")
lines!(ax1, time, value.(xn)[2, :], color = :red, label = "x2")
axislegend()
    