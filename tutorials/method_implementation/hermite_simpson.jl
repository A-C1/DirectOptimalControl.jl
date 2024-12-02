using JuMP
import Ipopt
using GLMakie

dubins = Model(Ipopt.Optimizer)
vm = 1.0   # Max forward speed
um = 1.0   # Max turning speed
ns = 3     # Number of states
nu = 1     # Number of inputs
n = 200    # Time steps

# System dynamics
function f(x, u)
    return [vm*cos(x[3]), vm*sin(x[3]), u[1]]
end

# Objective Function
function obj_f(x, u, Δt)
    return Δt
end

# Decision variables
@variables(dubins, begin
    0 <= Δt <= 0.05  # Time step
    # State variables
    x[1:3,1:n]               # Velocity
    # Control variables
    -um ≤ u[1:1,1:n] ≤ um    # Thrust
end)

# Objective
@objective(dubins, Min, obj_f(x[1:3,1:n], u[1,1:n], Δt))
#Initial conditions
@constraint(dubins, x[1:3, 1] .== [0, 0, -π/2 ] )
# Final conditions
@constraint(dubins, x[1:3, n] == [5.0, 5.0, π/2 + π/4])

# Dynamic constraints: hermite_simpson
for j in 1:n-1
    ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

    f1 = f(x[1:ns, j], u[1:nu, j])
    f2 = f(x[1:ns, j+1], u[1:nu, j+1])
    xb = (x[1:ns, j] + x[1:ns, j+1]) / 2 + Δt * (f1 - f2) / 8
    fb = f(xb, ub)

    @constraint(dubins, x[1:ns, j+1] == x[1:ns, j] + Δt * (f1 + 4 * fb + f2) / 6)
end

# Solve for the control and state
println("Solving...")
JuMP.optimize!(dubins)
solution_summary(dubins)

# Display results
println("Min time: ", objective_value(dubins)*n)

fig = Figure()
ax = Axis(fig[1,1], autolimitaspect = 1)
lines!(ax, value.(x[1,1:n]), value.(x[2, 1:n]))
fig