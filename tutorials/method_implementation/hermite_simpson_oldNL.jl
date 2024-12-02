using JuMP
import Ipopt
using Infiltrator

dubins = Model(Ipopt.Optimizer)

# #### Define the models and cost functions
# Now let us define the parameters and functions which make up the model
T = 4.446618e-3 # [lb]
Isp = 450 # [s]
mu = 1.407645794e16 # [ftˆ3/sˆ2]
gs = 32.174 # [ft/sˆ2]
Re = 20925662.73 # [ft]
J2 = 1082.639e-6
J3 = -2.565e-6
J4 = -1.608e-6
p0 = 21837080.052835 # [ft]
f0 = 0
g0 = 0
h0 = -0.25396764647494
k0 = 0
L0 = pi
w0 = 1
tau = -25;
t0 = 0;
tf = 90000;

t0 = 0; tmin = t0; tfmin = 50000; tmax = 100000;
p0 = 21837080.052835; pf = 40007346.015232;
f0 = 0;
g0 = 0;
h0 = -0.25396764647494;
k0 = 0;
L0 = pi;
w0 = 1;

pmin = 20000000; pmax = 60000000;
fmin = -1; fmax = +1;
gmin = -1; gmax = +1;
hmin = -1; hmax = +1;
kmin = -1; kmax = +1;
Lmin = L0; Lmax = 9*2*pi;
wmin = 0.1; wmax = w0;
urmin = -1; urmax = +1;
utmin = -1; utmax = +1;
uhmin = -1; uhmax = +1;
taumin = -50; taumax = 0;

x0 = [p0, f0, g0, h0, k0, L0, w0]           # Initial state
n = 20
ns = 7
nu = 3
@variable(dubins, kp[1:1])
function cross(a, b)
    return [a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]]
end

function f(x, u)
    # Extract state, control, and parameter for problem
    p = x[1]
    f = x[2]
    g = x[3]
    h = x[4]
    k = x[5]
    L = x[6]
    w = x[7]
    ur = u[1]
    ut = u[2]
    uh = u[3]
    tau = kp[1]

    # Gravitational disturbing acceleration
    q = 1 + f * cos(L) + g * sin(L)
    r = p / q
    alpha2 = h * h - k * k
    chi = sqrt(h * h + k * k)
    s2 = 1 + chi * chi
    rX = (r / s2) * (cos(L) + alpha2 * cos(L) + 2 * h * k * sin(L))
    rY = (r / s2) * (sin(L) - alpha2 * sin(L) + 2 * h * k * cos(L))
    rZ = (2 * r / s2) * (h * sin(L) - k * cos(L))
    rVec = [rX, rY, rZ]
    rMag = sqrt(rX^2 + rY^2 + rZ^2)
    rXZMag = sqrt(rX^2 + rZ^2)
    vX = -(1 / s2) * sqrt(mu / p) * (sin(L) + alpha2 * sin(L) - 2 * h * k * cos(L) + g - 2 * f * h * k + alpha2 * g)
    vY = -(1 / s2) * sqrt(mu / p) * (-cos(L) + alpha2 * cos(L) + 2 * h * k * sin(L) - f + 2 * g * h * k + alpha2 * f)
    vZ = (2 / s2) * sqrt(mu / p) * (h * cos(L) + k * sin(L) + f * h + g * k)
    vVec = [vX, vY, vZ]
    rCrossv = cross(rVec, vVec)
    rCrossvMag = sqrt(rCrossv[1]^2 + rCrossv[2]^2 + rCrossv[3]^2)
    rCrossvCrossr = cross(rCrossv, rVec)
    ir1 = rVec[1] / rMag
    ir2 = rVec[2] / rMag
    ir3 = rVec[3] / rMag
    ir = [ir1, ir2, ir3]
    it1 = rCrossvCrossr[1] / (rCrossvMag * rMag)
    it2 = rCrossvCrossr[2] / (rCrossvMag * rMag)
    it3 = rCrossvCrossr[3] / (rCrossvMag * rMag)
    it = [it1, it2, it3]
    ih1 = rCrossv[1] / rCrossvMag
    ih2 = rCrossv[2] / rCrossvMag
    ih3 = rCrossv[3] / rCrossvMag
    ih = [ih1, ih2, ih3]
    enir = ir3
    enirir1 = enir * ir1
    enirir2 = enir * ir2
    enirir3 = enir * ir3
    enenirir1 = 0 - enirir1
    enenirir2 = 0 - enirir2
    enenirir3 = 1 - enirir3
    enenirirMag = sqrt(enenirir1^2 + enenirir2^2 + enenirir3^2)
    in1 = enenirir1 / enenirirMag
    in2 = enenirir2 / enenirirMag
    in3 = enenirir3 / enenirirMag
    # Geocentric latitude
    sinphi = rZ / rXZMag
    cosphi = sqrt(1 - sinphi^2)
    # Legendre polynomials
    P2 = (3 * sinphi^2 - 2) / 2
    P3 = (5 * sinphi^3 - 3 * sinphi) / 2
    P4 = (35 * sinphi^4 - 30 * sinphi^2 + 3) / 8
    dP2 = 3 * sinphi
    dP3 = (15 * sinphi - 3) / 2
    dP4 = (140 * sinphi^3 - 60 * sinphi) / 8
    # Oblate earth perturbations
    sumn = (Re / r)^2 * dP2 * J2 + (Re / r)^3 * dP3 * J3 + (Re / r)^4 * dP4 * J4
    sumr = (2 + 1) * (Re / r)^2 * P2 * J2 + (3 + 1) * (Re / r)^3 * P3 * J3 + (4 + 1) * (Re / r)^4 * P4 * J4
    ∆gn = -(mu * cosphi / (r^2)) * sumn
    ∆gr = -(mu / (r^2)) * sumr
    ∆gnin1 = ∆gn * in1
    ∆gnin2 = ∆gn * in2
    ∆gnin3 = ∆gn * in3
    ∆grir1 = ∆gr * ir1
    ∆grir2 = ∆gr * ir2
    ∆grir3 = ∆gr * ir3
    ∆g1 = ∆gnin1 - ∆grir1
    ∆g2 = ∆gnin2 - ∆grir2
    ∆g3 = ∆gnin3 - ∆grir3
    Deltag1 = ir[1] * ∆g1 + ir[2] * ∆g2 + ir[3] * ∆g3
    Deltag2 = it[1] * ∆g1 + it[2] * ∆g2 + it[3] * ∆g3
    Deltag3 = ih[1] * ∆g1 + ih[2] * ∆g2 + ih[3] * ∆g3
    # Thrust acceleration
    DeltaT1 = ((gs * T * (1 + 0.01 * tau)) / w) * ur
    DeltaT2 = ((gs * T * (1 + 0.01 * tau)) / w) * ut
    DeltaT3 = ((gs * T * (1 + 0.01 * tau)) / w) * uh
    # Total acceleration
    Delta1 = Deltag1 + DeltaT1
    Delta2 = Deltag2 + DeltaT2
    Delta3 = Deltag3 + DeltaT3
    # Differential equations of motion
    dp = (2 * p / q) * sqrt(p / mu) * Delta2
    df = sqrt(p / mu) * sin(L) * Delta1 + sqrt(p / mu) * (1 / q) * ((q + 1) * cos(L) + f) * Delta2 - sqrt(p / mu) * (g / q) * (h * sin(L) - k * cos(L)) * Delta3
    dg = -sqrt(p / mu) * cos(L) * Delta1 + sqrt(p / mu) * (1 / q) * ((q + 1) * sin(L) + g) * Delta2 + sqrt(p / mu) * (f / q) * (h * sin(L) - k * cos(L)) * Delta3
    dh = sqrt(p / mu) * (s2 * cos(L) / (2 * q)) * Delta3
    dk = sqrt(p / mu) * (s2 * sin(L) / (2 * q)) * Delta3
    dL = sqrt(p / mu) * (1 / q) * (h * sin(L) - k * cos(L)) * Delta3 + sqrt(mu * p) * ((q / p)^2)
    dw = -(T * (1 + 0.01 * tau) / Isp)

    return [dp, df, dg, dh, dk, dL, dw]
end


@variables(dubins, begin
    Δt   # Time step
    # State variables
    x[1:7,1:n]               # Velocity
    # Control variables
    u[1:3,1:n]    # Thrust
end)


# Dynamic constraints: hermite_simpson
for j in 1:n-1
    f1 = f(x[1:ns, j], u[1:nu, j])
    f2 = f(x[1:ns, j+1], u[1:nu, j+1])

    # hermite-simpson
    ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

    xb = (x[1:ns, j] + x[1:ns, j+1]) / 2 + Δt * (f1 - f2) / 8
    fb = f(xb, ub)

    dx = Δt * (f1 + 4 * fb + f2) / 6

    # # trapezoidal
    # dx = Δt * (f1 + f2) / 2

    # @infiltrate
    @NLconstraint(dubins, x[1, j+1] == x[1, j] + dx[1])
end

# Objective
function obj_f(x, u, Δt)
    return Δt
end
@objective(dubins, Min, obj_f(x[1:7,1:n], u[1:3,1:n], Δt))

#Initial conditions
@constraint(dubins, x[1:7, 1] .== x0 )

optimize!(dubins)