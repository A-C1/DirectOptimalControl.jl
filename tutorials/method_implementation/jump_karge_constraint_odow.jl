using JuMP
import Ipopt

cross(a, b) = [a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1]]

function f(x, u, tau)
    T = 4.446618e-3
    Isp = 450
    mu = 1.407645794e16
    gs = 32.174
    Re = 20925662.73
    J2 = 1082.639e-6
    J3 = -2.565e-6
    J4 = -1.608e-6
    # Extract state, control, and parameter for problem
    p, f, g, h, k, L, w = x
    ur, ut, uh = u
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

n = 10
x_0 = [21837080.052835, 0, 0, -0.25396764647494, 0, pi, 1]
x_l = [20_000_000, -1, -1, -1, -1, x_0[6], 0.1]
x_u = [60_000_000, 1, 1, 1, 1, 9*2*pi, x_0[7]]

model = Model(Ipopt.Optimizer)
@variables(model, begin
    0 <= Δt <= 1000_000
    x_l[i] <= x[i = 1:7, 1:n] <= x_u[i]
    -1 <= u[1:3, 1:n] <= 1
    -50 <= tau <= 0
    f_i[1:7, 1:n]
    u_b[1:3, 1:n-1]
    x_b[1:7, 1:n-1]
    f_b[1:7, 1:n-1]
    d_x[1:7, 1:n-1]
end)
@objective(model, Max, x[4, n])
fix.(x[1:7, 1], x_0; force = true)
@constraints(model, begin
    [j in 1:n], f_i[:, j] .== f(x[:, j], u[:, j], tau)
    [j in 1:n-1], u_b[:, j] .== (u[:, j] + u[:, j+1])/2
    [j in 1:n-1], x_b[:, j] .== (x[:, j] + x[:, j+1]) / 2 + Δt * (f_i[:, j] - f_i[:, j+1]) / 8
    [j in 1:n-1], f_b[:, j] .== f(x_b[:, j], u_b[:, j], tau)
    [j in 1:n-1], d_x[:, j] .== Δt * (f_i[:, j] + 4 * f_b[:, j] + f_i[:, j+1]) / 6
    # [j in 1:n-1], d_x[:, j] .== Δt * (f_i[:, j] + f_i[:, j+1]) / 2
    [j in 1:n-1], x[:, j+1] .== x[:, j] + Δt * (f_i[:, j] + 4 * f_b[:, j] + f_i[:, j+1]) / 6
    # [j in 1:n-1], x[:, j+1] .== x[:, j] + d_x[:, j]
end);

optimize!(model)