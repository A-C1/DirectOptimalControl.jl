function path_constraint_adjoints(ph, OC)
    n = length(ph.path_constraints_upper)
    m = length(ph.path_constraints_upper[1])
    mu = zeros(m, n)
    h = value.(ph.h)

    if ph.collocation_method == "trapezoidal"
        mu[:, 1] = - (2/h[1])*(dual.(ph.path_constraints_upper[1]) + dual.(ph.path_constraints_lower[1]))
        for i = 2:n-1
            mu[:, i] .= - (2/(h[i-1] + h[i]))*(dual.(ph.path_constraints_upper[i]) + dual.(ph.path_constraints_lower[i]))
        end
        mu[:, n] = - (2/h[n-1])*(dual.(ph.path_constraints_upper[n]) + dual.(ph.path_constraints_lower[n]))
    elseif ph.collocation_method == "hermite-simpson"
        mu[:, 1] = - (6/h[1])*(dual.(ph.path_constraints_upper[1]) + dual.(ph.path_constraints_lower[1]))
        for i = 2:n-1
            mu[:, i] .= - (6/(h[i-1] + h[i]))*(dual.(ph.path_constraints_upper[i]) + dual.(ph.path_constraints_lower[i]))
        end
        mu[:, n] = - (6/h[n-1])*(dual.(ph.path_constraints_upper[n]) + dual.(ph.path_constraints_lower[n]))
    end
    return mu
end

function differential_constraints_adjoints(ph, OC)
    n = length(ph.collocation_constraints)
    m = ph.ns

    lambda = zeros(m, n)

    for i = 1:n
        lambda[:,i] .= dual.(ph.collocation_constraints[i])
    end

    return lambda
end
