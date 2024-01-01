function path_constraint_adjoints(ph, OC)
    if ph.collocation_method == "trapezoidal"
        n = length(ph.path_constraints_upper)
        m = length(ph.path_constraints_upper[1])
        mu = zeros(m, n)

        h = value.(ph.h)

        mu[:, 1] = - (2/h[1])*ph.path_constraints_upper[1]
        for i = 2:n-1
            mu[:, i] .= - (2/(h[i-1] + h[i]))*ph.path_constraints_upper[i] 
        end
        mu[:, n] = - (2/h[n-1])*ph.path_constraints_upper[n]
        

        return mu
    elseif ph.collocation_method == "hermite-simpson"
    end
end

function differential_constraints(ph, OC)
    n = length(ph.collocation_constraints)
    m = ph.ns

    
     
end