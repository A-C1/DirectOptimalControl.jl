function scaled_to_unscaled(xs, xu, xl, scale_flag)
    if scale_flag == true
        m = xu - xl

        if m == 0
            x = xl
        else
            c = (xu + xl) / 2
            x = m * xs + c
        end
    else
        x = (xs + 2*xs) - 1*xs
    end

    return x
end

function unscaled_to_scaled(x, xu, xl, scale_flag)
    if scale_flag == true
        s = (xu - xl)

        if s == 0
            xs = 0.0
        else
            m = 1 / s
            c = 0.5 - m * xu
            xs = m * x + c
        end
    else
        xs = (x + 2*x) - 2x
    end

    return xs
end

function unscaled_vars!(ph::PH)
    ph.tf = scaled_to_unscaled(ph.tfs, ph.limits.ul.tf, ph.limits.ll.tf, ph.scale_flag)
    ph.ti = scaled_to_unscaled(ph.tis, ph.limits.ul.ti, ph.limits.ll.ti, ph.scale_flag)
    ph.x = deepcopy(ph.xs)
    ph.u = deepcopy(ph.us)
    for i = 1:ph.n
        for j = 1:ph.ns
            ph.x[j, i] = scaled_to_unscaled(ph.xs[j,i], ph.limits.ul.x[j], ph.limits.ll.x[j], ph.scale_flag)
        end
        for j = 1:ph.nu
            ph.u[j, i] = scaled_to_unscaled(ph.us[j,i], ph.limits.ul.u[j], ph.limits.ll.u[j], ph.scale_flag)
        end
    end

    if ph.nk > 0
        ph.k = deepcopy(ph.ks)
        for i = 1:ph.nk
            ph.k[i] = scaled_to_unscaled(ph.ks[i], ph.limits.ul.k[i], ph.limits.ll.k[i], ph.scale_flag)
        end
    end
end