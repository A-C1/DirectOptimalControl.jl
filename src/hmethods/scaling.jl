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
        x = xs
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
        xs = x
    end

    return xs
end