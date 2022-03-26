function [v_l, a_l] = va_profiles(tspan)
%% [v_l, a_l] = va_profiles(dim)
% a function that provides a trapezoidal speed profile of the leading car.

    dim = max(size(tspan));

    a_l = zeros(1, dim);
    v_l = zeros(1, dim);

    for i=1:dim
        if tspan(i) < 10
            v_l(i) = 10;
        elseif tspan(i) >= 10 && tspan(i) < 18
            m = ca(10, 18, 10, 22);
            q = tn(10, 10, m);
            v_l(i) = m*tspan(i) + q;
            a_l(i) = m;
        elseif tspan(i) >= 18 && tspan(i) < 30
            v_l(i) = 22;
        elseif tspan(i) >= 30 && tspan(i) < 40
            m = ca(30, 40, 22, 27);
            q = tn(30, 22, m);
            v_l(i) = m*tspan(i) + q;
            a_l(i) = m;
        elseif tspan(i) >= 40 && tspan(i) < 50
            v_l(i) = 27;
        elseif tspan(i) >= 50 && tspan(i) < 60
            m = ca(50, 60, 27, 17);
            q = tn(50, 27, m);
            v_l(i) = m*tspan(i) + q;
            a_l(i) = m;
        else
            v_l(i) = 17;
        end
    end
    
end

function m = ca(xi, xf, yi, yf)
    m = (yf - yi)/(xf - xi);
end

function q = tn(xi, yi, m)
    q = -xi*m + yi;
end
