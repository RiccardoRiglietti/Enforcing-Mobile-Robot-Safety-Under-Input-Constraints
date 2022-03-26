function [h1, dh1, h2, dh2, h3, dh3] = ret_ZCBF_comp(x, f, g, af, al)
%% [h1, dh1, h2, dh2, h3, dh3] = ret_ZCBF(x, f, g, af, al)
% Compute the symbolic Lie derivatives of h and return it in all the 3 cases
% with dhi = [Lfhi, Lghi]
    
    vf = x(2);
    vl = x(3);
    D = x(1);
    g0 = 9.81;
    
    dh1 = sym('dh1', [1 2]);
    dh2 = sym('dh2', [1 2]);
    dh3 = sym('dh3', [1 2]);
    
    % case 1
    delta_opt1 = 1.8*vf;
    h1 = D - delta_opt1;
    dh1(1) = Lie(f, h1, x);
    dh1(2) = Lie(g, h1, x);
    
    % case 2
    delta_opt2 = 0.5*(1.8*af*g0 - vf)^2/(af*g0) + 1.8*vf - (vl)^2/(2*al*g0);
    h2 = D - delta_opt2;
    dh2(1) = Lie(f, h2, x);
    dh2(2) = Lie(g, h2, x);
    
    % case 3
    if al ~= af
        delta_opt3 = 0.5*(vl + 1.8*af*g0 - vf)^2/((af - al)*g0) + 2.8*vf;
        h3 = D - delta_opt3;
        dh3(1) = Lie(f, h3, x);
        dh3(2) = Lie(g, h3, x);
    else
        h3 = 0;
    end
        

end

