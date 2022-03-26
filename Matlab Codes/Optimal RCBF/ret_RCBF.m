function [B1, dB1, h1, B2, dB2, h2, B3, dB3, h3] = ret_RCBF(x, f, g, af, al)
%% [B1, dB1, h1, B2, dB2, h2, B3, dB3, h3] = ret_RCBF(x, f, g, af, al)
% Compute the symbolic Lie derivatives of the reciprocal barrier function
% B = 1/h in all the 3 cases. dBi = [LfBi, LgBi]
    
    vf = x(1);
    vl = x(2);
    D = x(3);
    g0 = 9.81;
    
    dB1 = sym('dB1', [1 2]);
    dB2 = sym('dB2', [1 2]);
    dB3 = sym('dB3', [1 2]);
    
    % case 1
    delta_opt1 = 1.8*vf;
    h1 = D - delta_opt1;
    B1 = simplify(1/h1);
%     B1 = simplify(-log(h1/(1+h1)));
    dB1(1) = Lie(f, B1, x);
    dB1(2) = Lie(g, B1, x);
    
    % case 2
    delta_opt2 = 0.5*(1.8*af*g0 - vf)^2/(af*g0) + 1.8*vf - (vl)^2/(2*al*g0);
    h2 = D - delta_opt2;
    B2 = simplify(1/h2);
%     B2 = simplify(-log(h2/(1+h2)));
    B2 = expand(B2);
    dB2(1) = Lie(f, B2, x);
    dB2(2) = Lie(g, B2, x);
    
    % case 3
    if al ~= af
        delta_opt3 = 0.5*(vl + 1.8*af*g0 - vf)^2/((af - al)*g0) + 2.8*vf;
        h3 = D - delta_opt3;
        B3 = simplify(1/h3);
%       B3 = simplify(-log(h3/(1+h3)));
        dB3(1) = Lie(f, B3, x);
        dB3(2) = Lie(g, B3, x);
    else
        h3 = 0;
        B3 = 0;
    end

end

