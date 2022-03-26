function [x_new, y_new] = area_bounds(x, bi, vmax, dmax)
%% [x_new, y_new] = area_bounds(x, bi, vmax, dmax)
% This function takes the i-th CBF (bi) and provides the boundaries of the
% corresponding safety region (x_new, y_new) by computing linear 
% interpolation, if the function is linear in x1, or by solving a
% minimization problem of the absolute value of the bi.

    % utilities
    x1 = x(1);
    t1 = linspace(0, dmax);
    opt = optimset('Display', 'off');
    
    % second order derivative of bi wrt x1
    ddbi = diff(diff(bi, x1), x1);
    
    % if the function is linear in x1, i.e. the second derivative is 0,
    % we can use linear interpolation
    if ddbi == 0
        
        % initial values
        x1i = 0;
        bi_x1i = simplify(subs(bi, x1, x1i));
        fun = @(x2) bi_sub_x2(bi_x1i, x2);
        [x2i, ~] = fminunc(fun, vmax/2, opt);
        
        % final values
        x1f = dmax;
        bi_x1f = simplify(subs(bi, x1, dmax));
        fun = @(x2) bi_sub_x2(bi_x1f, x2);
        [x2f, ~] = fminunc(fun, vmax/2, opt);
        
        % linear interpolation
        m = (x2f - x2i)/(x1f - x1i);
        q = -m*x1i + x2i;
        
        x_new = t1;
        y_new = m*x_new + q;

    % else, we solve a minimization problem
    else
        
        x2_sol = zeros(1, dmax);
        for i = 1:100
            bi_x1 = simplify(subs(bi, x1, t1(i)));
            fun = @(x2) bi_sub_x2(bi_x1, x2);
            [x2s, ~] = fminunc(fun, vmax/2, opt);
            x2_sol(i) = x2s;
        end

        x_new = t1;
        y_new = x2_sol;
    
    end

end

