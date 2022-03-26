function min_gamma = computation_of_gamma(dmax, vmax, k2, b0, b1, b2, b2_dot_sup)
%% min_gamma = computation_of_gamma(dmax, vmax, k2, b0, b1, b2, b2_dot_sup)
% function that controls if b2 is a proper CBF solving
%       min_gamma = argmin_x [b2_dot_sup + k2*b2]
%           subjected to x in C*
    
    syms x1 x2 real
    x = [x1 x2]';
    t1 = linspace(0, dmax);
    t2 = linspace(0, vmax);
    
    gamma = 100*ones([dmax dmax]);
    for i = 1:100
        for j = 1:100
            
            val0 = double(subs(b0, x, [t1(i) t2(j)]'));
            val1 = double(subs(b1, x, [t1(i) t2(j)]'));
            val2 = double(subs(b2, x, [t1(i) t2(j)]'));
            
            if val0 >= 0 && val1 >= 0 && isreal(val2) && val2 >= 0
                gamma(i, j) = double(subs(b2_dot_sup + k2*b2, x, [t1(i) t2(j)]'));
            end
        end
    end
    min_gamma = min(gamma, [], 'all');
end