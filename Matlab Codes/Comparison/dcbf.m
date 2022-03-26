function [dbi_inf, dbi_sup] = dcbf(x, f, g, bi, umax)
%% [dbi_inf, dbi_sup] = dcbf(x, f, g, bi, umax)
% This function takes the i-th CBF (bi) and provides dbi_inf and dbi_sup 
% which are used to compute the next CBF

    % compute Lie derivatives of i-th CBF
    Lfbi = Lie(f, bi, x);
    Lgbi = Lie(g, bi, x);

    % compute inf and sup of deriative of i-th CBF
    dbi_inf = simplify(Lfbi - abs(Lgbi)*umax);
    dbi_sup = simplify(Lfbi + abs(Lgbi)*umax);
    
end

