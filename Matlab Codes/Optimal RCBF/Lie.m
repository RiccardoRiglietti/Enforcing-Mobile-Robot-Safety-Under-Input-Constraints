function Lie = Lie(f, h, x)
%% Lie = Lie(f,h,x)
% computes the Lie derivative of vector h wrt x along f

    Lie = simplify(jacobian(h, x)*f);

end

