function [h, dh] = opt_ZCBF(af, al, vf, vl, g0, h1, h2, h3, dh1, dh2, dh3)
%% [B, dB, h] = opt_ZCBF(af, al, vf, vl, g0, B1, dB1, B2, dB2, B3, dB3, h1, h2, h3)
% Select the optimal ZCBF to be used depending on both the velocities and
% the acceleration of the leading and following vehicles

    if af == al
        if vf > 0 && vf < vl + 1.8*af*g0
            h = h1;
            dh = dh1;
        else
            h = h2;
            dh = dh2;
        end

    elseif af > al
        if vf > 0 && vf < vl + 1.8*af*g0
            h = h1;
            dh = dh1;
        elseif vf > (af/al)*vl + 1.8*af*g0
            h = h2;
            dh = dh2;
        else
            h = h3;
            dh = dh3;
        end

    elseif af < al
        if (vf > 0 && vf < sqrt(af/al)*vl + 1.8*af*g0)
            h = h1;
            dh = dh1;
        else
            h = h2;
            dh = dh2;
        end
    end


end

