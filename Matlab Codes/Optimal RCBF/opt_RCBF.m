function [B, dB, h] = opt_RCBF(af, al, vf, vl, g0, B1, dB1, B2, dB2, B3, dB3, h1, h2, h3)
%% [B, dB, h] = opt_RCBF(af, al, vf, vl, g0, B1, dB1, B2, dB2, B3, dB3, h1, h2, h3)
% Select the optimal RCBF to be used depending on both the velocities and
% the acceleration of the leading and following vehicles

    if af == al
        if vf > 0 && vf < vl + 1.8*af*g0
            B = B1;
            dB = dB1;
            h = h1;
        else
            B = B2;
            dB = dB2;
            h = h2;
        end

    elseif af > al
        if vf > 0 && vf < vl + 1.8*af*g0
            B = B1;
            dB = dB1;
            h = h1;
        elseif vf > (af/al)*vl + 1.8*af*g0
            B = B2;
            dB = dB2;
            h = h2;
        else
            B = B3;
            dB = dB3;
            h = h3;
        end

    elseif af < al
        if (vf > 0 && vf < sqrt(af/al)*vl + 1.8*af*g0)
            B = B1;
            dB = dB1;
            h = h1;
        else
            B = B2;
            dB = dB2;
            h = h2;
        end
    end


end

