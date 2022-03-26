function p = regionplot(f, t, limx, limy, col)
%% p = regionplot(f, t, limx, limy, col)
% function used in order to plot the safety region of the i-th CBF

    xlim(limx);
    ylim(limy);
    a = area(t, f); hold on;
    a.FaceAlpha = 0.8;
    a.FaceColor = col;

end

