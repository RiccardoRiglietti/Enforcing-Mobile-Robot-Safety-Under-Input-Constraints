function dx = sys(t, x, fn, g, u)

    dx = fn + g.*u;

end