function cmap = redblue()
% REDBLUE - Red-white-blue diverging colormap for correlation visualization
%
% SYNTAX:
%   cmap = redblue()
%
% OUTPUTS:
%   cmap - [128x3] colormap matrix
%
% USAGE:
%   colormap(redblue());
%
% NOTES:
%   Red indicates positive correlation
%   Blue indicates negative correlation
%   White is zero correlation

    n = 64;
    r = [(0:n-1)'/n; ones(n,1)];
    g = [(0:n-1)'/n; flipud((0:n-1)'/n)];
    b = [ones(n,1); flipud((0:n-1)'/n)];
    cmap = [r g b];
end