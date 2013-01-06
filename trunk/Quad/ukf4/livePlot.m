function out = livePlot(varargin)
% live plot function that takes input of the format: name1, data1, name2,
% data2, ..., followed by current iteration

iteration = varargin{nargin};
AXIS_X_MIN = iteration-500;
AXIS_X_MAX = iteration+200;
AXIS_Y_MIN = -20;
AXIS_Y_MAX = 20;

length = (nargin-1)/2;
cmap = hsv(6); % color map

for i = 1:length
    subplot(length, 1, i);
    colorIndex = mod(i, 6);
    data = varargin{2*i};
    plot(data);
    title(varargin{2*i-1});
    axis([AXIS_X_MIN, AXIS_X_MAX, AXIS_Y_MIN, AXIS_Y_MAX]);
end
drawnow

out = 0;
end