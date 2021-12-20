% https://www.mathworks.com/matlabcentral/answers/94495-how-can-i-create-animated-gif-images-in-matlab

function write2gif(h, n, filename, delay)

% Capture the plot as an image
frame = getframe(h);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,256);
% Write to the GIF File
if n == 1
    imwrite(imind,cm,filename,'gif', 'DelayTime',delay, 'Loopcount',inf);
else
    imwrite(imind,cm,filename,'gif', 'DelayTime',delay,'WriteMode','append');
end

end