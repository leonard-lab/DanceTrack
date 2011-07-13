clear all;
close all;

addpath('support_files');

% average height of dancers (estimate) [meters]
HEIGHT = 1.65;
% FPS of the video
FPS = 50;
% the tracking got mucked up around frame 2300
LAST_GOOD_FRAME = 2300;

f = ReadXDF('./Flock1.xdf');

% frame number to convert to time stamp
frame_number = LoadXDFDataFileByName(f, 'Frame_Number');
% position in image coordinates
X_I = LoadXDFDataFileByName(f, 'Tracked_X');
Y_I = LoadXDFDataFileByName(f, 'Tracked_Y');
% these were typos on my part - the names are wrong but the data is
% velocity in image coordinates
VX_I = LoadXDFDataFileByName(f, 'Tracked_Heading');
VY_I = LoadXDFDataFileByName(f, 'Tracked_Speed');

% read the camera calibration information
[fc, cc, alpha_c, kc, Rc, Tc] = ReadCalibrationData('Camera1Calibration.dat');

[Nt, Nd] = size(X_I);
Nt = LAST_GOOD_FRAME;
T = zeros(Nt, 1);
X = zeros(Nt, Nd);
Y = zeros(Nt, Nd);

% convert the positions to world coordinates using the camera calibration
CW = -Rc'*Tc;
ch = CW(3);
Rt = Rc';
for tx = 1 : Nt,
    for px = 1 : Nd,
        u = X_I(tx, px);
        v = Y_I(tx, px);
        x = (u - cc(1))/fc(1);
        y = (v - cc(2))/fc(2);
        
        r = [x; y; 1];
        
        t = (HEIGHT-ch)/(Rt(3,:)*r);
        rW = CW + t*Rt*r;
        if(abs(rW(3) - HEIGHT) > 1e-3),
            pause
        end
        
        T(tx) = (1/FPS)*frame_number(tx);
        X(tx, px) = rW(1);
        Y(tx, px) = rW(2);
        
    end
end

T = T - T(1);
Torig = T;
Xorig = X;
Yorig = Y;

% use interpolation to get evenly-spaced time steps
t = [0 : 1/20 : T(end)];
X = interp1(T, Xorig, t);
Y = interp1(T, Yorig, t);

% estimate the velocity by taking differences
Vx = diff(X)/(t(2)-t(1));
Vy = diff(Y)/(t(2)-t(1));
% make the vectors the right size by duplicating the last value
Vx = [Vx; Vx(end, :)];
Vy = [Vy; Vy(end, :)];

% the orientation can be estimated as the angle of the velocity vector
Theta = atan2(Vy, Vx);
% the speed is the magnitude of the velocity vector
Speed = sqrt(Vx.^2 + Vy.^2);

figure(1)
plot(X, Y);
xlabel('Position x [m]')
ylabel('Position y [m]')

figure(2)
subplot(2, 1, 1)
plot(t, Speed)
xlabel('Time [sec]')
ylabel('Speed [m/sec]')
xlim([0 t(end)])
subplot(2, 1, 2)
plot(t, unwrap(Theta))
xlabel('Time [sec]')
ylabel('Orientation [rad]')
xlim([0 t(end)])