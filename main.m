% initial parameters (run before simulink)
m = 0.3;
M = 1.5;
L = 1.2;
g = 9.81;
d = 0.7;

b = -1; % pendulum up (b=1)

A = [0   1          0               0;
     0  -d/M        b*m*g/M         0;
     0   0          0               1;
     0  -b*d/(M*L) -b*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; b/(M*L)];
C = [1 0 0 0;
     0 0 1 0];

%% To visualize the pendulum (run after simulink)
x = out.u_out.Data;
t = out.u_out.Time;
figure; % Create a new figure window for the animation
filename = 'pendulum_animationPlusg22.gif'; % Name of the file
frame_delay = 0.005; % Time between frames (1/25 = 0.04 for 25 fps)

for k=1:15:length(t)
    drawpend(x(k,:),m,M,L);
    % Force MATLAB to draw the image in the window *now*
    drawnow; 
    
    % --- Capture and Save Frame ---
    frame = getframe(gcf); % Capture the current figure window (gcf)
    im = frame2im(frame);  % Convert the frame to a true-color image
    [imind, cm] = rgb2ind(im, 256); % Convert the image to an indexed image for GIF

    % On the first frame, create the file. On subsequent frames, append.
    if k == 1
        % Write the first frame to the file
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', frame_delay);
    else
        % Append subsequent frames to the file
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_delay);
    end
end

disp(['Animation saved successfully to: ', filename]);


%%
% =======================
% D matrix is almost always zero for mechanical systems
D = [0; 0];

% Create a state-space object for analysis (optional, but good practice)
sys = ss(A, B, C, D);


%% Step 3: Design the State-Feedback Controller Gain (K)
% We use LQR to find the optimal gain K for the control law u = -Kx.
% We need to define our performance weights Q and R.

% Q penalizes state deviations [x, x_dot, theta, theta_dot]
% Make the penalty for theta (angle) high to prioritize stabilization.
% Make the penalty for x (position) non-zero to control position.
Q = diag([10, 1, 10, 1]); 

% R penalizes the control effort (the force u)
R = 0.0001;

% Calculate the LQR gain K
K = lqr(A, B, Q, R);


%% Step 4: Design the State Observer Gain (L)
% We want the observer to be faster than the controller.
% A good rule of thumb is to make the observer poles 2-5 times faster 
% than the controller poles.

% First, find the poles of the closed-loop controller
controller_poles = eig(A - B*K);

% Choose the desired observer poles to be 3x faster
observer_poles = 3 * controller_poles;

% Calculate the observer gain L using pole placement.
% The command uses the transpose of A and C, and the final result must be transposed back.
% This is due to the duality between controllability and observability.
Lc = place(A', C', observer_poles)';
%% Display the Results
disp('System Matrix A:');
disp(A);
disp('Input Matrix B:');
disp(B);
disp('Output Matrix C:');
disp(C);
disp('State-Feedback Gain K:');
disp(K);
disp('Observer Gain L:');
disp(Lc);