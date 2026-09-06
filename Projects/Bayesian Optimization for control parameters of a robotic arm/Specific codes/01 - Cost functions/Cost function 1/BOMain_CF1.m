%% BAYESIAN OPTIMIZATION

%% Cleanup %%

clear;
close all;
clear obj_PID_panda_1
delete(findall(0, 'Type', 'figure', 'Tag', 'TMWWaitbar'));

addpath(genpath('../../Necessary folders'));

%% Weights  definition %% 

w_pos_max = 50;
w_vel_max = 0;
w_pos_avg = 30;
w_vel_avg = 5;
w_jerk_mean = 1;   
w_tau_max = 0.02;

%% REQUESTS

project_case = 1;
[parallel_var, sat_var, rampa_var, exploration_iters, exploitation_iters, num_Seed_points] = user_requests(project_case);

%% Robot defiition

n_DoFs = 7;

friction = [2 2 2 2 2 2 2];

Robot = panda_robot();

%% Motion Reference

Ts = 1e-3;       
Tsim = 4;        

t0 = 0; 
tf = 4.; 
time = t0:Ts:Tsim; 

q0 = [-0.7160   -0.5850    0.3504   -1.5666    0.2241   -2.1201   -2.8398];

bool_figures = 0;
CF=1;
r = reference(rampa_var, time, q0, bool_figures, CF, Robot);

q_r = r.q_r;
dq_r = r.dq_r;
ddq_r = r.ddq_r;

disp('going to optimization...');

toll_qerr = 25*pi/180;
toll_qerr_min = 15*pi/180;

%% All constants in a structure for convenience 

const.Ts = Ts; 
const.Tsim = Tsim;
const.time = time;
const.n_DoFs = n_DoFs;
const.r = r;
const.Robot = Robot;
const.Robot_friction = friction;
const.q_0 = q0;
const.toll_qerr = toll_qerr;
const.toll_qerr_min = toll_qerr_min;
const.sat_var = sat_var;
const.w_pos_max = w_pos_max;
const.w_vel_max = w_vel_max;
const.w_pos_avg = w_pos_avg;
const.w_vel_avg = w_vel_avg;
const.w_jerk_mean = w_jerk_mean;   
const.w_tau_max = w_tau_max;

%% Bayesian optimization of controller parameters 

opt_vars = [];

bound_min_Kp = 0.;
bound_max_Kp = 10000;

bound_min_Kd = 0;
bound_max_Kd = 500;

bound_min_Ki = 0.;
bound_max_Ki = 1000.;

% Define bounds on the optimization variables first four joints

bound_max_Kp_new = 11000;
bound_max_Kd_new = 550;
bound_max_Ki_new = 1100;

%% Optimizable variables definition

opt_vars = [opt_vars optimizableVariable('Kp1', [bound_min_Kp, bound_max_Kp_new],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki1', [bound_min_Ki, bound_max_Ki_new],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd1', [bound_min_Kd, bound_max_Kd_new],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp2', [bound_min_Kp, bound_max_Kp_new],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki2', [bound_min_Ki, bound_max_Ki_new],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd2', [bound_min_Kd, bound_max_Kd_new],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp3', [bound_min_Kp, bound_max_Kp_new],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki3', [bound_min_Ki, bound_max_Ki_new],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd3', [bound_min_Kd, bound_max_Kd_new],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp4', [bound_min_Kp, bound_max_Kp_new],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki4', [bound_min_Ki, bound_max_Ki_new],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd4', [bound_min_Kd, bound_max_Kd_new],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp5', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki5', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd5', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp6', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki6', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd6', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp7', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki7', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd7', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative

%% Define objective function as a function of the optimization variables

func =  @(x_vars)(obj_PID_panda_1(x_vars,const));
initial_X = []; 

clear objBO; 

%% FIRST PHASE: EXPLORATION

results_explore = bayesopt(func, opt_vars, ...
    'Verbose', 0, ...
    'AcquisitionFunctionName', ...
    'lower-confidence-bound', ...
    'IsObjectiveDeterministic', true, ...
    'UseParallel', parallel_var,...
    'MaxObjectiveEvaluations', exploration_iters, ...
    'MaxTime', inf, ...
    'NumSeedPoints', num_Seed_points, ...
    'GPActiveSetSize', 300 ,...
    'PlotFcn', []);

%% Adjustment lower K bound

K_mat = table2array(results_explore.XTrace);
Kp_mat =K_mat(:,1:3:19);
bound_min_Kp = min(min(Kp_mat));

%% SECOND PHASE: EXPLOITATION 

results = bayesopt(func, opt_vars, ...
    'Verbose', 0, ...
    'AcquisitionFunctionName', ...
    'expected-improvement', ...
    'IsObjectiveDeterministic', true, ...
    'UseParallel', parallel_var,...
    'MaxObjectiveEvaluations', exploitation_iters+exploration_iters, ...
    'MaxTime', inf, ...
    'InitialX', results_explore.XTrace, ...
    'InitialObjective', results_explore.ObjectiveTrace, ...
    'GPActiveSetSize', 300, ...
    'PlotFcn', []);

%% Extract best variables

best_vars = results.bestPoint;
opt_cost=obj_PID_panda_1(best_vars,const);
idx_min = results.IndexOfMinimumTrace(end);

%% Plot BO research

N = length(results.ObjectiveTrace);

figure
plot(1:N, results.ObjectiveTrace, 'k*')
hold on;
min_ever = results.ObjectiveMinimumTrace;
min_ever(1:exploration_iters) = results_explore.ObjectiveMinimumTrace;
plot(min_ever, 'r', 'LineWidth', 2)
plot(idx_min, min_ever(idx_min), 'MarkerEdgeColor','black',...
    'MarkerFaceColor','gree', 'Marker', 'square', 'MarkerSize',10);
h=xlabel('Iteration index $i$ (-)');
set(h,'Interpreter', 'Latex');
h=ylabel('Performance cost  $\tilde J$ (-)');
set(h,'Interpreter', 'Latex');
grid('on');
xline(exploration_iters, '--')
title('Objective function evolution plot')
legend('Current point', 'Current best point', 'Overall best point', 'Exploration vs exploitation');

%% OPTIMAL DYNAMICS

Robot_eval = panda_robot();

Kp = diag([results.bestPoint.Kp1, results.bestPoint.Kp2, results.bestPoint.Kp3, results.bestPoint.Kp4, results.bestPoint.Kp5, results.bestPoint.Kp6, results.bestPoint.Kp7]);
Ki = diag([results.bestPoint.Ki1, results.bestPoint.Ki2, results.bestPoint.Ki3, results.bestPoint.Ki4, results.bestPoint.Ki5, results.bestPoint.Ki6, results.bestPoint.Ki7]);
Kd = diag([results.bestPoint.Kd1, results.bestPoint.Kd2, results.bestPoint.Kd3, results.bestPoint.Kd4, results.bestPoint.Kd5, results.bestPoint.Kd6, results.bestPoint.Kd7]);

tau_max = sat_var.*[87, 87, 87, 87, 12, 12, 12];

t=time;
f = const.Robot_friction;

B            = zeros(n_DoFs,n_DoFs,length(t));
g            = zeros(length(t),n_DoFs);
tau_l        = zeros(length(t),n_DoFs);
q_msr        = zeros(length(t),n_DoFs);
qerr         = zeros(length(t),n_DoFs); 
dqerr        = zeros(length(t),n_DoFs); 
ddqerr       = zeros(length(t),n_DoFs); 
dq_msr       = zeros(length(t),n_DoFs);
ddq_msr      = zeros(length(t),n_DoFs);
tau_PID      = zeros(length(t),n_DoFs);
tau_comp     = zeros(length(t),n_DoFs);

q_msr(1,:) = q_r(1,:); 

B(:,:,1)  = Robot.inertia(q_r(1,:)); 
g(1,:)    = Robot.gravload(q_r(1,:)); 
B_eval(:,:,1) = Robot_eval.inertia(q_r(1,:));
g_eval(1,:)   = Robot_eval.gravload(q_r(1,:)); 

wb = waitbar(0,'Please wait...');

ierr_m(1,:) = [0 0 0 0 0 0 0];
dierr_m(1,:) = [0 0 0 0 0 0 0];

jj = 1;

while jj<length(t)  
    
    jj=jj+1; 
   
    B(:,:,jj)  = Robot.inertia(q_msr(jj-1,:)); 
    g(jj,:)    = Robot.gravload(q_msr(jj-1,:)); 
    
    B_eval(:,:,jj) = Robot_eval.inertia(q_msr(jj-1,:));
    g_eval(jj,:)   = Robot_eval.gravload(q_msr(jj-1,:));
    
    tau_l(jj,:)    = f*dq_msr(jj-1,:)' + g(jj,:)' ; 
    tau_PID(jj,:)  = B_eval(:,:,jj)*(Kp * (q_r(jj,:) - q_msr(jj-1,:))' + Kd * (dq_r(jj,:)  - dq_msr(jj-1,:))' + Ki * ierr_m(jj-1,:)'); 
    tau_comp(jj,:) = f*dq_msr(jj-1,:)' + g_eval(jj,:)';
    
    ddq_msr(jj,:) = B(:,:,jj)\( - tau_l(jj,:)' + (max(min(tau_PID(jj,:) + tau_comp(jj,:), tau_max), -tau_max))');  
    dq_msr(jj,:) = dq_msr(jj-1,:) + ddq_msr(jj,:)*Ts;
    q_msr(jj,:) = q_msr(jj-1,:) + dq_msr(jj,:)*Ts;
    
    ierr_m(jj,:) = ierr_m(jj-1,:) + (abs(tau_PID(jj,:)+tau_comp(jj,:))<tau_max) .* (q_r(jj,:)-q_msr(jj,:))*Ts; % Antiwindup
    dierr_m(jj,:) = dierr_m(jj-1,:) + abs(dq_r(jj,:) - dq_msr(jj,:)) * Ts; 

    qerr(jj,:)   = q_r(jj,:) - q_msr(jj,:);
    dqerr(jj,:)  = dq_r(jj,:) - dq_msr(jj,:);
    ddqerr(jj,:) = ddq_r(jj,:) - ddq_msr(jj,:);
    
    waitbar(jj/length(t),wb);
 
end

close(wb);
 
%% PLOT

% Robot trajectories

figure 
for i = 1:7
    subplot(7,1,i)
    plot(t, q_r(:,i)* 180/pi, 'LineWidth', 1.2)
    hold on
    plot(t, q_msr(:,i)* 180/pi, '--', 'LineWidth', 1.2)
    ylabel(['Joint ' num2str(i)])
    if i == 1
        title('Reference vs Measured trajectories [deg]')
    end
    if i == 7
        xlabel('Time [s]')
    end
    legend(['q ref' num2str(i)], ['q msr' num2str(i)])
    grid on
end

% Robot velocities

figure 
for i = 1:7
    subplot(7,1,i)
    plot(t, dq_r(:,i)* 180/pi, 'LineWidth', 1.2)
    hold on
    plot(t, dq_msr(:,i)* 180/pi, '--', 'LineWidth', 1.2)
    ylabel(['Joint ' num2str(i)])
    if i == 1
        title('Reference vs Measured velocity profiles [deg/s]')
    end
    if i == 7
        xlabel('Time [s]')
    end
    legend(['dq ref' num2str(i)], ['dq msr' num2str(i)])
    grid on
end

% Robot torques

figure
if sat_var==1
    for i=1:7
        subplot(7,1,i)
        plot(time, max(min(tau_PID(:,i)+tau_comp(:,i),tau_max(i)),-tau_max(i)))
        if i <= 4
            ylim([-95 95]);
        else
            ylim([-15 15]);
        end
        if i == 1
            title('Motor torques [Nm]')
        end
        if i == 7
            xlabel('Time [s]')
        end
        title('Saturated torques of the joints')
        grid on
    end
else
    for i=1:7
        subplot(7,1,i)
        plot(time, tau_PID(:,i)+tau_comp(:,i))
        if i == 1
            title('Motor torques [Nm]')
        end
        if i == 7
            xlabel('Time [s]')
        end
        title('Torques of the joints')
        grid on
    end
end


%% Metrics

% Positional metrics

RMS_pos = rms(qerr);
mean_pos_abs = mean(abs(qerr));
mean_pos = mean(qerr);
max_pos = max(abs(qerr));
var_pos_abs = var(abs(qerr));
var_pos = var(qerr);

% Velocity metrics

RMS_vel = rms(dqerr);
mean_vel_abs = mean(abs(dqerr));
mean_vel = mean(dqerr);
max_vel = max(abs(dqerr));
var_vel_abs = var(abs(dqerr));
var_vel = var(dqerr);


%% Save results 

if sat_var == 1
    sat_str = 'sat';
else
    sat_str = 'nosat';
end
if rampa_var == 1
    rampa_str = 'rmp';
else
    rampa_str = 'normp';
end
file_name_exploit = sprintf('results_exploit_%d_%d_%d_%d_%d_%d_%d_%d_%s_%s_aw.mat', ...
     exploration_iters, exploitation_iters, w_pos_max, w_vel_max, w_pos_avg, w_vel_avg, w_jerk_mean,w_tau_max, sat_str,rampa_str); 
file_name_explore = sprintf('results_explore_%d_%d_%d_%d_%d_%d_%d_%d_%s_%s_aw.mat', ...
     exploration_iters, exploitation_iters, w_pos_max, w_vel_max, w_pos_avg, w_vel_avg, w_jerk_mean,w_tau_max, sat_str,rampa_str); 


if isfile(file_name_exploit)
    disp('⚠️  Cambia il nome: il file esiste già.');
    error('Simulazione interrotta per evitare sovrascrittura.');
end

save(file_name_exploit, 'results');
save(file_name_explore, 'results_explore');