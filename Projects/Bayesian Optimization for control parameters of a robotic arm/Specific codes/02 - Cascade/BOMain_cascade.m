%% BAYESIAN OPTIMIZATION (cascade case)

%% Cleanup %%

clear;
close all;
clear obj_PID_panda_cascade
delete(findall(0, 'Type', 'figure', 'Tag', 'TMWWaitbar'));

addpath(genpath('../Necessary folders'));

%% Weights  definition %% 

w_pos_max = 50;
w_vel_max = 0;
w_pos_avg = 30;
w_vel_avg = 5;
w_jerk_mean = 1;   
w_tau_max = 0.02;

%% REQUESTS

project_case = 2;
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

bound_min_Kp_original = 0.;
bound_max_Kp_original = 10000;

bound_min_Kd_original = 0;
bound_max_Kd_original = 500;

bound_min_Ki_original = 0.;
bound_max_Ki_original = 1000.;

% Define bounds on the optimization variables first four joints

bound_max_Kp_new = 11000;
bound_max_Kd_new = 550;
bound_max_Ki_new = 1100;

best_vars = zeros(3,n_DoFs);

kpvect = [];
kivect = [];
kdvect = [];

%% Cascade BO

complete_results = cell(2, 7);

tic

for dof = 7:-1:1

    clear obj_PID_panda_cascade
    fprintf("\nOptimizing DOF %.0d\n\n", dof);
    const.dof = dof;
    opt_vars = [];

    const.kpvect = kpvect;
    const.kdvect = kdvect;
    const.kivect = kivect;
    
    if dof<5
        bound_min_Kp=bound_min_Kp_original; bound_max_Kp=bound_max_Kp_new;
        bound_min_Ki=bound_min_Ki_original; bound_max_Ki=bound_max_Ki_new;
        bound_min_Kd=bound_min_Kd_original; bound_max_Kd=bound_max_Kd_new;
    else
        bound_min_Kp=bound_min_Kp_original; bound_max_Kp=bound_max_Kp_original;
        bound_min_Ki=bound_min_Ki_original; bound_max_Ki=bound_max_Ki_original;
        bound_min_Kd=bound_min_Kd_original; bound_max_Kd=bound_max_Kd_original;
    end
    
    opt_vars = [opt_vars optimizableVariable(['Kp_' num2str(dof)], [bound_min_Kp, bound_max_Kp], 'Type','real')];
    opt_vars = [opt_vars optimizableVariable(['Ki_' num2str(dof)], [bound_min_Ki, bound_max_Ki], 'Type','real')];
    opt_vars = [opt_vars optimizableVariable(['Kd_' num2str(dof)], [bound_min_Kd, bound_max_Kd], 'Type','real')];
    
    %% FUNCTION DEFINITION
    
    func =  @(x_vars)(obj_PID_panda_cascade(x_vars,const));
    initial_X = []; 
    clear objBO; 
    
   %% FIRST PHASE: EXPLORATION

    results_explore = bayesopt(func, opt_vars, ...
        'Verbose', 0, ...
        'AcquisitionFunctionName', ...
        'lower-confidence-bound'	, ...
        'IsObjectiveDeterministic', true, ...
        'MaxObjectiveEvaluations', exploration_iters, ...
        'UseParallel', parallel_var, ...
        'MaxTime', inf, ...
        'NumSeedPoints', num_Seed_points, ...
        'GPActiveSetSize', 300 ,...
        'PlotFcn', []);

    %% SECOND PHASE: EXPLOITATION 
    
    results = bayesopt(func, opt_vars, ...
        'Verbose', 0, ...
        'AcquisitionFunctionName', ...
        'expected-improvement', ...
        'IsObjectiveDeterministic', true, ...
        'UseParallel', parallel_var, ...
        'MaxObjectiveEvaluations', exploitation_iters+exploration_iters, ...
        'MaxTime', inf, ...
        'InitialX', results_explore.XTrace, ...
        'InitialObjective', results_explore.ObjectiveTrace, ...
        'GPActiveSetSize', 300, ...
        'PlotFcn', []);

    %% Plot BO research

    N = length(results.ObjectiveTrace);
    idx_min = results.IndexOfMinimumTrace(end);
    
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
    title(sprintf('Objective function evolution plot (Dof = %d)', dof));
    legend('Current point', 'Current best point', 'Overall best point', 'Exploration vs exploitation');
    
    %% EXTRACT BEST VARIABLES

    best_vars(:,dof) = table2array(results.bestPoint);
    
    kpvect = best_vars(1, :);
    kivect = best_vars(2, :);
    kdvect = best_vars(3, :);

    complete_results{1,dof} = {results_explore};
    complete_results{2,dof} = {results};

end 

toc

clear obj_PID_panda_cascade

%% Evaluate performance of the optimal design

const.dof = 0;
const.kpvect = kpvect;
const.kivect = kivect;
const.kdvect = kdvect;
dummy_x_var = table();
opt_cost = obj_PID_panda_cascade(dummy_x_var, const);

%% OPTIMAL DYNAMIC SIMULATION

Robot_eval = panda_robot();

Kp = diag(kpvect);
Ki = diag(kivect);
Kd = diag(kdvect);

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
    
    tau_l(jj,:)    = f*dq_msr(jj-1,:)' + g(jj,:)'; 
    tau_PID(jj,:)  = B_eval(:,:,jj)*(Kp * (q_r(jj,:) - q_msr(jj-1,:))' + Kd * (dq_r(jj,:)- dq_msr(jj-1,:))' + Ki * ierr_m(jj-1,:)');
    tau_comp(jj,:) = f*dq_msr(jj-1,:)' + g_eval(jj,:)';
      
    ddq_msr(jj,:) = B(:,:,jj)\( - tau_l(jj,:)' + (max(min(tau_PID(jj,:) + tau_comp(jj,:), tau_max), -tau_max))');  %- tau_l(jj,:)+ tau_comp(jj,:) = 0 SE INERZIE SONO CONSTANTI
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

file_name = sprintf('complete_ results_%d_%d_%d_%d_%d_%d_%d_%d_%s_%s_aw.mat', ...
     exploration_iters, exploitation_iters, w_pos_max, w_vel_max, w_pos_avg, w_vel_avg, w_jerk_mean,w_tau_max, sat_str,rampa_str); 

if isfile(file_name)
    disp('⚠️  Cambia il nome: il file esiste già.');
    error('Simulazione interrotta per evitare sovrascrittura.');
end

save(file_name, 'complete_results');