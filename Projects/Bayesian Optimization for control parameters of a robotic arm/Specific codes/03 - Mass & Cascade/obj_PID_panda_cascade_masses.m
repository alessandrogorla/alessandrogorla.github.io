function J_tot = obj_PID_panda_cascade_masses(x_var, const, masses)

    
%% Count the number of function evaluations %%

persistent idx_sim;
if isempty(idx_sim)
    idx_sim = 1;
end

%% assign all constant values %%

sim_var = const;
current_dof = const.dof;

if current_dof ~= 0
    assert(size(x_var, 1) == 1, 'x_var must be a single row table.');
    x_var_struct = table2struct(x_var(1, :));
    fname = fieldnames(x_var_struct);
    for idx_fn = 1:length(fname)
        sim_var.(fname{idx_fn}) = x_var_struct.(fname{idx_fn});
    end
end

kp_already_computed = const.kpvect;
ki_already_computed = const.kivect;
kd_already_computed = const.kdvect;

%% Simulation settings - inner loop and outer loop configuration %%
   
Kp = zeros(const.n_DoFs);
Ki = zeros(const.n_DoFs);
Kd = zeros(const.n_DoFs);

if current_dof ~= 0

    fieldNameKp = ['Kp_' num2str(current_dof)];
    fieldNameKi = ['Ki_' num2str(current_dof)];
    fieldNameKd = ['Kd_' num2str(current_dof)];

    Kp(current_dof,current_dof) = sim_var.(fieldNameKp);
    Ki(current_dof,current_dof) = sim_var.(fieldNameKi);
    Kd(current_dof,current_dof) = sim_var.(fieldNameKd);
    
    for jj = 7:-1:current_dof+1
        idx = 7 - jj + 1; 
        Kp(jj,jj) = kp_already_computed(idx);
        Ki(jj,jj) = ki_already_computed(idx);
        Kd(jj,jj) = kd_already_computed(idx);
    end

else
    
    for jj = 1:7
        Kp(jj,jj) = kp_already_computed(jj);
        Ki(jj,jj) = ki_already_computed(jj);
        Kd(jj,jj) = kd_already_computed(jj);
    end

end

Robot_eval = panda_robot(masses);
    
%% Assign all constant values %%

toll_qerr = const.toll_qerr;
toll_qerr_min = const.toll_qerr_min;
Ts = const.Ts;
t = const.time;
n_DoFs = const.n_DoFs;
Robot = const.Robot;
q_r = const.r.q_r;
dq_r = const.r.dq_r;
ddq_r = const.r.ddq_r;
f = const.Robot_friction;
sat_var = const.sat_var;
w_pos_max = const.w_pos_max;
w_vel_max = const.w_vel_max;
w_pos_avg = const.w_pos_avg;
w_vel_avg = const.w_vel_avg;
w_jerk_mean = const.w_jerk_mean;   
w_tau_max = const.w_tau_max;

%% RUN THE SIMULATION

g_eval = zeros(length(t),n_DoFs);
B_eval = zeros(n_DoFs,n_DoFs,length(t));

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
exit_flag = false;

penalty=0;  
penalty_prop=0;   
penalty_max=2000;

tau_max = sat_var.*[87, 87, 87, 87, 12, 12, 12];

while (jj<length(t) && exit_flag==false) 
    
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

    % Penalty check

    % Threshold-based exit: max joint position error
    if any(abs(qerr(jj,:)) >= toll_qerr)
        disp(['Simulation stopped: position error exceeded at t = ', num2str(t(jj))]);
        exit_flag = true;
    end
    
    if exit_flag==true
         penalty = penalty_max;
    end

    waitbar(jj/length(t),wb);
      
end

if any(any(abs(qerr) > toll_qerr_min & abs(qerr) < toll_qerr)) & exit_flag==false
     norm = 2000/((toll_qerr-toll_qerr_min)*7);
     wrong_links = any(abs(qerr) > toll_qerr_min & abs(qerr) < toll_qerr);
     penalty_prop=norm*sum(wrong_links.*max(abs(qerr)-toll_qerr_min));
end

%% COST FUNCTION

freq = 1;
q_r_amp = 10*pi/180;
dq_r_amp = 2*pi*freq*10*pi/180;
dddq_r_amp = (2*pi*freq)^3*10*pi/180;
tau_norm=zeros(7,1);
tau_norm(1:4)=1/87;
tau_norm(5:7)=1/12;

qerr_norm  = qerr/q_r_amp;
dqerr_norm  = dqerr/dq_r_amp;
jerk_err = diff(ddqerr)/const.Ts;
jerk_err_norm = jerk_err/dddq_r_amp/10000;

J_pos_max = sum(max(abs(qerr_norm)));
J_vel_max = sum(max(abs(dqerr_norm)));
J_pos_avg = sum(mean(abs(qerr_norm)));
J_vel_avg = sum(mean(abs(dqerr_norm)));
J_jerk_mean = sum(mean(abs(jerk_err_norm)));  
J_tau_max = sum(max(abs(tau_PID+tau_comp)).*tau_norm');

J_tot = w_pos_max*J_pos_max+ w_vel_max*J_vel_max+ w_pos_avg*J_pos_avg+ w_vel_avg*J_vel_avg +w_jerk_mean*J_jerk_mean+w_tau_max*J_tau_max+penalty+penalty_prop; 

close(wb)
    
if exit_flag==false
    disp(['Weighted sum of average position errors: ', num2str(w_pos_avg*J_pos_avg)]);
    disp(['Weighted sum of maximum position errors: ', num2str(w_pos_max*J_pos_max) ]);
    disp(['Weighted sum of average velocity errors: ', num2str(w_vel_avg*J_vel_avg)]);
    disp(['Weighted sum of maximum velocity errors: ', num2str(w_vel_max*J_vel_max)]);
    disp(['Weighted control effort: ', num2str(w_tau_max*J_tau_max)]);
    disp(['Weighted sum of average jerk errors: ', num2str(w_jerk_mean*J_jerk_mean)]);
    disp(['Weighted sum of maximum control effort: ',num2str(max(abs(tau_PID+tau_comp)))]);
    disp(['Proportional penalty: ',num2str(penalty_prop)]);
end
if exit_flag==false
    disp(['Simulation has been interrupted since a penalty condition has been reached, penalty value: ',num2str(penalty)]);
end
if ~all(all((abs(tau_PID(:,:)+tau_comp(:,:))<tau_max)))
    disp('At least one motor saturated');
end

fprintf('Function evaluation %.0f: Final cost: %12.8f \n', idx_sim, J_tot)
fprintf('-------------------------------\n')
idx_sim = idx_sim + 1;
    
end