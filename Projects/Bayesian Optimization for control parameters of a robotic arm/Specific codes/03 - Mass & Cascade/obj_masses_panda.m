function J_tot = obj_masses_panda(masses_vars, const)

%% Count the number of function evaluations 

persistent idx_sim_masses;
if isempty(idx_sim_masses)
    idx_sim_masses = 1;
end

%% assign all constant values 

sim_var = const;
assert(size(masses_vars,1) == 1)
masses_vars_struct = table2struct(masses_vars(1,:));
fname = fieldnames(masses_vars_struct);
for idx_fn = 1:length(fname)
   sim_var.(fname{idx_fn}) = masses_vars_struct.(fname{idx_fn});
end

%% Simulation settings - inner loop and outer loop configuration %%

m1 = sim_var.m1;
m3 = sim_var.m3;
m5 = sim_var.m5;
m7 = sim_var.m7;
masses = [m1, 0, m3, 0, m5, 0, m7];

Robot_eval = panda_robot(masses);
    
%% Assigning constant values
    
toll_qerr = const.toll_qerr;
Ts = const.Ts;
t = const.time;
n_DoFs = const.n_DoFs;
Robot = const.Robot;
q_r = const.r.q_r;
ddq_r = const.r.ddq_r;
f = const.Robot_friction;
sat_var = const.sat_var;

%% Run the simulation

tau_max = sat_var.*[87, 87, 87, 87, 12, 12, 12];

g_eval = zeros(length(t),n_DoFs);
B_eval = zeros(n_DoFs,n_DoFs,length(t));

B            = zeros(n_DoFs,n_DoFs,length(t));
g            = zeros(length(t),n_DoFs);
tau_l        = zeros(length(t),n_DoFs);
q_msr        = zeros(length(t),n_DoFs);
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

jj = 1;
exit_flag = false;

penalty=0;  
penalty_prop=0;   
penalty_max=2000;

tau_max = sat_var.*[87, 87, 87, 87, 12, 12, 12];

while (jj<length(t) && exit_flag==false) 
    
    jj=jj+1; % instante jj
   
    B(:,:,jj)  = Robot.inertia(q_msr(jj-1,:)); 
    g(jj,:)    = Robot.gravload(q_msr(jj-1,:)); 
    B_eval(:,:,jj) = Robot_eval.inertia(q_msr(jj-1,:));
    g_eval(jj,:)   = Robot_eval.gravload(q_msr(jj-1,:));
    
    tau_l(jj,:)    = f*dq_msr(jj-1,:)' + g(jj,:)' ;  
    tau_comp(jj,:) = f*dq_msr(jj-1,:)' + g_eval(jj,:)';
    
    ddq_msr(jj,:) = B(:,:,jj)\( - tau_l(jj,:)' + (max(min(tau_PID(jj,:) + tau_comp(jj,:) + ddq_r(jj,:)*(B_eval(:,:,jj)'), tau_max), -tau_max))');
    dq_msr(jj,:) = dq_msr(jj-1,:) + ddq_msr(jj,:)*Ts;
    q_msr(jj,:) = q_msr(jj-1,:) + dq_msr(jj,:)*Ts;
    
    ddqerr(jj,:) = ddq_r(jj,:) - ddq_msr(jj,:);
    
    waitbar(jj/length(t),wb);
   
end

%% COST FUNCTION

J_tot = sum(max(abs(ddqerr))) + sum(mean(abs(ddqerr)));

close(wb)
    
fprintf('Function evaluation %.0f: final cost: %12.8f \n', idx_sim_masses, J_tot)
fprintf('-------------------------------\n')
    
idx_sim_masses = idx_sim_masses + 1;
    
end