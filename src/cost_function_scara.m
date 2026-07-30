function J_total = cost_function_scara(particle)
    % Evaluates the objective function for the SCARA model.
    %
    % This function calculates the Integral of Time-multiplied Absolute Error 
    % (ITAE) for both planar tracking and vertical force regulation.
    %
    % Args:
    %     particle (1x7 array): Candidate gains [Kp_x, Kp_y, Kd_x, Kd_y, Kp_f, Ki_f, Kad_f].
    %
    % Returns:
    %     J_total (double): The aggregated scalar cost value.
    warning('off', 'all');
    
    task = getCurrentTask();
    
    if isempty(task)
        worker_id = 0; 
    else
        worker_id = task.ID; 
    end
    
    worker_dir = fullfile(pwd, sprintf('slprj_worker_%d', worker_id));
    if ~exist(worker_dir, 'dir')
        mkdir(worker_dir)
    end
    
    Simulink.fileGenControl('set', ...
        'CacheFolder', worker_dir, ...
        'CodeGenFolder', worker_dir, ...
        'createDir', true);
    
    evalin('base', 'init');
    
   
    simIn = Simulink.SimulationInput('SCARAHybCtrl');
    simIn = simIn.setModelParameter('SimulationMode', 'normal');
    simIn = simIn.setModelParameter('Solver', 'ode15s');
    simIn = simIn.setModelParameter('MaxStep', '0.01');
    simIn = simIn.setModelParameter('RelTol', '1e-3');
    simIn = simIn.setModelParameter('StopTime', '20');
    
    Kp_p_matrix = diag([particle(1), particle(2), 0.0]);
    Kd_p_matrix = diag([particle(3), particle(4), 0.0]);
    Kp_f_matrix = diag([0.0, 0.0, particle(5)]);
    Ki_f_matrix = diag([0.0, 0.0, particle(6)]);
    Kd_f_scalar = particle(7); 
    
    simIn = simIn.setVariable('Kp_p', Kp_p_matrix);
    simIn = simIn.setVariable('Kd_p', Kd_p_matrix);
    simIn = simIn.setVariable('Kp_f', Kp_f_matrix);
    simIn = simIn.setVariable('Ki_f', Ki_f_matrix);
    simIn = simIn.setVariable('Kd_f', Kd_f_scalar);
    
    try
        out = sim(simIn);
    catch ME
        J_total = 1e12;
        return;
    end
    
    t = out.tout;
    ep  = squeeze(out.error_p); 
    ef  = squeeze(out.error_f); 
    tau_cmd = squeeze(out.tau_cmd); 
    dq  = squeeze(out.dq);    
    q   = squeeze(out.q);
    
    if size(ep, 1) == 3
        ep  = ep'; ef  = ef'; tau_cmd = tau_cmd';
        dq  = dq'; q   = q';
    end
    detJ = squeeze(out.detJ);
    
    % 1. ITAE Cost Calculation
    geometric_error = sqrt(ep(:, 1).^2 + ep(:, 2).^2);
    force_error = abs(ef(:, 3));
    
    ITAE_pos = trapz(t, t .* geometric_error);
    ITAE_force = trapz(t, t .* force_error);

    ITAE_pos_nominal = 0.3505; 
    ITAE_force_nominal = 79.9975;
    
    % Normalized costs
    J_pos_norm = ITAE_pos / ITAE_pos_nominal;
    J_force_norm = ITAE_force / ITAE_force_nominal;
    
    w_pos = 0.5;
    w_force = 0.5;
    
    % BAse Cost
    J_base = (w_pos * J_pos_norm) + (w_force * J_force_norm);
    
    % 2. Limits and Constraints
    tau_max = [60.0, 30.0, 150.0];      
    dq_max  = [4.71, 7.50, 1.0];        
    q_min   = [-1.745, -2.443, -0.05];  
    q_max   = [1.745, 2.443, 0.2]; 

    peak_tau = max(abs(tau_cmd));
    peak_dq  = max(abs(dq));
    min_q    = min(q);
    max_q    = max(q);
    min_det  = min(abs(detJ));
    
    % 3. Smooth Penalty Gradients
    tau_violation = sum(max(0, peak_tau - tau_max).^2);
    dq_violation  = sum(max(0, peak_dq - dq_max).^2);
    
    q_violation_lower = sum(max(0, q_min - min_q).^2);
    q_violation_upper = sum(max(0, max_q - q_max).^2);
    q_violation = q_violation_lower + q_violation_upper;
    
    singularity_threshold = 0.001;
    singularity_violation = max(0, singularity_threshold - min_det)^2;
    
    W_penalty = 10000.0; 
    penalty_cost = W_penalty * (tau_violation + dq_violation + q_violation + singularity_violation);
    
    % 4. Total Aggregation
    J_total = J_base + penalty_cost;
    
    if isnan(J_total) || isinf(J_total)
        J_total = 1e12;
    end
end