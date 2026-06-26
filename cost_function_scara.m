function J_total = cost_function_scara(particle)
    % Evaluates the objective function for the SCARA model.
    %
    % This function calculates the Integral of Time-multiplied Absolute Error 
    % (ITAE) for both planar tracking and vertical force regulation. It also 
    % enforces kinematic and dynamic hardware constraints using smooth 
    % quadratic penalties.
    %
    % Args:
    %     particle (1x6 array): Candidate gains [Kp_x, Kp_y, Kd_x, Kd_y, Kp_f, Ki_f].
    %
    % Returns:
    %     J_total (double): The aggregated scalar cost value.
    
    % 1. Simulation Setup
    simIn = Simulink.SimulationInput('SCARAHybCtrl');
    simIn = simIn.setModelParameter('Solver', 'ode15s');
    
    % Map particle array to diagonal gain matrices
    Kp_p_matrix = diag([particle(1), particle(2), 0.0]);
    Kd_p_matrix = diag([particle(3), particle(4), 0.0]);
    Kp_f_matrix = diag([0.0, 0.0, particle(5)]);
    Ki_f_matrix = diag([0.0, 0.0, particle(6)]);
    
    simIn = simIn.setVariable('Kp_p', Kp_p_matrix);
    simIn = simIn.setVariable('Kd_p', Kd_p_matrix);
    simIn = simIn.setVariable('Kp_f', Kp_f_matrix);
    simIn = simIn.setVariable('Ki_f', Ki_f_matrix);
    
    % 2. Execution
    try
        out = sim(simIn);
    catch
        % Catch integration failures (e.g., extreme stiffness from unstable gains)
        J_total = 1e6;
        return;
    end
    
    % 3. Data Extraction
    t = out.tout;
    ep = out.error_p; 
    ef = out.error_f; 
    tau = out.tau; 
    dq = out.dq;   
    ddq = out.ddq; 
    q = out.q;     
    
    % 4. Performance Evaluation (ITAE)
    geometric_error = sqrt(ep(:, 1).^2 + ep(:, 2).^2);
    force_error = abs(ef(:, 3));
    
    ITAE_pos = trapz(t, t .* geometric_error);
    ITAE_force = trapz(t, t .* force_error);
    
    w = 0.01; % Balancing factor mapping Newtons to Meters
    J_base = ITAE_pos + w * ITAE_force;
    
    % 5. Physical Constraints (Soft Quadratic Penalties)
    % Limits: [Joint 1, Joint 2, Joint 3]
    tau_max = [150.0, 80.0, 50.0];  % Nm and N
    dq_max  = [5.0, 5.0, 2.0];      % rad/s and m/s
    ddq_max = [20.0, 20.0, 10.0];   % rad/s^2 and m/s^2
    q_min   = [-pi, -pi, 0.0];      % rad and m
    q_max   = [ pi,  pi, 0.2];      % rad and m
    
    % Compute boundary violations
    tau_violation = sum(sum(max(0, abs(tau) - tau_max).^2));
    dq_violation  = sum(sum(max(0, abs(dq) - dq_max).^2));
    ddq_violation = sum(sum(max(0, abs(ddq) - ddq_max).^2));
    
    q_violation_lower = sum(sum(max(0, q_min - q).^2));
    q_violation_upper = sum(sum(max(0, q - q_max).^2));
    q_violation = q_violation_lower + q_violation_upper;
    
    % Penalty weighting
    W_penalty = 10000.0; 
    penalty_cost = W_penalty * (tau_violation + dq_violation + ddq_violation + q_violation);
    
    % 6. Total Cost Aggregation
    J_total = J_base + penalty_cost;
    
    if isnan(J_total) || isinf(J_total)
        J_total = 1e6;
    end
end