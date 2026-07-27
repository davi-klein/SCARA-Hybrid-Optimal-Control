function post_process_results(optimal_gains, best_cost, optimization_time, output)
% POST_PROCESS_RESULTS plots results and saves the optimized gains
% in a .mat file
%
% Args:
%   optimal_gains (array): optimal PSO gains
%   best_cost (float): final cost value
%   optimization_time (float): total elapsed time for PSO
%   output (struct): PSO output structure with iteration details

    disp('--------- Processing Results ---------')
    
    pso_results = struct();
    pso_results.Kp_x = optimal_gains(1);
    pso_results.Kp_y = optimal_gains(2);
    pso_results.Kd_x = optimal_gains(3);
    pso_results.Kd_y = optimal_gains(4);
    pso_results.Kp_f = optimal_gains(5);
    pso_results.Ki_f = optimal_gains(6);
    pso_results.Kd_f = optimal_gains(7);
    pso_results.best_cost = best_cost;
    pso_results.optimization_time = optimization_time;
    pso_results.iterations = output.iterations;
    pso_results.funcCount = output.funccount;
    
    fprintf('\nOptimal Gains:\n');
    fprintf('Kp_x  = %.2f\n', pso_results.Kp_x);
    fprintf('Kp_y  = %.2f\n', pso_results.Kp_y);
    fprintf('Kd_x  = %.2f\n', pso_results.Kd_x);
    fprintf('Kd_y  = %.2f\n', pso_results.Kd_y);
    fprintf('Kp_f  = %.2f\n', pso_results.Kp_f);
    fprintf('Ki_f  = %.2f\n', pso_results.Ki_f);
    fprintf('Kad_f = %.2f\n', pso_results.Kd_f);
    
    timestamp = char(datetime('now', 'Format', 'yyyy_MM_dd_HHmm'));
    filename = sprintf('data/optimal_gains_%s.mat', timestamp); 
    save(filename, 'pso_results');
    
    fprintf('\nResults successfully saved to: %s\n', filename);
    disp('-------------------------------');
end