function [qstar, error, exitflag] = optimizationFunction(robot, q, q_original, q_ikcon)
    
    % check if Optimization Toolbox exists, we need it
    if ~exist('fmincon')
        error('rtb:qmincon:nosupport', 'Optimization Toolbox required');
    end
    M = size(q,1);
    n = robot.n;
    
    qstar = zeros(M,n);
    error = zeros(M,1);
    exitflag = zeros(M,1);
    
    result_obj_f = zeros(M-1,1);
    
    opt = optimoptions('fmincon', ...
        'Algorithm', 'active-set', ...
        'Display', 'off');
    
    lb = robot.qlim(:,1);
    ub = robot.qlim(:,2);
    
    x_m = 0;
    
    qstar(1,:) = q_original(1,:);
    
    SDisplay = sprintf('OPTIMIZATION OPTIONS:');
    disp(SDisplay);
    SDisplay = sprintf('[0] - No optimization;');
    disp(SDisplay);
    SDisplay = sprintf('[1] - Minimize joint variation;');
    disp(SDisplay);
    SDisplay = sprintf('[2] - Maximize distance from joint limits;');
    disp(SDisplay);
    SDisplay = sprintf('[3] - Maximize manipulability;');
    disp(SDisplay);
    SDisplay = sprintf('[4] - All of the previous');
    disp(SDisplay);
    SDisplay = sprintf('[5] - Combo [1] & [2]');
    disp(SDisplay);
    SDisplay = sprintf('[6] - Combo [1] & [3]');
    disp(SDisplay);
    
    user_choice = input('Select the optimization plan:');
    
    if (user_choice ~= 0)
        
        w1 = 0;
        w2 = 0;
        w3 = 0;

        switch (user_choice)
            case 1
                w1 = 1;
                w2 = 0;
                w3 = 0;

            case 2
                w1 = 0;
                w2 = 1;
                w3 = 0;

            case 3
                w1 = 0;
                w2 = 0;
                w3 = 1;

            case 4
                w1 = 0.5;
                w2 = 0.4;
                w3 = 0.1;

            case 5
                w1 = 0.7;
                w2 = 0.3;
                w3 = 0;

            case 6
                w1 = 0.8;
                w2 = 0;
                w3 = 0.2;

        end 

        for m = 2:M
            q_m = q(m,:);
            q_m_original = q(m - 1,:);

            J = robot.jacobn(q(m,:));
            N = null(J);

            if isempty(N)
                error('rtb:qmincon:badarg', 'pHRIWARE:Robot is not redundant');
            end
            
            f = @(x) w1*(100/0.7249)*(sumsqr((N*x + q_m') - q_m_original'))+ w2*(100/1.9092)*(sumsqr((2*(N*x + q_m') - ub - lb)./(ub-lb))) - w3*(100/1.8652)*sqrt((det(robot.jacobn(N*x + q_m')*robot.jacobn(N*x + q_m')')));

            x0 = zeros(size(N,2), 1) + x_m;

            A = [N; -N];
            b = [ub-q_m'; q_m'-lb];

            [x_m, err_m, ef_m] = fmincon(f,x0,A,b,[],[],[],[],[],opt);

            qstar(m,:) = q(m,:) + (N*x_m)';
            error(m) = err_m;
            exitflag(m) = ef_m;
        end
    else
        qstar = q_ikcon;
    end
    
end

function s = sumsqr(A)
    s = sum(A.^2);
end