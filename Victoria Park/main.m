function varargout = main(varargin)
    % function main excecutes a single Monte Carlo simulation of PHD-SLAM using
    % the Victoria Park data set
    %
    % Input:
    %    varargin{1}         - file identifier
    %    varargin{2}         - plot mode
    %    varargin{3}         - L, max number of OID iterations
    %    varargin{4}         - J, max number of GMM OID components
    %    varargin{5}         - N, particle number
    %    
    % Output:
    %    varargout{1}        - (1 X T) vector that containts the RMSE of vehicle for each time instant
    %    varargout{2}        - (1 x T) vector that containts the CPU time for each time instant
    %    varargout{3}        - (1 x T) vector that containts the N_eff for each time instant
    %
    % Original Author: Ossi Kaltiokallio
    %                  Tampere University, Department of Electronics and
    %                  Communications Engineering
    %                  Korkeakoulunkatu 1, 33720 Tampere
    %                  ossi.kaltiokallio@tuni.fi
    % Last Rev (Original): 29/9/2022
    %
    % Modified by : Zhiyuan Bai
    %               Hangzhou Dianzi University, Artificial Intelligence
    %               Xiasha Higher Education Zone, Hangzhou, 310018
    %               bai846@hdu.edu.cn
    % Last Rev     : 22/10/2024
    % Tested       : Matlab version 23.2.0.2365128 (R2023b)
    %
    % Copyright notice: You are free to modify, extend and distribute 
    %    this code granted that the author of the original code is 
    %    mentioned as the original author of the code.

    MEX = true;
     
    % load simulation parameters
    if nargin == 0
        file_idx = 1;
        [params,sim,~] = simulation_setup(file_idx,'load',MEX);
        maxNumCompThreads(1); % force maximum number of computation threads to one
        clear plot_estimate
    else
        [params,sim,~] = simulation_setup(varargin{1},'load',MEX);
        params.f_mode = varargin{2}; 
        params.N_particle = varargin{3};
        params.resample = varargin{4};
        if params.resample
            params.N_eff = 1;
        else
            params.N_eff = 0;
        end
    end 

    % initialize PHD-SLAM density and arrays to store data
    [obj,sim] = initialize(sim,params);
    ukf = init_ukf(3, 1e-4, 0, 2);

    % help variables
    k_last_control = 1;
    j = 1;
    t_prev = [];
    u_prev = [];
     
    % for each time instants, do
    for k = 1:params.K   
        if sim.o_time{j} < sim.u_time{k}
            % get current measurement
            y = sim.y{j};
           
            % get control input and control times
            t = [t_prev sim.u_time{k_last_control:k-1} sim.o_time{j}];
            u = [u_prev sim.odometry{k_last_control:k-1}];
            if k > 1
                t_prev = sim.o_time{j};
                u_prev = sim.odometry{k-1};
            end
            if nargin == 0
                [obj,est] = lgm_phd_pmau(obj,y,u,t,params,ukf);
            else
                switch varargin{5}
                    case 1
                        [obj,est] = gm_phd_pmau(obj,y,u,t,params,ukf);
                    case 2
                        [obj,est] = lgm_phd_pmau(obj,y,u,t,params,ukf);
                    case 3
                        [obj,est] = rbpf_phd(obj,y,u,t,params);
                    otherwise
                        fprintf('Incorrect algorithm\n')
                end
            end
            % store data for evaluation purposes
            sim.XX(:,:,j) = [obj.xn];
            sim.WW(:,j) = [obj.w];
            sim.MM(:,j) = est.m_hat;
            sim.PP(:,:,j) = est.P_hat;
            sim.MM_map{1,j} = est.mu_hat;
            sim.PP_map{1,j} = est.C_hat;
            sim.cpu_time(j) = est.dt;
            
            % illustrate 
            plot_estimate(obj,est,y,j,sim,params)
            
            k_last_control = k;
            j = j + 1;
        end

        if j > params.T
            break
        end
    end
    
    if nargin == 0
        params.f_mode = true;
    end
    % compute performance metrics
    pos_e = performance_summary(params,sim);

    if nargin > 0 
        varargout{1} = pos_e;
        varargout{2} = sim.cpu_time;
    end
    clear('params','sim','obj')
end