function [obj,est] = gm_phd_pmau(obj,y,u,t,params,ukf)
    % This function performs one iteration of the PHD-SLAM filter recursion 
    % including prediction, computing the proposal, update, map, pruning, 
    % estimation

    % Input:
    %    obj    - a (1 x N) struct that represent the PHD-SLAM density at time k - 1
    %    y      - a (2 x m_k) matrix that contains the received measurements
    %    u      - a (2 x t_k) control input 
    %    t      - a (1 x t_k) time vector 
    %    params - simulation parameters
    %
    % Output:
    %    obj   - updated PHD-SLAM density at time k
    %    est   - a struct that contains the vehicle and map estimates
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
    
    tic
    iPrevParent = 0;

    % predict and make a temporary copy of the particle
    [obj_i, ukf] = pmau_predict(obj(1),u,t,params,ukf);

    % compute parameters of the proposal
    iParent = obj_i.iparent;
    if iParent == 0 || iParent ~= iPrevParent
        if params.J == 0 || size(obj_i.xl,2) == 0 || size(y,2) == 0
            % when there are no features nor measurements, use the
            % motion model as the sampling distribution
            wp = 0;
            mp = obj_i.xn;
            Pp = obj_i.Pn;
        else
            % compute cost matrix
            [P_D, L] = cost_matrix(obj_i, y, params);
            
            % solve ranked assignment problem
            [col4rowBest,~,~] = kBest2DAssign(-L,params.J,false,params.DA_threshold);
            
            % compute parameters of GM-ID approximation
            [wp,mp,Pp] = pmauiplid(obj_i,double(col4rowBest),y,log(P_D),params,ukf,t);
        end
        iPrevParent = iParent;
    end
    
    % CI fusion
    obj_i = CI_fusion(obj_i,wp,mp,Pp,params);

    % update PHD parameters
    obj_i = update(obj_i,y,params);

    % prune and merge PHD components, and store the particle in the 
    % PHD-SLAM density object
    obj(1) = hypothesisReductionAlgorithm(obj_i,params);

    % estimate and resample
    est = posterior_est(obj,params);

    est.dt = toc;
end