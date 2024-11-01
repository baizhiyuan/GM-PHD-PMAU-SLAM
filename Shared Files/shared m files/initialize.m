function [obj,sim] = initialize(sim,params)
    % Initialize PHD-SLAM density and arrays to store date

    % Input:
    %    sim    - struct containing the initial estimates
    %    params - simulation parameters
    %
    % Output:
    %    obj    - a (1 x N) struct that represent the PHD-SLAM density
    %    sim    - a struct that is used to store data
    %
    %    The fields of particle j for obj are:
    %    xn             - state of the vehicle
    %    Pn             - covariance of the vehicle
    %    w              - particle weight
    %    iParent        - index of the parent particle
    %    eta            - (1 x M) vector of landmark weights
    %    xl             - (2 x M) matrix of landmark states
    %    Pl             - (2 x 2 x M) matrix of landmark covariances
    %    label          - (3 x M) vector of landmark labels
    %    eta_p          - (1 x M) vector of landmark weights (outside FOV)
    %    xl_p           - (2 x M) matrix of landmark states (outside FOV)
    %    Pl_p           - (2 x 2 x M) matrix of landmark covariances (outside FOV)
    %    label_p        - (1 x M) vector of landmark labels (outside FOV)
    %    birth_y        - (2 x C) matrix containing measurements used to create new landmarks
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
    
    % obj is a (1 x N) struct that represent the PHD-SLAM density
    obj = repmat( ...
        struct('xn',sim.x0, ...
        'Pn',sim.P0, ...
        'w',log(1/params.N_particle), ...
        'iparent',1, ...
        'eta',[], ...
        'xl',[], ...
        'Pl',[], ...
        'label',[], ... 
        'label_p',[], ...
        'eta_p',[], ...
        'xl_p',[], ...
        'Pl_p',[], ...
        'birth_y',[]) ...
        ,1,params.N_particle);
    % initialize arrays to store data
    sim.XX = zeros(params.xn_dim,params.N_particle,params.T);
    sim.WW = zeros(params.N_particle,params.T);
    sim.MM = zeros(params.xn_dim,params.T);
    sim.PP = zeros(params.xn_dim,params.xn_dim,params.T);
    sim.MM_map = cell(1,params.T);
    sim.PP_map = cell(1,params.T);
    sim.cpu_time = zeros(1,params.T);
    sim.Neff = zeros(1,params.T);
    sim.resample = zeros(1,params.T);
end