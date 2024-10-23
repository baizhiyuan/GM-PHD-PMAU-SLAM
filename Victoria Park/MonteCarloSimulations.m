function MonteCarloSimulations
    % the function can be used to generate data of N simulations, run the 
    % MCSs and compute the performance metrics averaged over the N MCSs
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

    % set option = 1 to create data
    % set option = 2 to run MCS
    % set option = 3 to compute performance metrics
    
    option = 2;
    
    % number of simulations
    MCS = 100;
    
    switch option
        case 1
            generate_data(MCS)
        case 2
            fprintf('GM-PHD-PMAU-SLAM Performance:\n');
            perform_mcs(MCS,1);
            fprintf('LGM-PHD-PMAU-SLAM Performance:\n');
            perform_mcs(MCS,2);
            fprintf('PHD-SLAM 3.0 Performance:\n');
            perform_mcs(MCS,3);
        case 3
            compute_performance_metrics
        otherwise
            fprintf('Incorrect option\n')
    end
    
end


function generate_data(MCS)
    reply = input('Are you certain, you want to generate new measurements? (y/n): ','s');
    if strcmpi(reply,'y')
        fprintf('Generating data for Monte Carlo simulations.\n')
    else
        fprintf('No action taken.\n')
        return
    end

    % get current folder, define path name and add it to the path
    folderParts = strsplit(pwd, filesep);
    meas_path = strjoin([folderParts 'measurements' 'VP data set'], filesep);
    m_path = strjoin([folderParts(1:end-1) 'Shared Files' 'm source'], filesep);
    shared_path = strjoin([folderParts(1:end-1) 'Shared Files' 'shared m files'], filesep);
    
    % add measurement path to measurements and m files
    if ~contains(path,meas_path), addpath(meas_path); end
    if ~contains(path,m_path), addpath(m_path); end
    if ~contains(path,shared_path), addpath(shared_path); end

    % create data and save
    pw = PoolWaitbar(MCS, 'Creating data, please wait ...');
    for ii = 1:MCS
        clear('params','model','sim','obj')
        [params,sim,filename]  = simulation_setup(ii,'create');
        save(filename, 'params', 'sim')
        increment(pw)
    end
    delete(pw)
    
    % remove created paths
    rmpath(meas_path);
    rmpath(m_path);
    rmpath(shared_path);
end

function perform_mcs(MCS,algorithm)
    folderParts = strsplit(pwd, filesep);
    shared_m_path = strjoin([folderParts(1:end-1) 'Shared Files' 'shared m files'], filesep);
    if ~contains(path,shared_m_path), addpath(shared_m_path); end

    % create filename
    filepath  = 'results\';
    if strcmp(filesep,'\')
        switch algorithm
            case 1
                filename = strjoin([{filepath} 'GM_PHD_PMAU_N%d.mat'], filesep);
            case 2
                filename = strjoin([{filepath} 'LGM_PHD_PMAU_N%d.mat'], filesep);
            case 3
                filename = strjoin([{filepath} 'PHD3.0_N%d.mat'], filesep);
            otherwise
                fprintf('Incorrect option\n')
        end
    end
    
    % Particle number
    if algorithm == 1 || algorithm == 2
        N = 1;
    else
        N = [1 5 30]';
    end
    resample = true;        % resample flag
    for j = 1:size(N,1)
        POS_E = cell(MCS,1);
        CPU = cell(MCS,1);
        
        % If Parallel Computing Toolbox isn't installed, replace
        % parfor-loop with a regular for-loop.
        pw = PoolWaitbar(MCS, 'Simulation in progress, please wait ...');
        parfor ii = 1:MCS
            [POS_E{ii,1},CPU{ii,1}] = main(ii,false,N(j),resample,algorithm);
            increment(pw)
        end
        delete(pw)
        
        % save MCS results
        save(sprintf(filename,N(j)),'POS_E','CPU')
        print_performance_metrics(N(j),POS_E,CPU);
    end
end

function compute_performance_metrics   
    % create filename
    filepath  = 'results\';
    if strcmp(filesep,'\')
        filename1 = strjoin([{filepath} 'GM_PHD_PMAU_N%d.mat'], filesep);
        filename2 = strjoin([{filepath} 'LGM_PHD_PMAU_N%d.mat'], filesep);
        filename3 = strjoin([{filepath} 'PHD3.0_N%d.mat'], filesep);
    end

    N = [1 5 30]';    % Particle number
    try
        % load data
        fprintf('GM-PHD-PMAU-SLAM Performance:\n');
        load(sprintf(filename1,N(1)),'POS_E','CPU')
        print_performance_metrics(N(1),POS_E,CPU);
        
        fprintf('LGM-PHD-PMAU-SLAM Performance:\n');
        load(sprintf(filename2,N(1)),'POS_E','CPU')
        print_performance_metrics(N(1),POS_E,CPU);
    catch
        fprintf('Can''t load file, incorrect filename!\n')
    end
    fprintf('PHD-SLAM 3.0 Performance:\n');
    for j = 1:size(N,1)
        try
            % load data
            load(sprintf(filename3,N(j)),'POS_E','CPU')
            print_performance_metrics(N(j),POS_E,CPU);
        catch
            fprintf('Can''t load file, incorrect filename!\n')
        end
    end
end


function print_performance_metrics(N,POS_E,CPU)
    pos_e = cell2mat(POS_E);
    cpu = cell2mat(CPU);

    fprintf('N:%d, POS.=%.2f [m], CPU=%.2f [ms], TIME=%.2f [s]\n', ...
        N, ...
        sqrt(mean(pos_e.^2,'all','omitnan')), ...
        mean(cpu,'all')*1000, mean(sum(cpu,2)));
end