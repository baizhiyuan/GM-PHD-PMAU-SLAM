function [ukf] = init_ukf(L, alpha, ki, beta)
    % Initialize ukf parameters

    % Input:
    %    L      - dimensions of the vehicle states
    %    alpha  - ukf parameter
    %    ki     - ukf parameter
    %    beta   - ukf parameter
    %
    % Output:
    %    ukf    - ukf parameters
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

    ukf.L=L;
    ukf.alpha=alpha;
    ukf.ki=ki;
    ukf.beta=beta;
    ukf.lambda=ukf.alpha^2*(ukf.L+ukf.ki)-ukf.L;
    ukf.c=ukf.L+ukf.lambda;
    ukf.Wm=[ukf.lambda/ukf.c 0.5/ukf.c+zeros(1,2*ukf.L)];
    ukf.Wc=ukf.Wm;
    ukf.Wc(1)=ukf.Wc(1)+(1-ukf.alpha^2+ukf.beta);
    ukf.c=sqrt(ukf.c);
    ukf.X1=zeros(L,L*2 + 1);
    ukf.X2=zeros(L,L*2 + 1);
end