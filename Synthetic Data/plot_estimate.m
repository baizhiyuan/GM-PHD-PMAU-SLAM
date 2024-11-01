function plot_estimate(obj,est,y,k,sim,params)
    % This function illustrates the scenario and estimate at time k 

    % Input:
    %    obj    - a (1 x N) struct that represent the PHD-SLAM density at time k
    %    est    - a struct that represent the vehicle and map estimate at k
    %    y      - a (2 x m_k) matrix that contains the measurements
    %    k      - time step
    %    sim    - struct containing the simulation data
    %    params - simulation parameters
    %
    % Author   : Ossi Kaltiokallio
    %            Tampere University, Department of Electronics and
    %            Communications Engineering
    %            Korkeakoulunkatu 1, 33720 Tampere
    %            ossi.kaltiokallio@tuni.fi
    % Last Rev : 1/9/2022
    % Tested   : '9.8.0.1359463 (R2020a) Update 1'
    %
    % Copyright notice: You are free to modify, extend and distribute 
    %    this code granted that the author of the original code is 
    %    mentioned as the original author of the code.
    
    % illustrates the scenario and estimate only if f_mode flag is true
    if params.f_mode
        persistent FoV;
        persistent h;
        persistent ax;

        if isempty(FoV)
            % initialize FOV, figure and handles to the axis and plots
            theta_pi = linspace(-params.fov_angle,params.fov_angle,180);
            FoV = [[cos(pi); sin(pi)] params.fov_range.*[cos(theta_pi); sin(theta_pi)]];
            h = [];

            figure(1); clf; box on; hold on; grid on;
            ax = gca;
        end

        % illustrate 1st and every 10th time instant to speed up visualization 
        if k == 1 || mod(k,10) == 0
            
            % project measurements to 2D Euclidean space
            if ~isempty(y)
                range = y(1,:);
                bearing = y(2,:);
            else
                range = nan;
                bearing = nan;
            end
            s = sin(est.m_hat(3)+bearing);
            c = cos(est.m_hat(3)+bearing);
            YtoX = [est.m_hat(1) + range.*c;
                est.m_hat(2) + range.*s];

            % get particles
            particles = [obj.xn];
            
            % FOV
            FoV_hat = repmat(est.m_hat(1:2),1,size(FoV,2)) + [cos(est.m_hat(3)) -sin(est.m_hat(3)); sin(est.m_hat(3)) cos(est.m_hat(3))]*FoV;
            str = sprintf('round: %d, dt: %.2f [ms]',k,est.dt*1000);

            try
                % update handles to plots
                set(h(2),'Xdata',est.mu_hat(1,est.active==1),'Ydata',est.mu_hat(2,est.active==1));
                if params.visualize_passive_components
                    set(h(3),'Xdata',est.mu_hat(1,est.active==0),'Ydata',est.mu_hat(2,est.active==0));
                end
                set(h(4),'Xdata',FoV_hat(1,:),'Ydata',FoV_hat(2,:));
                set(h(5),'Xdata',particles(1,:),'Ydata',particles(2,:));
                set(h(6),'Xdata',YtoX(1,:),'Ydata',YtoX(2,:));
                set(h(8),'Xdata',est.m_hat(1),'Ydata',est.m_hat(2));
                set(h(9),'Xdata',sim.state(1,k),'Ydata',sim.state(2,k));
                set(h(10),'String',str);

            catch
                % initialize handles to plots
                h(1) = plot(ax,sim.map(1,:),sim.map(2,:),'ks');
                
                % initialize map with dummy variables
                h(2) = plot(ax,0,0,'ro','linewidth',1,'markersize',10,'visible','on');
                if params.visualize_passive_components
                    h(3) = plot(ax,0,0,'rs','linewidth',1,'markersize',10,'visible','on');
                end

                h(4) = fill(ax,FoV_hat(1,:),FoV_hat(2,:),'r','FaceColor',[1 0 0],'EdgeColor',[1 0 0],'FaceAlpha',0.1);
                h(5) = plot(ax,particles(1,:),particles(2,:),'r+','markersize',12,'linewidth',0.5);
                h(6) = plot(ax,YtoX(1,:),YtoX(2,:),'k.','markersize',16,'linewidth',2);
                h(7) = plot(ax,sim.state(1,:),sim.state(2,:),'k','linewidth',2);
                h(8) = plot(ax,est.m_hat(1),est.m_hat(2),'rx','markersize',16,'linewidth',2);
                h(9) = plot(ax,sim.state(1,k),sim.state(2,k),'rs','markersize',16,'linewidth',2);
                h(10) = title(str,'FontName','Calibri Light'); %                 h(10) = title(str,'interpreter','latex');
                
                set(ax,'xlim',[200 1200],'ylim',[200 1200],'XLimMode','manual','YLimMode','manual')
                set(ax,'ticklabelinterpreter','latex','fontsize',16)
                xlabel('x / meter','interpreter','latex')
                ylabel('y / meter','interpreter','latex')
            end
            drawnow
        end
    end
end