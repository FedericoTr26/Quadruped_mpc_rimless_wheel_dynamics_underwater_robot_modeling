function [t,EA,EAd,figHandle] = fig_animate(tout,Xout,Uout,Xdout,Udout,Uext,p)

flag_movie = p.flag_movie;
output_folder = fullfile(pwd, 'figures');
if ~isfolder(output_folder)
    mkdir(output_folder);
end

persistent saved_flag;
saved_flag = false;


if flag_movie
    
    try
        name = ['test.mp4'];
        vidfile = VideoWriter(name,'MPEG-4');
    catch ME
        name = ['test'];
        vidfile = VideoWriter(name,'Motion JPEG AVI');
    end
    open(vidfile);
end

%% smoothen for animation
t = (tout(1):p.simTimeStep:tout(end));
X = interp1(tout,Xout,t);
U = interp1(tout,Uout,t);
Xd = interp1(tout,Xdout,t);
Ud = interp1(tout,Udout,t);
Ue = interp1(tout,Uext,t);

%% loop through frames
figHandle=figure('Position',[200 100 1000 600]);
set(0, 'DefaultFigureRenderer', 'opengl');
set(gcf, 'Color', 'white')
N = 3;M = 3;    % subplot size

nt = length(t);
EA = zeros(nt,3);
EAd = zeros(nt,3);
for ii = 1:nt
    EA(ii,:) = fcn_X2EA(X(ii,:));
    EAd(ii,:) = fcn_X2EA(Xd(ii,:));
end

% for ZOH force
t2 = repelem(t,2);
t2(1) = []; t2(end+1) = t2(end);
U2 = repelem(U,2,1);

%%%%%% subfigure handles %%%%%%%
h_x = subplot(N,M,3);
h_dx = subplot(N,M,6);
h_w = subplot(N,M,9);
h_u = subplot(N,M,[7,8]);

for ii = 1:p.playSpeed:nt
    %% The main animation
    % plot setting
    pcom = X(ii,1:3)';
    h_main = subplot(N,M,[1,2,4,5]);
    hold on; grid on;axis square;axis equal;
    h_main.XLim = [pcom(1)-0.5 pcom(1)+0.5];
    h_main.YLim = [pcom(2)-0.5 pcom(2)+0.5];
    h_main.ZLim = [-0.2 0.6];
    
    viewPt = [0.2,0.5,0.2];
    view(viewPt);
    
    % plot robot & GRF
    % real
    fig_plot_robot(X(ii,:)',U(ii,:)',Ue(ii,:)',p)
    % desired
    fig_plot_robot_d(Xd(ii,:)',0*Ud(ii,:)',p)
    
    % text
    txt_time = ['t = ',num2str(t(ii),2),'s'];
    text(pcom(1),pcom(2),0.4,txt_time)
    txt_vd = ['vd = ',num2str(Xd(ii,4),2),'m/s'];
    text(pcom(1),pcom(2),0.5,txt_vd)
    txt_v = ['v = ',num2str(X(ii,4),2),'m/s'];
    text(pcom(1),pcom(2),0.45,txt_v)

    %% states
    %%%%%%%%% position %%%%%%%%%
    plot(h_x,t(1:ii),X(1:ii,1),'r',...
         t(1:ii),X(1:ii,2),'g',...
         t(1:ii),X(1:ii,3),'b',...
         t(1:ii),Xd(1:ii,1),'r--',...
         t(1:ii),Xd(1:ii,2),'g--',...
         t(1:ii),Xd(1:ii,3),'b--','linewidth',1)
    h_x.XLim = [t(1) t(end)];
    set( get(h_x,'Title'), 'String', 'Position [m]');

    %%%%%%%%% velocity %%%%%%%%%
    plot(h_dx,t(1:ii),X(1:ii,4),'r',...
         t(1:ii),X(1:ii,5),'g',...
         t(1:ii),X(1:ii,6),'b',...
         t(1:ii),Xd(1:ii,4),'r--',...
         t(1:ii),Xd(1:ii,5),'g--',...
         t(1:ii),Xd(1:ii,6),'b--','linewidth',1)
    h_dx.XLim = [t(1) t(end)];
    set( get(h_dx,'Title'), 'String', 'Velocity [m/s]');

    %%%%%%%%% Angular velocity %%%%%%%%% 
    plot(h_w,t(1:ii),X(1:ii,16),'r',...
         t(1:ii),X(1:ii,17),'g',...
         t(1:ii),X(1:ii,18),'b',...
         t(1:ii),Xd(1:ii,16),'r--',...
         t(1:ii),Xd(1:ii,17),'g--',...
         t(1:ii),Xd(1:ii,18),'b--','linewidth',1)
    h_w.XLim = [t(1) t(end)];
    set( get(h_w,'Title'), 'String', 'Angular velocity [rad/s]' );
    
    
    %% control
    plot(h_u,t2(1:2*ii),U2(1:2*ii,3),'r',...
         t2(1:2*ii),U2(1:2*ii,6),'g',...
         t2(1:2*ii),U2(1:2*ii,9),'b',...
         t2(1:2*ii),U2(1:2*ii,12),'k',...
         t(1:ii),Ud(1:ii,3),'r--',...
         t(1:ii),Ud(1:ii,6),'g--',...
         t(1:ii),Ud(1:ii,9),'b--',...
         t(1:ii),Ud(1:ii,12),'k--','linewidth',1)
    h_u.XLim = [t(1) t(end)];
    set( get(h_u,'Title'), 'String', 'Fz [N]' );
    
    %% make movie
    if flag_movie
        writeVideo(vidfile, getframe(gcf));
    end
    
    drawnow
 

 % Salva la figura pochi istanti prima della fine altrimenti il robot scompare
    if ii == 1
        t_target = t(end) - 0.03;
        [~, idx_capture] = min(abs(t - t_target));
    end
    
    if ~saved_flag && abs(ii - idx_capture) <= 1
    % Nome dinamico: final_configuration_1.pdf, _2.pdf, ...
    existing_files = dir(fullfile(output_folder, 'final_configuration_*.pdf'));
    next_idx = numel(existing_files) + 1;

    filename = sprintf('final_configuration_%d.pdf', next_idx);
    output_file = fullfile(output_folder, filename);

    drawnow;
    pause(0.1);
    print(gcf, output_file, '-dpdf', '-r300', '-bestfit');

    saved_flag = true;
    end




    if ii < nt
        cla(h_main);
    end
    
end

if flag_movie
    close(vidfile);
end

    





% === Salvataggio figura statica delle grandezze dinamiche ===
fig_all = figure('Position', [200 200 900 700]);


% subplot(4,1,1)
% plot(t, X(:,1), 'r', 'LineWidth', 1.5); hold on;
% plot(t, X(:,2), 'g', 'LineWidth', 1.5);
% plot(t, X(:,3), 'b', 'LineWidth', 1.5);
% plot(t, Xd(:,1), 'r--', 'LineWidth', 1.5);
% plot(t, Xd(:,2), 'g--', 'LineWidth', 1.5);
% plot(t, Xd(:,3), 'b--', 'LineWidth', 1.5);
% ylabel('[m]'); grid on; title('Position');


subplot(4,1,1)
plot(t, X(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, X(:,2), 'g', 'LineWidth', 1.5);
plot(t, X(:,3), 'b', 'LineWidth', 1.5);
plot(t, Xd(:,1), 'r--', 'LineWidth', 1.5);
plot(t, Xd(:,2), 'g--', 'LineWidth', 1.5);
plot(t, Xd(:,3), 'b--', 'LineWidth', 1.5);
ylabel('[m]'); grid on; title('Position');
legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'bestoutside');


% subplot(4,1,2)
% plot(t, X(:,4), 'r', 'LineWidth', 1.5); hold on;
% plot(t, X(:,5), 'g', 'LineWidth', 1.5);
% plot(t, X(:,6), 'b', 'LineWidth', 1.5);
% plot(t, Xd(:,4), 'r--', 'LineWidth', 1.5);
% plot(t, Xd(:,5), 'g--', 'LineWidth', 1.5);
% plot(t, Xd(:,6), 'b--', 'LineWidth', 1.5);
% ylabel('[m/s]'); grid on; title('Velocity');

subplot(4,1,2)
plot(t, X(:,4), 'r', 'LineWidth', 1.5); hold on;
plot(t, X(:,5), 'g', 'LineWidth', 1.5);
plot(t, X(:,6), 'b', 'LineWidth', 1.5);
plot(t, Xd(:,4), 'r--', 'LineWidth', 1.5);
plot(t, Xd(:,5), 'g--', 'LineWidth', 1.5);
plot(t, Xd(:,6), 'b--', 'LineWidth', 1.5);
ylabel('[m/s]'); grid on; title('Velocity');
legend('v_x', 'v_y', 'v_z', 'v_{x,d}', 'v_{y,d}', 'v_{z,d}', 'Location', 'bestoutside');


% subplot(4,1,3)
% plot(t, X(:,16), 'r', 'LineWidth', 1.5); hold on;
% plot(t, X(:,17), 'g', 'LineWidth', 1.5);
% plot(t, X(:,18), 'b', 'LineWidth', 1.5);
% plot(t, Xd(:,16), 'r--', 'LineWidth', 1.5);
% plot(t, Xd(:,17), 'g--', 'LineWidth', 1.5);
% plot(t, Xd(:,18), 'b--', 'LineWidth', 1.5);
% ylabel('[rad/s]'); grid on; title('Angular Velocity');

subplot(4,1,3)
plot(t, X(:,16), 'r', 'LineWidth', 1.5); hold on;
plot(t, X(:,17), 'g', 'LineWidth', 1.5);
plot(t, X(:,18), 'b', 'LineWidth', 1.5);
plot(t, Xd(:,16), 'r--', 'LineWidth', 1.5);
plot(t, Xd(:,17), 'g--', 'LineWidth', 1.5);
plot(t, Xd(:,18), 'b--', 'LineWidth', 1.5);
ylabel('[rad/s]'); grid on; title('Angular Velocity');
legend('\omega_x', '\omega_y', '\omega_z', '\omega_{x,d}', '\omega_{y,d}', '\omega_{z,d}', 'Location', 'bestoutside');


% subplot(4,1,4)
% plot(t2, U2(:,3), 'r', 'LineWidth', 1.5); hold on;
% plot(t2, U2(:,6), 'g', 'LineWidth', 1.5);
% plot(t2, U2(:,9), 'b', 'LineWidth', 1.5);
% plot(t2, U2(:,12), 'k', 'LineWidth', 1.5);
% plot(t, Ud(:,3), 'r--', 'LineWidth', 1.5);
% plot(t, Ud(:,6), 'g--', 'LineWidth', 1.5);
% plot(t, Ud(:,9), 'b--', 'LineWidth', 1.5);
% plot(t, Ud(:,12), 'k--', 'LineWidth', 1.5);
% ylabel('[N]'); xlabel('Time [s]'); grid on; title('Fz');

subplot(4,1,4)
plot(t2, U2(:,3), 'r', 'LineWidth', 1.5); hold on;
plot(t2, U2(:,6), 'g', 'LineWidth', 1.5);
plot(t2, U2(:,9), 'b', 'LineWidth', 1.5);
plot(t2, U2(:,12), 'k', 'LineWidth', 1.5);
plot(t, Ud(:,3), 'r--', 'LineWidth', 1.5);
plot(t, Ud(:,6), 'g--', 'LineWidth', 1.5);
plot(t, Ud(:,9), 'b--', 'LineWidth', 1.5);
plot(t, Ud(:,12), 'k--', 'LineWidth', 1.5);
ylabel('[N]'); xlabel('Time [s]'); grid on; title('Fz');
legend('Fz_1', 'Fz_2', 'Fz_3', 'Fz_4', 'Fz_{1,d}', 'Fz_{2,d}', 'Fz_{3,d}', 'Fz_{4,d}', 'Location', 'bestoutside');


% Salva con nome incrementale
existing_files = dir(fullfile(output_folder, 'summary_plot_*.eps'));
next_idx = numel(existing_files) + 1;
filename = sprintf('summary_plot_%d.eps', next_idx);
output_file = fullfile(output_folder, filename);
print(fig_all, output_file, '-depsc2', '-r600');


end




