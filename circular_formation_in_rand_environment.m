%% Balanced circular formation and random motion
%%
% For x and f: (1),(2),(3) -> x,y,theta circ car 1
%              (4),(5),(6) -> x,y,theta circ car 2, etc
%   x_all = [x  y  th  x  y  th ... x  y  th  x  y  th ... x  y  th]
%            <-------------(xc)-------> 3*Nc; <---(xr)-------> 3*Nc+3*Nr
%% Real dimensions:
%    - T in seconds
%    - Epsilon = 0.075 m
%    - dim1 = 1.2 m
%    - v_max = 0.13 m/s
%% TO BE NOTED:
% The total number of robots has to be greater than 1 for the function
% Dists() to work
%%
clear all
clc
rng('default'); % creates sequence of rand numbers
fprintf('Circular formation with random motion\n');

% Definition of parameters
% General parameters
epsilon = 0.075; % diameter of cars
dim1 = 1.2; dim2 = 1.2; % dim arena
theta = 0:pi/50:2*pi;
d_detect = epsilon/2 + 0.035; % min distance to start avoiding, measured from the center of the robot
% Parameters of the circular robots
Nc = 2; % number of circular cars
R = dim1/3; % radius of circular formation
G0 = [dim1/2 dim2/2]; % target (centre of circular formation)
k_v = 1; k_w = 1; % dimensionless control gains, they should be in [0,1]
v_max = 0.13; w_max = 4.85; % max linear and angular velocities
phi = -atan((k_w*w_max*R)/(k_v*v_max)); % ctant encoding radius
% Parameters of the random robots
Nr = 3; % number of circular robots
s = v_max*0.6; % speed random robots
lambd = 0.8; % turning rate

[G1,G2] = setTarget(G0,'static');

% Initialise everything that has random numbers
T = 100;  % Phi_r defined for longer T than needed
Phi_r = cell(Nr,1); % creates cell array vector
for i = 1:Nr
    Phi_r{i} = calculatePhi(T,lambd,'',w_max);
end
x0 = zeros(1,3*(Nc+Nr));
x0(1,1:3*Nc) = initialise_position_c(Nc,G1,G2,R);
x0(1,3*Nc+1:3*(Nc+Nr)) = initialise_position_r(Nr,dim1,dim2,'rand',x0(1,1:3*Nc),Nc,d_detect);

paint_initial_positions(Nr,Nc,epsilon,x0(1,3*Nc+1:3*(Nc+Nr)),x0(1,1:3*Nc),dim1,dim2,false,[],theta,d_detect)
%%
format long
T = 5;
tic
[t_all,x_all,F] = solveAll(x0,Nr,Nc,T,G1,G2,s,d_detect,dim1,dim2,epsilon,...
    k_v,k_w,v_max,w_max,phi,Phi_r);  
          % omega is sampled in [-w_max, w_max] (see function calculatePhi)    
toc

%% Select the number of points that we want to plot
t_steps = round(length(t_all));
delta_t = T/t_steps; % distance between time steps
[t_equi,xc,xr,G_print] = selectPoints(Nr,Nc,G1,G2,t_steps,t_all,x_all,delta_t); 

% Paint or show video
which_plot = 2; % change to 1, 2 or 3 according to the Figure we wish to plot
follow_G = false;
d = 0.2; % delay time of video
myColorOrder = [     0         0         1;  % blue
                     1         0         0;  % red
                     0         1         0;  % green
                     1       0.5       0.7;  % pink
                0.4940    0.2840    0.4560;	 % magenta fosc
                     0        0.5        0;  % dark green
               0.3010    0.7450    0.9330];  % rosa/blau claret
set(groot,'defaultAxesColorOrder',myColorOrder)

if which_plot==1
    paint_whole_trajectories(Nr,Nc,xr,xc,dim1,dim2,follow_G,G_print,T,d_detect,s,lambd,myColorOrder);
elseif which_plot==2
    show_video_trajectories(Nr,Nc,epsilon,xr,xc,d,dim1,dim2,follow_G,G_print,theta,t_steps,d_detect);   
elseif which_plot==3
    paint_initial_positions(Nr,Nc,epsilon,xr,xc,dim1,dim2,follow_G,G_print,theta,d_detect)
end

%% Plot velocities
x_equi = [xc xr];
sz = size(x_equi);
v_all_equi = zeros(sz(1),3*(Nc+Nr));
v_mod = zeros(sz(1),Nc+Nr);
w = zeros(sz(1),Nc+Nr);
for i=1:sz(1) % number of time steps
    v_all_equi(i,:) = F(t_equi(i),x_equi(i,:)');
    v_mod(i,:) = sqrt(v_all_equi(i,1:3:end).^2+v_all_equi(i,2:3:end).^2);
    w(i,:) = v_all_equi(i,3:3:end);
end

% module of linear velocity
plot_vc(t_equi,v_mod,'Module of linear velocity (m/s)',Nc,T,d_detect,s,lambd,v_max,phi)
plot_vr(t_equi,v_mod,'Module of linear velocity (m/s)',Nc,Nr,T,d_detect,s,lambd)
plot_vAll(t_equi,v_mod,'Module of linear velocity (m/s)',Nc,Nr,T,d_detect,s,lambd)
% angular velocity
plot_vc(t_equi,w,'Angular veloctiy (rad/s)',Nc,T,d_detect,s,lambd,v_max,phi)
plot_vr(t_equi,w,'Angular veloctiy (rad/s)',Nc,Nr,T,d_detect,s,lambd)
plot_vAll(t_equi,w,'Angular veloctiy (rad/s)',Nc,Nr,T,d_detect,s,lambd)

%% Plot histogram of random robots
M = 70; % M^2 grid
plot_freqs(M,count_cells_plot(M,xr,dim1,dim2,Nr),dim1,dim2) 


%% FUNCTIONS
% Solve ode
function [G1,G2] = setTarget(G0,tag)
    if strcmp(tag,'static') 
        G1 = @(t) G0(1); 
        G2 = @(t) G0(2); % static G
    elseif strcmp(tag,'linear')
        v_g = 0.01;
        G1 = @(t) G0(1) + v_g*t; 
        G2 = @(t) G0(2) + 0*t;
    else
        fprintf('Problem in setTarget\n');
    end
end
function [t_all,x_all,F] = solveAll(x0,Nr,Nc,T,G1,G2,s,d_detect,dim1,dim2,epsilon,k_v,k_w,v_max,w_max,phiR,Phi_r)

    fc = calc_fc(k_v,k_w,v_max,w_max,phiR,G1,G2,Nc); % fc(t,x,gamma,flag)
    fr = calc_fr(s,Nr,Nc,Phi_r); % fr(t,x,gamma,flag)

    F = @(t,x) calc_F(t,x,Nr,Nc,d_detect,epsilon,dim1,dim2,fc,fr,w_max);

    tspan = 0:0.1:T;
    fprintf('F created\n')
    
    opts = odeset('RelTol',1e-10,'AbsTol',1e-10,'MaxStep',epsilon/(15*v_max));
    [t_all,x_all] = ode45(F,tspan,x0,opts);
    
end
function [f_c] = calc_fc(k_v,k_w,v_max,w_max,phi,G1,G2,Nc)
    % x = x(i), y = x(i+1), theta = x(i+2)
    % (all)  x = x(1:3:end); y = x(2:3:end); theta = x(3:3:end);
    
    fprintf('Calling calc_fc\n');
    
    g = @(x,i) 1 - sum(cos(atan2(x(2:3:i+1-3) - x(i+1), x(1:3:i-3) - x(i)) - x(i+2)))...
        - sum(cos(atan2(x(i+1+3:3:3*Nc) - x(i+1), x(i+3:3:3*Nc) - x(i)) - x(i+2)));
            % 1 -  sum of cos(beta_ij) for all j<i  -  sum of cos(beta_ij) for all j>i
            
    vi = @(t,x,i) min(v_max, max(-v_max,...
        g(x,i)*k_v*v_max*cos(atan2(G2(t)- x(i+1), G1(t)- x(i)) - x(i+2) +phi)));
    wi = @(t,x,i) min(w_max, max(-w_max,...
        g(x,i)*k_w*w_max*sin(atan2(G2(t)- x(i+1), G1(t)- x(i)) - x(i+2) +phi)));
    
    Fp = @(t,x,i)[vi(t,x,i)*cos(x(i+2)),  vi(t,x,i)*sin(x(i+2)), wi(t,x,i)];

    f_c = @(t,x) cell2mat(arrayfun(@(i) Fp(t,x,i),1:3:3*Nc,'UniformOutput',0));
            % array of functions for all circ robots
    
end
function [f_r] = calc_fr(s,Nr,Nc,Phi_r)
    % x = x(i), y = x(i+1), theta = x(i+2)
    % (all)  x = x(1:3:end); y = x(2:3:end); theta = x(3:3:end);
      
    fprintf('Calling calc_fr\n');
    
    Fp = @(t,x,i)[s*cos(x(i+2)), s*sin(x(i+2)), Phi_r{(i-1)/3+1-Nc}(t)];
                      % 'plot_true' prints graph of phi 

    f_r = @(t,x) cell2mat(arrayfun(@(i) Fp(t,x,3*Nc+i),1:3:3*Nr,'UniformOutput',0)); 
            % array of functions for all rand robots (after the deterministic ones)

end
function P = calculatePhi(T,lambd,tag_plot,w_max)
    ts = 0;  % initial time
    
    fprintf('Calling calculatePhi\n');

    while max(ts) < T
        rs = rand(1);
        ts = [ts, ts(end)+(1/lambd)*log(1/rs)];
    end
	
    M = length(ts);
    %phis = 2*pi*rand(M,1);
    phis = -w_max + 2*w_max*rand(M,1);
    
    P = @(t) arrayfun(@(TS)Phi(TS,phis,ts),t);
    
    if strcmp(tag_plot,'plot_true')
        times = linspace(0,T,1e4);
        plot(times,P(times),'.','MarkerSize',9);
        hold on
        xlabel('t (s)', 'FontSize', 12);
        ylabel('\psi (rad/s)','Interpreter','tex', 'FontSize', 12);
        title(['T=',num2str(T),', lambd=',num2str(lambd),', Phi in [',num2str(-w_max*w_reduction),',',...
            num2str(w_max*w_reduction),']']);
        set(gca,'FontSize',12)
        hold off
    end
end
function phi = Phi(t,phis,ts)
    size(ts);
    size(phis);
    I = find(t - ts>=0,1,'last');
    phi = phis(I);
end
function [F] = calc_F(t,x,Nr,Nc,d_detect,epsilon,dim1,dim2,fc,fr,w_max)

    [gamma,flag] = Dists(x,Nr,Nc,dim1,dim2,epsilon,d_detect);
    F = [fc(t,x) fr(t,x)]';
    
    g1 = 10; g2 = 10;
    Hq = heav(cos(gamma),g2); % quadrant (heading)
    Hd = heav(d_detect-flag,g1); % distance
    F(1:3:end) = F(1:3:end).*(1-0.9.*Hq.*Hd);
    F(2:3:end) = F(2:3:end).*(1-0.9.*Hq.*Hd);
    ssk = -1 + heaviside(sin(-gamma))*2;
    F(3:3:end) = F(3:3:end) + Hq.*Hd.*(w_max.*ssk - F(3:3:end));
    
    t
end
function [bearing,flag] = Dists(x,Nr,Nc,dim1,dim2,epsilon,d_detect)
    bnds = [0,0,dim1,dim2]; % vector of domain boundaries
    rad = epsilon/2; % radius
    xs = x(1:3:end); ys = x(2:3:end); ths = x(3:3:end); % dim (Nr+Nc,1)
    
    D = squareform(pdist([xs ys])); % D(i,j) = dist from center of robot i to center of j for all i,j
    D = D - rad; % dist from center of robot i to surface of j
    D = D + diag(Inf*ones(Nc+Nr,1)); % set diagonal to Inf    
    db= ys'-bnds(2); dl= xs'-bnds(1); % Dist from center of robot to bndies; dim (1, Nr+Nc)
    dt= bnds(4)-ys'; dr= bnds(3)-xs';
    [min_dists,~] = min([D; db; dl; dt; dr]); % minimum distance (minimum of every colum)
    flag = min_dists';

    D2 = [D  db'.*0.8  dl'.*0.8  dt'.*0.8  dr'.*0.8];  % add distance to bndies to D (dim (Nc+Nr,Nc+Nr+4))
    M = (sum((1./(D2.^2))'))';  % dim (Nc+Nr,1)
    xdif = xs - repmat(xs',Nr+Nc,1); % dim (Nc+Nr,Nc+Nr)
    ydif = ys - repmat(ys',Nr+Nc,1); % dim (Nc+Nr,Nc+Nr)
    xdif = [xdif; zeros(1,Nc+Nr); -dl; zeros(1,Nc+Nr); dr]; % dim (Nc+Nr+4,Nc+Nr)
    ydif = [ydif; -db; zeros(1,Nc+Nr); dt; zeros(1,Nc+Nr)]; % dim (Nc+Nr+4,Nc+Nr)
    
    [I,J] = find(D2'<=d_detect+0.05);
    xdif_final = zeros(Nc+Nr+4,Nc+Nr);
    ydif_final = zeros(Nc+Nr+4,Nc+Nr);
    xdif_final(I,J) = xdif(I,J);
    ydif_final(I,J) = ydif(I,J);
    
    Rx = diag((1./(D2.^2))*xdif_final).*(1./M); % dim (Nc+Nr,1)
    Ry = diag((1./(D2.^2))*ydif_final).*(1./M);
    
    bearing = atan2(Ry,Rx) - ths(:);  % dim (Nc+Nr,1)
end
function x0 = initialise_position_c(Nc,G1,G2,R)
    x0 = zeros(1,3*Nc);
    for i=1:Nc
        x0(1,3*(i-1)+1) = G1(0) + R*cos(2*pi*(i-1)/Nc);
        x0(1,3*(i-1)+2) = G2(0) + R*sin(2*pi*(i-1)/Nc);
        x0(1,3*(i-1)+3) = 2*pi*(i-1)/Nc + pi/2;
    end
end
function x0 = initialise_position_r(Nr,dim1,dim2,tag,x0c,Nc,d_detect)
    if strcmp(tag,'rand')
        ep = 2*d_detect; % diameter of detection zone
        x = zeros(Nr,2); x0 = zeros(1,3*Nr);
        for i = 1:Nr
            x(i,1) = (dim1-2*ep/2)*rand+ep/2;
            x(i,2) = (dim2-2*ep/2)*rand+ep/2;
            m = 1:Nc;
            n_aux = 0;
            while (~isempty(m))  % check that it does not collide with any existing circ
                n_aux = n_aux + 1;
                if (sqrt((x(i,1)-x0c(1,3*(n_aux-1)+1))^2 + (x(i,2)-x0c(1,3*(n_aux-1)+2))^2) <= ep )
                    x(i,1) = (dim1-2*ep/2)*rand+ep/2;
                    x(i,2) = (dim2-2*ep/2)*rand+ep/2;
                    m = 1:Nc; % restart
                    n_aux = 0;
                end          
                if n_aux ~= 0 
                    m(Nc+1-n_aux)=[]; % we delete them in inverse order
                end
            end
            j = 1:i-1;
            k_aux = 0;
            while (~isempty(j)) % check that it does not collide with any existing rand
                k_aux = k_aux + 1;
                if (sqrt((x(i,1)-x(k_aux,1))^2 + (x(i,2)-x(k_aux,2))^2) <= ep )      
                    x(i,1) = (dim1-2*ep/2)*rand+ep/2;
                    x(i,2) = (dim2-2*ep/2)*rand+ep/2;
                    m = 1:Nc;
                    n_aux = 0;
                    while (~isempty(m))  % check that it does not collide with any existing circ
                        n_aux = n_aux + 1;
                        if (sqrt((x(i,1)-x0c(1,3*(n_aux-1)+1))^2 + (x(i,2)-x0c(1,3*(n_aux-1)+2))^2) <= ep )
                            x(i,1) = (dim1-2*ep/2)*rand+ep/2;
                            x(i,2) = (dim2-2*ep/2)*rand+ep/2;
                            m = 1:Nc; % restart
                            n_aux = 0;
                        end          
                        if n_aux ~= 0 
                            m(Nc+1-n_aux)=[]; 
                        end
                    end
                    j = 1:i-1;
                    k_aux = 0; 
                end
                if k_aux ~= 0 
                    j(i-k_aux)=[]; 
                end
            end
        end
        for i = 1:Nr
            x0(1,3*(i-1)+1) = x(i,1);
            x0(1,3*(i-1)+2) = x(i,2);
            x0(1,3*(i-1)+3) = 2*pi*rand;
        end
    elseif strcmp(tag,'chosen')
        if Nr == 4
            x0 = [0.15 1.05 2*pi*rand  0.15 0.3 2*pi*rand  0.9 0.9 2*pi*rand  0.75 0.3 2*pi*rand];
        else
            fprintf('Problem in initialise_position_r')
        end
    end
end

function [t_equi,xc,xr,G_print] = selectPoints(Nr,Nc,G1,G2,t_steps,t_all,x_all,delta_t)
    xc = zeros(t_steps,3*Nc); % select x of the equispaced time steps
    xr = zeros(t_steps,3*Nr); % select x of these equispaced time steps
    t_equi = zeros(t_steps,1);
    G_print = zeros(t_steps,2);
    j_aux = 1;
    t1 = t_all(j_aux);
    t_equi(1,1) = t1;
    xc(1,:) = x_all(j_aux,1:(3*Nc));
    xr(1,:) = x_all(j_aux,(3*Nc+1):(3*Nc+3*Nr));
    G_print(1,1) = G1(t1);
    G_print(1,2) = G2(t1);    
    for k=1:t_steps-1   
        while t1 < k*delta_t
            j_aux = j_aux+1;
            t1 = t_all(j_aux);
        end
        t_equi(k+1,1) = t1;
        xc(k+1,:) = x_all(j_aux,1:(3*Nc));
        xr(k+1,:) = x_all(j_aux,(3*Nc+1):(3*Nc+3*Nr));
        G_print(k+1,1) = G1(t1);
        G_print(k+1,2) = G2(t1);
     end
end
function h = heav(x,g)
    h = (tanh(g.*x)+1)./2;
end

% PAINT
% Plot/Video positions
function [] = paint_whole_trajectories(Nr,Nc,xr,xc,dim1,dim2,follow_G,G,T,d_detect,s,lambd,myColorOrder)
    dim = size(xr);
    if Nr ~= 0
        plot(xr(dim(1),1),xr(dim(1),2),'o','MarkerSize',16,... % paint first rand robot at its last position
            'MarkerFaceColor',[0.8 0.8 0.8],'MarkerEdgeColor','k','Color','k')  
    else
        plot(xc(dim(1),1),xc(dim(1),2),'o','MarkerSize',16,... % paint first circ robot at its last position
                'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','k','Color','k') 
    end
    axis([0 dim1 0 dim2]);
    hold on
    for i=2:Nr  % paint all other rand robots at its last position
        plot(xr(dim(1),3*(i-1)+1),xr(dim(1),3*(i-1)+2),'o','MarkerSize',16,...
            'MarkerFaceColor',[0.8 0.8 0.8],'MarkerEdgeColor','k','Color','k')
    end
    for i=1:Nr  % paint whole trajectory of all rand robots
        plot(xr(:,3*(i-1)+1),xr(:,3*(i-1)+2),'LineWidth',1.2); %'Color',myColorOrder(i,:)); %(only if Nr<7)
    end

    if Nr ~= 0
        for i=1:Nc  % paint all circ robots at its last position
            plot(xc(dim(1),3*(i-1)+1),xc(dim(1),3*(i-1)+2),'o','MarkerSize',16,...
                'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','k','Color','k')
        end
    else
        for i=2:Nc  % paint all other circ robots at its last position
            plot(xc(dim(1),3*(i-1)+1),xc(dim(1),3*(i-1)+2),'o','MarkerSize',16,...
                'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','k','Color','k')
        end
    end
    for i=1:Nc  % paint whole trajectory of all circ robots
        plot(xc(:,3*(i-1)+1),xc(:,3*(i-1)+2),'LineWidth',1.2);
    end
    
    if follow_G == true
        plot(G(:,1),G(:,2),'--k')
        sz = size(G);
        plot(G(sz(1),1), G(sz(1),2), 'xk','MarkerSize',10,'Color','k')
    end
    xlabel('x (m)', 'FontSize', 12);
    ylabel('y (m)', 'FontSize', 12);
    %legend("Robot 1", "Robot 2", 'Location', 'northwest');
    title(['T=',num2str(T),', ddetect=',num2str(d_detect),', s=',num2str(s),...
        ', lambd=',num2str(lambd)]);
    set(gca,'FontSize',12)
    hold off
end
function [] = show_video_trajectories(Nr,Nc,epsilon,xr,xc,d,dim1,dim2,follow_G,G,theta,t_steps,d_detect)
    dim = size(xr); % [number of rows, number of columns]
    for j=1:dim(1)
        if Nr ~= 0
            plot(xr(1:j,1),xr(1:j,2),'LineWidth',1.2) % trajectory of first rand robot
        else
            plot(xc(1:j,1),xc(1:j,2),'LineWidth',1.2) % trajectory of first circ robot
        end
        axis([0 dim1 0 dim2]);
        title(['Iteration =',num2str(j),'/',num2str(t_steps)]);
        hold on
        for i=2:Nr  % trajectories of all other rand robots
            plot(xr(1:j,3*(i-1)+1),xr(1:j,3*(i-1)+2),'LineWidth',1.2)
        end
        if Nr ~= 0
            for i=1:Nc  % trajectories of all circ robots
                plot(xc(1:j,3*(i-1)+1),xc(1:j,3*(i-1)+2),'LineWidth',1.2)
            end
        else
            for i=2:Nc  % trajectories of all circ robots
                plot(xc(1:j,3*(i-1)+1),xc(1:j,3*(i-1)+2),'LineWidth',1.2)
            end            
        end
        if follow_G == true  
            plot(G(1:j,1),G(1:j,2),'--k')  % Trajectory of target
            plot(G(j,1),G(j,2), 'xk','MarkerSize',10)  % Paint position of target
        end
        for i=1:Nr
            plot((epsilon/2)*cos(theta) + xr(j,3*(i-1)+1), ...  
                (epsilon/2)*sin(theta) + xr(j,3*(i-1)+2),'LineWidth',2,'Color',[0.8 0.8 0.8]);
                    % Paint actual rand robots as circles
            plot((epsilon/2)*cos(xr(j,3*(i-1)+3))+ xr(j,3*(i-1)+1), ...
                (epsilon/2)*sin(xr(j,3*(i-1)+3)) + xr(j,3*(i-1)+2), ...
                'p','MarkerSize',7,'MarkerFaceColor','y','Color','k')
                    % Mark direction to which the rand robot is looking
            plot(d_detect*cos(theta) + xr(j,3*(i-1)+1), ...  
                d_detect*sin(theta) + xr(j,3*(i-1)+2),'--','LineWidth',1,'Color',[0.6 0.6 0.6]);
                    % Paint d_detect
        end
        for i=1:Nc
            plot((epsilon/2)*cos(theta) + xc(j,3*(i-1)+1), ...  
                (epsilon/2)*sin(theta) + xc(j,3*(i-1)+2),'LineWidth',2,'Color','k');
                    % Paint actual circ robots as circles
            plot((epsilon/2)*cos(xc(j,3*(i-1)+3))+ xc(j,3*(i-1)+1), ...
                (epsilon/2)*sin(xc(j,3*(i-1)+3)) + xc(j,3*(i-1)+2), ...
                'p','MarkerSize',7,'MarkerFaceColor','y','Color','k')
                    % Mark direction to which the circ robot is looking
            plot(d_detect*cos(theta) + xc(j,3*(i-1)+1), ...  
                d_detect*sin(theta) + xc(j,3*(i-1)+2),'--','LineWidth',1,'Color',[0.6 0.6 0.6]);
                    % Paint d_detect
        end
        
        hold off
        img = frame2im(getframe); % Save as .gif
        [img,cmap] = rgb2ind(img,256);
        if j == 1    % imwrite ho guarda com a .gif a la carpeta on estic executant
            imwrite(img,cmap,'0_together.gif','gif','LoopCount',Inf,'DelayTime',d);
        else
            imwrite(img,cmap,'0_together.gif','gif','WriteMode','append','DelayTime',d);
        end
    end   
end
function [] = paint_initial_positions(Nr,Nc,epsilon,xr,xc,dim1,dim2,follow_G,G,theta,d_detect)
    if Nc ~= 0
        plot((epsilon/2)*cos(theta) + xc(1,1),(epsilon/2)*sin(theta) + xc(1,2),'LineWidth',2,'Color','k');
                    % Paint first circ robots as circle
    else
        plot((epsilon/2)*cos(theta) + xr(1,1),(epsilon/2)*sin(theta) + xr(1,2),'LineWidth',2,'Color',[0.8 0.8 0.8]);
                    % Paint first rand robots as circle
    end
    axis([0 dim1 0 dim2]);
    hold on
    if Nc ~= 0
        plot((epsilon/2)*cos(xc(1,3))+ xc(1,1),(epsilon/2)*sin(xc(1,3)) + xc(1,2),...
            'p','MarkerSize',7,'MarkerFaceColor','y','Color','k')
                    % Mark direction to which the first circ robot is looking
        plot(d_detect*cos(theta) + xc(1,1), ...
            d_detect*sin(theta) + xc(1,2),'--','LineWidth',1,'Color',[0.6 0.6 0.6]);
                    % Paint d_detect
    end
    for i=2:Nc
        plot((epsilon/2)*cos(theta) + xc(1,3*(i-1)+1),...
            (epsilon/2)*sin(theta) + xc(1,3*(i-1)+2),'LineWidth',2,'Color','k');
                    % Paint other circ robots as circles
        plot((epsilon/2)*cos(xc(1,3*(i-1)+3))+ xc(1,3*(i-1)+1), ...
            (epsilon/2)*sin(xc(1,3*(i-1)+3)) + xc(1,3*(i-1)+2), ...
            'p','MarkerSize',7,'MarkerFaceColor','y','Color','k')
                    % Mark direction to which the circ robot is looking
        plot(d_detect*cos(theta) + xc(1,3*(i-1)+1), ...  
            d_detect*sin(theta) + xc(1,3*(i-1)+2),'--','LineWidth',1,'Color',[0.6 0.6 0.6]);
                    % Paint d_detect
    end
    for i=1:Nr
        plot((epsilon/2)*cos(theta) + xr(1,3*(i-1)+1),...
            (epsilon/2)*sin(theta) + xr(1,3*(i-1)+2),'LineWidth',2,'Color',[0.8 0.8 0.8]);
                    % Paint all rand robots as circles
        plot((epsilon/2)*cos(xr(1,3*(i-1)+3))+ xr(1,3*(i-1)+1), ...
            (epsilon/2)*sin(xr(1,3*(i-1)+3)) + xr(1,3*(i-1)+2), ...
            'p','MarkerSize',7,'MarkerFaceColor','y','Color','k')
                    % Mark direction to which the rand robot is looking
        plot(d_detect*cos(theta) + xr(1,3*(i-1)+1), ...  
            d_detect*sin(theta) + xr(1,3*(i-1)+2),'--','LineWidth',1,'Color',[0.6 0.6 0.6]);
                    % Paint d_detect
    end
    if follow_G == true
        plot(G(:,1),G(:,2),'--k')
        s = size(G);
        plot(G(s(1),1), G(s(1),2), 'xk','MarkerSize',10)
    end
    xlabel('x/m', 'FontSize', 12);
    ylabel('y/m', 'FontSize', 12);
    %legend("Robot 1", "Robot 2", 'Location', 'northwest');
    set(gca,'FontSize',12)
    hold off
end
% Velocities
function [] = plot_vAll(t,v,tag,Nc,Nr,T,d_detect,s,lambd)

    plot(t,v(:,1),'LineWidth',1.2)  % first circ robot
    hold on
    for i=2:Nc  % all other circ robots
        plot(t,v(:,i),'LineWidth',1.2)
    end
    for i=1:Nr  % all rand robots
        plot(t,v(:,Nc+i),'--','LineWidth',1.2)
    end
    xlabel('Time (s)', 'FontSize', 12);
    ylabel(tag, 'FontSize', 12);
    %legend("Robot 1", "Robot 2", 'Location', 'northwest');
    title(['T=',num2str(T),', ddetect=',num2str(d_detect),', s=',num2str(s),...
        ', lambd=',num2str(lambd)]);
    set(gca,'FontSize',12)
    hold off
end
function [] = plot_vc(t,v,tag,Nc,T,d_detect,s,lambd,v_max,phi)
    sz = size(v);  vc = zeros(sz(1),1); % vc = mean velocity
    for i=1:sz(1)
        vc_aux = 0;
        for j=1:Nc
            vc_aux = vc_aux + v(i,j);
        end
        vc(i,1) = vc_aux/Nc;
    end
    
    if Nc>0
        plot(t,v(:,1),'LineWidth',1.2)  % first circ robot  
        hold on
        for i=2:Nc  % all other circ robots
            plot(t,v(:,i),'LineWidth',1.2)
        end
        %plot(t,vc(:,1),'LineWidth',1.2)  % mean of all robots
        %plot(t,v_max*sin(-phi)*ones(1,length(t)),'--','LineWidth',1); % expected v_f
    
        xlabel('Time (s)', 'FontSize', 12);
        ylabel(tag, 'FontSize', 12);
        %legend("Robot 1", "Robot 2", "Robot 3", "Expected final v", 'Location', 'southeast');
        %legend("Robot 1", "Robot 2", "Robot 3", 'Location', 'southeast');
        title(['T=',num2str(T),', ddetect=',num2str(d_detect),', s=',num2str(s),...
            ', lambd=',num2str(lambd)]);
        set(gca,'FontSize',12)
        hold off
    else
        fprintf('Nc = 0\n');
    end
end
function [] = plot_vr(t,v,tag,Nc,Nr,T,d_detect,s,lambd)
    sz = size(v);
    vr = zeros(sz(1),1); % vr = mean velocity
    for i=1:sz(1)
        vr_aux = 0;
        for j=(Nc+1):(Nc+Nr)
            vr_aux = vr_aux + v(i,j);
        end
        vr(i,1) = vr_aux/Nr;
    end
    
    if Nr>0
        plot(t,v(:,Nc+1),'LineWidth',1.2)  % first rand robot
        hold on
        for i=2:Nr  % all other rand robots
            plot(t,v(:,Nc+i),'LineWidth',1.2)
        end
        %plot(t,vr(:,1),'LineWidth',1.2) % mean of all rand robots
        
        xlabel('Time (s)', 'FontSize', 12);
        ylabel(tag, 'FontSize', 12);
        legend("Robot 1","Robot 2","Robot 3","Robot 4","Robot 5","Robot 6", 'Location', 'northwest');
        title(['T=',num2str(T),', ddetect=',num2str(d_detect),', s=',num2str(s),...
            ', lambd=',num2str(lambd)]);
        set(gca,'FontSize',12)
        hold off
    else
        fprintf('Nr = 0\n');
    end
end
% Histogram
function [cells_2d_plot] = count_cells_plot(M,xr,dim1,dim2,Nr)
    cells = zeros(M+1);
    X_grid = 0:dim1/M:dim1; % M+1
    Y_grid = 0:dim2/M:dim2; % M+1
    dim = size(xr);
    found = 1;
    for q=1:dim(1) % for each time step
        for m=1:Nr % for each robot
            for i=1:M % for each line of grid (x)
                if (xr(q,3*(m-1)+1)>=X_grid(i) && xr(q,3*(m-1)+1)<X_grid(i+1)) 
                    for j=1:M % for each line of grid (y)
                        if (xr(q,3*(m-1)+2)>=Y_grid(j) && xr(q,3*(m-1)+2)<Y_grid(j+1))
                            cells(i,j) = cells(i,j) + 1;
                            found = 0;
                            break;
                        end
                    end
                end
                if found == 0
                    found = 1;
                    break;
                end
            end
        end
    end
    cells_2d_plot = transpose(cells);
end
function [] = plot_freqs(M,cells,dim1,dim2)
    X_grid = 0:dim1/M:dim1; % M+1
    Y_grid = 0:dim2/M:dim2; % M+1
    figure;
    surf(X_grid,Y_grid,cells,'EdgeColor','None');
    colorbar;
    set(gca,'FontSize',12)
    view(2);
end





