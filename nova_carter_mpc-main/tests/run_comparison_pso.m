%% run_comparison.m — NMPC vs PID vs LQR vs PSO_MPC (v11 公平版)
%  ★ 所有控制器仅获得轨迹点位姿,无角速度前馈
%  ★ LQR 无 ω_ref → 弯道稳态误差, PSO-MPC自然处理


clear; clc; close all;
fprintf('====================================================================\n');
fprintf('  差速轮式机器人控制器对比 (公平版 v11)\n');
fprintf('====================================================================\n\n');


%% [0] 路径
ROOT_DIR = 'F:\论文\nova_carter_mpc-main';
addpath(genpath(ROOT_DIR));


%% [1] 仿真参数
dt=0.02;
T1=35; t1=0:dt:T1; N1=length(t1)-1;
T2=22; t2=0:dt:T2; N2=length(t2)-1;
T3=35; t3=0:dt:T3; N3=length(t3)-1;


%% [2] 参考轨迹
% 圆形 v=1.0m/s, R=5m
x_ref1=zeros(3,N1+50); Rc=5; v1=1.0; w1=v1/Rc;
for k=1:length(x_ref1)
    t=(k-1)*dt; x_ref1(:,k)=[Rc*sin(w1*t); Rc*(1-cos(w1*t)); w1*t];
end
% S形 v=1.2m/s, Ay=3, f=0.12Hz
x_ref2=zeros(3,N2+50); v2=1.2; Ay=3; fq=0.12;
for k=1:length(x_ref2)
    t=(k-1)*dt; dy=Ay*fq*2*pi*cos(fq*2*pi*t);
    x_ref2(:,k)=[v2*t; Ay*sin(fq*2*pi*t); atan2(dy,v2)];
end
% 8字形
x_ref3=zeros(3,N3+50); A8x=5; A8y=3; w8=0.18;
for k=1:length(x_ref3)
    t=(k-1)*dt;
    xp=A8x*w8*cos(w8*t); yp=2*A8y*w8*cos(2*w8*t);
    x_ref3(:,k)=[A8x*sin(w8*t); A8y*sin(2*w8*t); atan2(yp,xp)];
end


%% [3] 控制器参数
N_ctl=15; Qmat=diag([100,100,10]); Rmat=diag([0.1,0.1]);
Kp_v=2.0; Kp_h=4.0; Kp_c=1.5; Ki_h=0.05;
LQR_Q=diag([1,1,1]); LQR_R=diag([1,1]);
pso_swarm=20; pso_iter=30;


%% [4] 主仿真
set_names={'圆形(1.0m/s)','S形(1.2m/s)','8字形(变速)'};
ctrl_names={'NMPC','PID','LQR','PSO\_MPC'};
refs={x_ref1,x_ref2,x_ref3};
N_list=[N1,N2,N3]; T_list=[T1,T2,T3];
results=cell(3,4); times=cell(3,4);
%fprintf('[2/7] 运行 (4×3=12组)...\n\n');


for ti=1:3
    x_ref=refs{ti}; N_steps=N_list(ti);
    for ci=1:4
        fprintf('  [%d/12] %s_%s\n',(ti-1)*4+ci,set_names{ti},ctrl_names{ci});
        xk=zeros(3,1); x_hist=zeros(3,N_steps+1); x_hist(:,1)=xk;
        u_hist=zeros(2,N_steps); t_hist=zeros(N_steps,1);


        if ci==1 % NMPC
            N_c=10;
            u_guess=zeros(2,N_c);
            for k=1:N_steps
                tic;
                if k+N_c<=size(x_ref,2), seg=x_ref(:,k:k+N_c);
                else, seg=x_ref(:,end)*ones(1,N_c+1); end
                [u_opt,info]=nmpc_controller(xk,seg,dt,N_c,u_guess);
                t_hist(k)=toc*1000;
                u_hist(:,k)=min(max(u_opt(:,1),-20),20);
                xk=simple_dyn(xk,u_hist(:,k),dt); x_hist(:,k+1)=xk;
                if N_c>1, u_guess=[u_opt(:,2:end),u_opt(:,end)]; end
                if mod(k,100)==1||k==N_steps
                   % fprintf('    S%d Cost%.1f %.1fms\n',k,info.cost,t_hist(k));
                end
            end


        elseif ci==2 % PID
            v_ref_vals=[1.0,1.2,0.9]; v_des=v_ref_vals(ti); e_int_head=0;
            for k=1:N_steps
                ref_k=x_ref(:,min(k,size(x_ref,2)));
                e_pos=ref_k(1:2)-xk(1:2);
                e_head=atan2(sin(ref_k(3)-xk(3)),cos(ref_k(3)-xk(3)));
                cth=cos(xk(3)); sth=sin(xk(3));
                e_fwd=e_pos(1)*cth+e_pos(2)*sth; e_cross=-e_pos(1)*sth+e_pos(2)*cth;
                v_cmd=v_des*cos(e_head)+Kp_v*e_fwd; v_cmd=max(min(v_cmd,2.0),-1.0);
                e_int_head=e_int_head+e_head*dt; e_int_head=max(min(e_int_head,1.0),-1.0);
                w_cmd=Kp_h*e_head+Ki_h*e_int_head+Kp_c*e_cross; w_cmd=max(min(w_cmd,3.0),-3.0);
                r=0.1; L=0.28;
                u_cmd=[(v_cmd-w_cmd*L/2)/r;(v_cmd+w_cmd*L/2)/r];
                u_hist(:,k)=min(max(u_cmd,-20),20);
                xk=simple_dyn(xk,u_hist(:,k),dt); x_hist(:,k+1)=xk;
            end


        elseif ci==3 % LQR ★★★ 去掉 ω_ref 前馈 ★★★
            for k=1:N_steps
                ref_k=x_ref(:,min(k,size(x_ref,2)));
                if k>1&&k<N_steps
                    vn=norm(x_ref(1:2,k+1)-x_ref(1:2,k-1))/(2*dt);
                else
                    vn=norm(x_ref(1:2,2)-x_ref(1:2,1))/dt;
                end
                th=ref_k(3);
                Ac=[0,0,-vn*sin(th);0,0,vn*cos(th);0,0,0];
                Bc=[cos(th),0;sin(th),0;0,1];
                Ad=eye(3)+Ac*dt; Bd=Bc*dt;
                K_lqr=dlqr(Ad,Bd,LQR_Q,LQR_R);
                e=ref_k-xk; e(3)=atan2(sin(e(3)),cos(e(3)));
                du=K_lqr*e;
                v_cmd=vn+du(1);          % 保留 v_ref 前馈
                w_cmd=du(2);              % ★ 无 ω_ref！仅靠误差反馈
                v_cmd=max(min(v_cmd,2.0),-1.0); w_cmd=max(min(w_cmd,3.0),-3.0);
                r=0.1; L=0.28;
                u_cmd=[(v_cmd-w_cmd*L/2)/r;(v_cmd+w_cmd*L/2)/r];
                u_hist(:,k)=min(max(u_cmd,-20),20);
                xk=simple_dyn(xk,u_hist(:,k),dt); x_hist(:,k+1)=xk;
            end


        else % PSO_MPC (同v10)
            u_guess=zeros(2,N_ctl);
            for k=1:N_steps
                tic;
                if k+N_ctl<=size(x_ref,2), seg=x_ref(:,k:k+N_ctl);
                else, seg=x_ref(:,end)*ones(1,N_ctl+1); end
                [u_opt,info]=pso_mpc_controller(xk,seg,dt,N_ctl,u_guess,...
                    Qmat,Rmat,pso_swarm,pso_iter);
                t_hist(k)=toc*1000;
                u_hist(:,k)=min(max(u_opt(:,1),-20),20);
                xk=simple_dyn(xk,u_hist(:,k),dt); x_hist(:,k+1)=xk;
                if N_ctl>1, u_guess=[u_opt(:,2:end),u_opt(:,end)]; end
                if mod(k,50)==1||k==N_steps
                  %  fprintf('    S%d Cost%.1f %.0fms\n',k,info.cost,t_hist(k));
                end
            end
        end


        results{ti,ci}=x_hist; times{ti,ci}=t_hist;
       % fprintf('    ✓ %.1fms/step\n\n',mean(t_hist(t_hist>0)));
    end
end


%% [5] 绘图
fprintf('[3/7] 绘图...\n');
figure('Position',[50,50,1400,1000]);
colors={'b','r','g','m'};
for ti=1:3
    x_ref=refs{ti}; N_steps=N_list(ti);
    for ci=1:4
        subplot(4,3,(ci-1)*3+ti); hold on; grid on; axis equal;
        plot(x_ref(1,1:N_steps+1),x_ref(2,1:N_steps+1),'k--','LineWidth',1.2);
        xt=results{ti,ci};
        plot(xt(1,:),xt(2,:),'Color',colors{ci},'LineWidth',1.5);
        plot(xt(1,1),xt(2,1),'go','MarkerSize',8,'MarkerFaceColor','g');
        plot(xt(1,end),xt(2,end),'ro','MarkerSize',8,'MarkerFaceColor','r');
        xlabel('X(m)'); ylabel('Y(m)');
        if ti==2, title(sprintf('%s',ctrl_names{ci})); end
    end
end




for ti=1:3
    subplot(4,3,ti); title(sprintf('%s',set_names{ti}),'FontSize',12);
end
subplot(4,3,1); legend({'参考','实际','起点','终点'},'Location','best');


%% [6] RMSE
fprintf('[4/7] RMSE...\n');
rmse_pos=zeros(3,4); rmse_head=zeros(3,4);
for ti=1:3
    x_ref=refs{ti}; N_steps=N_list(ti);
    for ci=1:4
        xt=results{ti,ci}; n=min(size(xt,2),N_steps+1);
        err=xt(:,1:n)-x_ref(:,1:n); err(3,:)=atan2(sin(err(3,:)),cos(err(3,:)));
        rmse_pos(ti,ci)=sqrt(mean(err(1,:).^2+err(2,:).^2));
        rmse_head(ti,ci)=sqrt(mean(err(3,:).^2))*180/pi;
    end
end
figure('Position',[100,100,900,400]);
subplot(1,2,1); bar(rmse_pos);
set(gca,'XTickLabel',set_names,'FontSize',10);
ylabel('位置RMSE(m)'); title('位置跟踪误差对比(无ω前馈)');
legend(ctrl_names,'Location','best'); grid on;
subplot(1,2,2); bar(rmse_head);
set(gca,'XTickLabel',set_names,'FontSize',10);
ylabel('航向RMSE(deg)'); title('航向跟踪误差对比');
legend(ctrl_names,'Location','best'); grid on;


%% [7] 时域误差
fprintf('[5/7] 时域误差...\n');
ti=1; x_ref=refs{ti}; N_steps=N_list(ti); t_vec=0:dt:T_list(ti);
figure('Position',[100,100,900,600]);
for ci=1:4
    xt=results{ti,ci}; n=min(size(xt,2),N_steps+1);
    err=xt(:,1:n)-x_ref(:,1:n); err(3,:)=atan2(sin(err(3,:)),cos(err(3,:)));
    subplot(3,1,1); hold on; grid on;
    plot(t_vec(1:n),err(1,:),'Color',colors{ci},'LineWidth',1.2);
    ylabel('X误差(m)'); title('圆形跟踪误差(无ω前馈)');
    subplot(3,1,2); hold on; grid on;
    plot(t_vec(1:n),err(2,:),'Color',colors{ci},'LineWidth',1.2);
    ylabel('Y误差(m)');
    subplot(3,1,3); hold on; grid on;
    plot(t_vec(1:n),err(3,:)*180/pi,'Color',colors{ci},'LineWidth',1.2);
    ylabel('航向误差(deg)'); xlabel('时间(s)');
end
subplot(3,1,1); legend(ctrl_names,'Location','best');


%% [8] 计算时间
fprintf('[6/7] 计算时间...\n');
avg_times=zeros(3,4);
for ti=1:3
    for ci=1:4
        tt=times{ti,ci}; avg_times(ti,ci)=mean(tt(tt>0&tt<1000));
    end
end
figure('Position',[100,100,700,400]);
bar(avg_times);
set(gca,'XTickLabel',set_names,'FontSize',10);
ylabel('平均每步求解时间(ms)'); title('计算效率对比');
legend(ctrl_names,'Location','best'); grid on;


% %% [9] 表格
% fprintf('[7/7] 输出...\n\n');
% fprintf('====================================================\n');
% fprintf('  RMSE对比 (公平版: 无ω_ref前馈)\n');
% fprintf('====================================================\n');
% fprintf('  工况        | 控制器    | 位置RMSE(m) | 航向RMSE(deg)\n');
for ti=1:3
    for ci=1:4
        fprintf('  %-11s | %-8s |   %6.4f   |    %6.2f\n',...
            set_names{ti},ctrl_names{ci},rmse_pos(ti,ci),rmse_head(ti,ci));
    end
end
save('comparison_v11.mat','results','times','rmse_pos','rmse_head','avg_times');
fprintf('\n  完成!\n');






%% ==================== 辅助函数 ====================


function xn=simple_dyn(x,u,dt)
    r=0.1; L=0.28;
    v=r*(u(1)+u(2))/2; w=r*(u(2)-u(1))/L;
    xn=x+dt*[v*cos(x(3));v*sin(x(3));w];
    xn(3)=atan2(sin(xn(3)),cos(xn(3)));
end


function [u_opt,out]=nmpc_controller(x0,x_ref,dt,N,u_guess)
    r=0.1; L=0.28; Q=diag([100,100,10]); R=diag([0.1,0.1]);
    lb=repmat([-20;-20],N,1); ub=repmat([20;20],N,1);
    if nargin<5||isempty(u_guess), u0=zeros(2*N,1); else, u0=u_guess(:); end
    opt=optimoptions('fmincon','Algorithm','sqp','MaxIterations',100,...
        'MaxFunctionEvaluations',5000,'Display','off','StepTolerance',1e-6);
    [uv,fval,exitf,~]=fmincon(@(u)cost_fun(u,x0,x_ref,dt,N,Q,R,r,L),...
        u0,[],[],[],[],lb,ub,[],opt);
    u_opt=reshape(uv,2,N); out.cost=fval; out.exitflag=exitf;
end


function J=cost_fun(u_vec,x0,x_ref,dt,N,Q,R,r,L)
    u=reshape(u_vec,2,N); x=x0; J=0;
    for k=1:N
        v=r*(u(1,k)+u(2,k))/2; w=r*(u(2,k)-u(1,k))/L;
        xn=x+dt*[v*cos(x(3));v*sin(x(3));w];
        xn(3)=atan2(sin(xn(3)),cos(xn(3)));
        e=xn-x_ref(:,min(k+1,size(x_ref,2)));
        e(3)=atan2(sin(e(3)),cos(e(3)));
        J=J+e'*Q*e+u(:,k)'*R*u(:,k); x=xn;
    end
end


function [u_opt,out]=pso_mpc_controller(x0,x_ref,dt,N,u_guess,Q,R,n_swarm,max_iter)
    r=0.1; L=0.28; dim=2*N;
    lb=repmat([-20;-20],N,1); ub=repmat([20;20],N,1);
    if ~isempty(u_guess)&&length(u_guess)==dim
        center=u_guess;
    else
        center=zeros(dim,1);
    end
    span=0.15;
    particles=repmat(center,1,n_swarm)+(ub-lb).*(rand(dim,n_swarm)-0.5)*span;
    particles=max(min(particles,ub),lb);
    vel=span*(ub-lb).*(2*rand(dim,n_swarm)-1);
    pbest=particles; pbest_fit=inf(1,n_swarm);
    for i=1:n_swarm
        pbest_fit(i)=cost_fun(particles(:,i),x0,x_ref,dt,N,Q,R,r,L);
    end
    [gbest_fit,gi]=min(pbest_fit); gbest=pbest(:,gi);
    chi=0.729; c1=2.05; c2=2.05; v_max=0.5*(ub-lb);
    stall=0;
    for it=1:max_iter
        r1=rand(dim,n_swarm); r2=rand(dim,n_swarm);
        vel=chi*(vel+c1*r1.*(pbest-particles)+c2*r2.*(gbest-particles));
        vel=max(min(vel,v_max),-v_max);
        particles=particles+vel; particles=max(min(particles,ub),lb);
        for i=1:n_swarm
            fit=cost_fun(particles(:,i),x0,x_ref,dt,N,Q,R,r,L);
            if fit<pbest_fit(i), pbest_fit(i)=fit; pbest(:,i)=particles(:,i); end
        end
        old=gbest_fit;
        [mf,mi]=min(pbest_fit);
        if mf<gbest_fit, gbest_fit=mf; gbest=pbest(:,mi); end
        if abs(gbest_fit-old)<1e-8, stall=stall+1; else, stall=0; end
        if stall>=20
            n_r=max(1,round(0.3*n_swarm));
            idx=randperm(n_swarm,n_r);
            particles(:,idx)=repmat(gbest,1,n_r)+(ub-lb).*(rand(dim,n_r)-0.5)*0.1;
            particles=max(min(particles,ub),lb);
            stall=0;
        end
    end
    u_opt=reshape(gbest,2,N); out.cost=gbest_fit;
    out.exitflag=0; out.swarm_size=n_swarm; out.iterations=max_iter;
end
