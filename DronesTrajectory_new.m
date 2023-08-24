%GUI
function GUI()

W=inputdlg({'Radius of the scan area','Effective scanning radius of the UAV','Effective communication radius of the UAV','Set the number of scanning sectors','Time measurement','Outermost ring flight speed m/s','Number of target nodes','Total angular velocity scale','UAV battery energy','Information collection rounds','UAV starting position within the sector'},'Set of input parameters',1,{'1000','125','250','6','0.001','30','50','8000','100000','30','1'});
SL=str2double(W{1});%待扫描区域半径
SR=str2double(W{2});%无人机有效扫描半径
Sr=str2double(W{3});%无人机有效通讯半径
Sm=str2double(W{4});%设置扫描扇区数
St=str2double(W{5});%时间度量单位
V=str2double(W{6});%最外环飞行速度m/s
Node=str2double(W{7});%目标节点个数
cell=str2double(W{8});%角速度刻度
Jer=str2double(W{9});%无人机能量
turn=str2double(W{10});%信息收集轮次
pp=str2double(W{11});%无人机起始于扇区的位置（1表示无人机处于扇区中轴线）

if 2*SR < Sr %不同环间无人机覆盖有效扫描域之和小于有效通讯半径
    pr=2*SR;  %设置环间距为2*SR
else         %其他情况
    pr=Sr;    %设置环间距为Sr
end
avr=SL;%环形扫面区域最大半径
r =(avr-pr)/ceil((avr-pr)/pr);   %调整环间距为r
circle = ceil((avr-pr)/r)+1;   %环形路径层数
dcell = 2*pi/cell;   %个环角速度

i =1;
dr =[];
while i <=circle
    dr = [dr;pr/2+(i-1)*r];%设置每个环的航行半径
    i=i+1;
end
dread =[];
dhave=[];
time_each_turn=[];
TD=[];
DD=[];
ED=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
profile_drag_coefficient=0.012; %剖面阻力系数
air_dencity=1.225; %空气密度
rotor_solidity=0.05;%转子坚固性
A=0.503;%转子盘面积
blade_angular_velocity=300;%转子转速
rotor_radius=0.4;%转子半径
k=0.1;%修正参数
w=20;%机身重量，牛顿
V0=4.03;%滞空引导平均速度
Utip=120;%转子的最大转速
fuselage_drag_ratio=0.6;%机身阻力比
P0=(profile_drag_coefficient/8)*air_dencity*rotor_solidity*A*(blade_angular_velocity^3)*(rotor_radius^3);%drone悬停时的功率参数1
Pi=(1+k)*(w^1.5)/sqrt(2*air_dencity*A);%drone悬停时的功率参数2
Ph=P0+Pi;%drone悬停时的功率
Pv=P0*(1+3*(V^2)/(Utip^2))+Pi*V0/V+1/2*fuselage_drag_ratio*air_dencity*rotor_solidity*A*V^3;%环中drone以既定速度飞行时的功率公式%
%%Ps=P0*(1+3*((dr(i)/dr(circle))^2)/(Utip^2))+Pi*V0/(dr(i)/dr(circle))+1/2*fuselage_drag_ratio*air_dencity*rotor_solidity*A*(dr(i)/dr(circle))^3;%环中drone以既定速度飞行时的功率
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bandwidth=20*2^20;%带宽，单位赫兹
Wavelength=0.125;%波长，单位m
Pt=0.25;%无人机传送拍摄数据的功率，单位W
Noise_power=1*10^(-10);%噪声功率，单位m
Gt=1;%天线传输增益
Gr=1;%天线接收增益
Cinside=bandwidth*log2(1+((Wavelength/(4*pi*(pr/2)))^2)*(Gt*Gr*Pt/Noise_power));%内环传输速率
Coutside=bandwidth*log2(1+((Wavelength/(4*pi*(r)))^2)*(Gt*Gr*Pt/Noise_power));%外环传输速率
liuliang=10000*8*(2^10);%数据包大小，单位bit
tCinside=liuliang/Cinside;%内环传输时间，单位s
tCoutside=liuliang/Coutside;%外环传输时间，单位s

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pphoto=0.5;%无人机拍摄数据的功率，单位W
%PTransmit=0.5;%无人机传送拍摄数据的功率，单位W

% 定义每个环中drone以既定速度飞行时的功率
i =1;
dp =[];
while i <=circle
    dp = [dp;P0*(1+3*(((dr(i)/dr(circle))*V)^2)/(Utip^2))+Pi*((1+((((dr(i)/dr(circle))*V))^4)/(4*(V0)^4))^(1/2)-((((dr(i)/dr(circle))*V))^2)/(2*(V0)^2))^(1/2)+1/2*fuselage_drag_ratio*air_dencity*rotor_solidity*A*((dr(i)/dr(circle))*V)^3;];%每个环中drone以既定速度飞行时的功率
    i=i+1;
end

% X*(((2*pi*dr(circle)/V)/cell)/St)=Y;X是程序时间，Y是实际时间，最后可以换算到现实时间
S_St=((2*pi*dr(circle)/V)/cell);%程序中的St换算成实际的时间
% 程序中的St时间每个环drone飞行消耗的功率x1t，单位J
i =1;
St_P =[];
while i <=circle
    St_P = [St_P;dp(i)*S_St];
    i=i+1;
end
St_Ph=Ph*S_St;%程序中的St时间drone悬停时的功率x1t，单位J
St_Pphoto=Pphoto*S_St;%程序中的St时间无人机拍摄数据的功率x1t，单位J
St_PTransmit=Pt*S_St;%程序中的St时间无人机传送拍摄数据的功率x1t，单位J
tphoto=3;%拍摄数据倒计时s
%ttramsmit=0.5;%传输一个数据包倒计时s
St_tphoto=tphoto*St/S_St;%拍摄数据倒计时s
%St_ttramsmit=ttramsmit*St/S_St;%传输一个数据包倒计时s
St_ttCinside=tCinside*St/S_St;%内环传输一个数据包倒计时s
St_ttCoutside=tCoutside*St/S_St;%外环传输一个数据包倒计时s

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function do(~,~) %暂停按钮样式设置
        state = get(bt,'Value');
        if state
            set(bt,'String','>>');
            uiwait(Fig)
        else
            uiresume(Fig)
            set(bt,'String','||');
        end
    end

% 创建窗口
Fig =figure("name","Drones Trajectory new", "position", [100 100 1200 750]);
%增加暂停按钮
bt =uicontrol(Fig,'Style','togglebutton','String','||','FontSize',10,'Units','normalized','Position',[0.92,0.3,0.05,0.05],'Callback',@do);

% 创建坐标轴

subplot(2,3,1) % drone飞行轨迹
axis([-SL SL -SL SL]);
axis square;
hold on;
% 添加标题和标签
title('Drones Trajectory');
xlabel('X m');
ylabel('Y m');

subplot(2,3,2) % 节点Aoi
axis square;
hold on;
% 添加标题和标签
xlabel("t-s");
ylabel("Aoi-s");
title("N-Aoi");

subplot(2,3,3) % 长期Aoi
axis square;
hold on;
% 添加标题和标签
xlabel("t-s");
ylabel("Aoi-s");
title("L-Aoi");

subplot(2,3,4) % 周期耗时
axis square;
hold on;
% 添加标题和标签
xlabel("turns");
xticks(0:1:50);
ylabel("t-s");
title("time");

subplot(2,3,5) % drone使用数量（个）
axis square;
hold on;
% 添加标题和标签
xlabel("turns");
xticks(0:1:50);
ylabel("drone number");
yticks(0:10:200);
title("drone used");

subplot(2,3,6) % 能量总消耗
axis square;
hold on;
% 添加标题和标签
xlabel("turns");
xticks(0:1:50);
ylabel("Energy");
title("Energy cost");


% 画出扇区
i=1;
while i <=Sm
    subplot(2,3,1)
    plot([0,SL*cos((2*pi/Sm)*(i-1))],[0,SL*sin((2*pi/Sm)*(i-1))],'r-');
    i=i+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 初始化drone的起始角
theta = [];
i =1;
while i <=Sm
    theta = [theta,(2*pi/Sm)*(i-1)+pp*pi/Sm];%每个drone的起始角
    i=i+1;
end

tcopy=1;
dronenumber=Sm*circle;% 已用无人机数量
dronenumbercope=dronenumber;
subplot(2,3,5) % 定位窗口
plot(1,dronenumber,'*','color',[0 0 1], 'MarkerSize', 5 );% 绘制已用无人机数量
DD=[DD;dronenumber];


Node_aoi=0;%临时参数
Node_aoi_copy=0;%临时参数
Node_aoi_T=0;%临时参数
kklcopy=1;
kkl=0;
kkj=0;
T_aoi=0;%临时参数
T_aoi_node=0;%临时参数
g=0;%临时参数
energy_before=0;%临时参数
energy_mid=0;%临时参数
dy=[];


t=1;
T=0;%系统时间
RT=0;%每个周期结束时间
while t<=turn %数据收集轮数

    if g~=0
        delete(g);
    end
    g=annotation('textbox',[0.92,0.4,0.05,0.05],'LineStyle','-','LineWidth',2,'String',t);%文本框显示轮次

    % 初始化drone的目标节点node
    node=zeros(Node,9);%node的数组信息：位置角，位置半径，扇区数，环区数，遭遇时间，坐标位置，数据收集结束时间，投递结束时间，doom标志（1：有效，0：无效）
    dg = [];
    i =1;
    while i <=Sm
        dg = [dg ;(2*pi/Sm)*i];%扇与扇之间的接触边界
        i=i+1;
    end
    dx = [];
    i =1;
    while i <=circle-1
        dx = [dx ;(dr(i)+dr(i+1))/2];%环与环之间的接触边界
        i=i+1;
    end

    i=1;
    while i <=Node%产生随机目标节点
        while true
            cc=(randi(cell-4)+2)*2*pi/cell;%产生随机角度
            drag=0;
            j=1;
            while j<=Sm
                if cc == dg(j) ||cc == theta(j)
                    drag=1;%排除扇与扇之间接触边界的节点和无人机中轴线节点
                end
                j=j+1;
            end
            if drag==0
                break;
            end
        end
        shan=ceil(cc/(2*pi/Sm));%所属扇区

        while true
            ee=randi(SL*10)/10;%产生随机半径
            drag=0;
            j=1;
            while j<=circle-1
                if ee == dx(j)
                    drag=1;%排除环与环之间接触边界的节点
                end
                j=j+1;
            end
            if drag==0
                break;
            end
        end

        j=1;
        while j <=circle-1
            if ee<dx(j)
                node(i,4)=j;%所属环
                break;
            end
            j=j+1;
        end

        node(i,1)=cc;
        node(i,2)=ee;
        node(i,3)=shan;
        if node(i,4) == 0
            node(i,4)=circle;%所属环
        end
        node(i,9)=1;
        i=i+1;
    end

    i=1;
    while i <=Node
        subplot(2,3,1)
        b=plot(node(i,2)*cos(node(i,1)),node(i,2)*sin(node(i,1)),'o','color',[1 0 1], 'MarkerSize', 4 );% 绘制目标节点
        node(i,6)=b;%坐标位置集
        i=i+1;
    end


    % 初始化drone
    if t ==1
        drone=zeros(circle,19,Sm);%drone的数组信息：现在位置，上一跳位置，起始位置，现在坐标位置，顺时针最远目标位置，逆时针最远目标位置，剩余能量，携带数据包个数，拍摄数据倒计时，投递数据包倒计时，方向位标志1，方向位标志2，方向位标志3,开始产生数据时间,开始投递数据时间,结束投递时间,被外环锁住标志,投递数据包个数,无线电波连线
        drone_node=zeros(circle,Node,Sm);
    end
    z=1;
    while  z <= Sm     %循环每个扇区
        x=1;
        while  x <= circle %循环每个环

            n = dr(x)*cos(theta(z));
            m = dr(x)*sin(theta(z));
            subplot(2,3,1)
            b=plot(n,m,'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制初始drone
            drone(x,1,z)=theta(z);%更新现在位置
            drone(x,2,z)=theta(z);%更新上一跳位置
            drone(x,3,z)=theta(z);%更新起始位置
            drone(x,4,z)=b;%更新现在跳点坐标位置
            if t ==1
                drone(x,7,z)=Jer;%更新剩余能量
                drone(x,14,z)=0;%开始产生数据包时间
                drone(x,15,z)=0;%开始投递数据时间
                drone(x,16,z)=0;%结束投递数据时间
                drone(x,18,z)=0;%更新投递数据包个数
            end
            drone(x,8,z)=0;%更新携带数据包个数
            drone(x,9,z)=0;%更新拍摄数据倒计时
            drone(x,10,z)=0;%更新投递数据包倒计时
            drone(x,11,z)=1;%更新逆时针飞行标志
            drone(x,12,z)=1;%更新投递终点标志
            drone(x,13,z)=1;%更新顺时针飞行标志
            drone(x,17,z)=0;%更新被外环锁住标志
            drone(x,19,z)=0;%无线电波

            min_shun=drone(x,3,z);
            max_ni=drone(x,3,z);
            u=1;
            while  u <= Node     %更新环上的每个扇区
                neck = node(u,1);
                if node(u,3)==z && node(u,4)==x
                    if neck<min_shun
                        min_shun =neck;%找到顺时针最远目标位置
                    end
                    if neck>max_ni
                        max_ni =neck;%找到逆时针最远目标位置
                    end
                end
                u=u+1;
            end
            drone(x,5,z)=min_shun;%更新顺时针最远目标位置
            drone(x,6,z)=max_ni;%更新逆时针最远目标位置
            x=x+1;
        end
        z=z+1;
    end


    while true
        T=T+St;
        pause(St); % 延时一段时间，以控制绘制速度
        z=1;
        while  z <= Sm     %更新每个扇区
            x=1;
            while  x <= circle %更新每个环
                if x==1
                    c=drone(x,13,z);%c是顺时针方向标
                    zmin=drone(x,5,z);%最右端
                    zmax=drone(x,6,z);%最左端
                    if drone(x,1,z)==drone(x,3,z) %drone在中轴线
                        if drone(x,17,z)==1 %被外环锁
                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                        else
                            if c==1 %朝着顺时针方向
                                if drone(x,3,z)==zmin %最右端等于初始位置
                                    drone(x,13,z)=0;%终结顺时针方向
                                    if drone(x,3,z)==zmax %最左端等于初始位置
                                        drone(x,11,z)=0;%a置零，a是逆时针方向标，逆时针没有数据
                                        drone(x,12,z)=0;%b置零，b是原点静止标志，已在投递节点，无须移动
                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                    else
                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                        drone(x,1,z)=drone(x,1,z)+dcell;%逆时针飞行，更新现在位置
                                        subplot(2,3,1)
                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                        delete(drone(x,4,z));%删除上一跳drone
                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行能量减少

                                        if drone(x,1,z)>=zmax && zmax>=drone(x,2,z)  %到达最左端
                                            drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                            drone(x,11,z)=0;%a置零，逆时针没有数据
                                            i=1;
                                            while i<=Node
                                                if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                    node(i,5)=T;%更新遭遇时间
                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                    j=1;
                                                    while j<=Node
                                                        if drone_node(x,j,z) ==0
                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                            break
                                                        end
                                                        j=j+1;
                                                    end
                                                    %  break;
                                                end
                                                i=i+1;
                                            end
                                        end
                                    end
                                else
                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                    drone(x,1,z)=drone(x,1,z)-dcell;%顺时针飞行，更新现在位置
                                    subplot(2,3,1)
                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                    delete(drone(x,4,z));%删除上一跳drone
                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                    if drone(x,1,z)<=zmin && zmin<=drone(x,2,z) %到达最右端
                                        drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                        drone(x,13,z)=0;%c置零，顺时针没有数据
                                        i=1;
                                        while i<=Node
                                            if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                node(i,5)=T;%更新遭遇时间
                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                j=1;
                                                while j<=Node
                                                    if drone_node(x,j,z) ==0
                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                        break
                                                    end
                                                    j=j+1;
                                                end
                                                %  break;
                                            end
                                            i=i+1;
                                        end
                                    end
                                end
                            else
                                if drone(x,11,z)==1
                                    if drone(x,9,z)>0%处于drone拍摄时间
                                        drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                        drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                        if drone(x,9,z)<=0 %drone拍摄时间结束
                                            i=Node;
                                            need=0;
                                            while i>=1
                                                if drone_node(x,i,z)~=0
                                                    need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中
                                                    if node(need,9)==1
                                                        node(need,7)=T;%更新数据收集结束时间
                                                        node(need,9)=0;%更新doom标志为零
                                                        delete(node(need,6));%更新doom节点，令其消失
                                                    else
                                                        break;
                                                    end
                                                end
                                                i=i-1;
                                            end
                                            if drone(x,8,z)>0 %判断是否有携带包
                                                drone(x,10,z)=drone(x,8,z)*St_ttCinside; %投递时间等于携带包个数X每个包的投递时间
                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                drone(x,15,z)=T;%开始投递的时间
                                                drone(x,14,z)=T;%开始产生投递数据的时间
                                                subplot(2,3,1)%定位窗口位置
                                                b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                            end
                                        end
                                    else
                                        if drone(x,10,z)>0 %处于投递数据包时间
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                            drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻
                                            if drone(x,10,z)>0
                                                if T-drone(x,15,z)>=St_ttCinside
                                                    drone(x,15,z)=T;
                                                    drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            delete(drone(x,19,z)) %删除投递无线电波
                                                        end
                                                    else
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                            delete(drone(x,19,z)) %删除投递无线电波
                                                        end
                                                    end
                                                end
                                            else
                                                if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                            node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                            dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                            drone_node(x,i,z) =0; %node目录点置零
                                                            break;%已经传完一个包
                                                        end
                                                        i=i+1;
                                                    end
                                                end

                                                if x~=1 %非内环
                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                end
                                                drone(x,8,z) = 0;%携带包的个数置零
                                                drone(x,16,z) = T;%更新投递结束时间

                                                i=1;
                                                while i<=Node
                                                    if drone_node(x,i,z) ~=0
                                                        drone_node(x,i,z) =0; %node目录置零
                                                        break;
                                                    end
                                                    i=i+1;
                                                end
                                                delete(drone(x,19,z)) %删除投递无线电波
                                            end
                                        else
                                            if drone(x,8,z)>0
                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,8,z)*St_ttCinside-St; %投递时间等于携带包个数X每个包的投递时间
                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                drone(x,15,z)=T-St;%开始投递的时间
                                                drone(x,14,z)=T-St;%开始产生投递数据的时间
                                                subplot(2,3,1)%定位窗口位置
                                                b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                            else
                                                drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                drone(x,1,z)=drone(x,1,z)+dcell;%逆时针飞行，更新现在位置
                                                subplot(2,3,1)
                                                b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                delete(drone(x,4,z));%删除上一跳drone
                                                drone(x,4,z)=b;%更新现在跳点坐标位置
                                                drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                if drone(x,2,z)<=zmax && zmax<=drone(x,1,z)  %到达最左端
                                                    drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                    drone(x,11,z)=0;%a置零，逆时针没有数据
                                                    i=1;
                                                    while i<=Node
                                                        if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                            node(i,5)=T;%更新遭遇时间
                                                            drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                            drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x,j,z) ==0
                                                                    drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                    break
                                                                end
                                                                j=j+1;
                                                            end
                                                            %  break;
                                                        end
                                                        i=i+1;
                                                    end
                                                end
                                            end
                                        end
                                    end
                                else
                                    if drone(x,9,z)>0%处于drone拍摄时间
                                        drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                        drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                        if drone(x,9,z)<=0 %drone拍摄时间结束
                                            i=Node;
                                            need=0;
                                            while i>=1
                                                if drone_node(x,i,z)~=0
                                                    need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中
                                                    if node(need,9)==1
                                                        node(need,7)=T;%更新数据收集结束时间
                                                        node(need,9)=0;%更新doom标志为零
                                                        delete(node(need,6));%更新doom节点，令其消失
                                                    else
                                                        break;
                                                    end
                                                end
                                                i=i-1;
                                            end
                                            drone(x,10,z)=drone(x,8,z)*St_ttCinside; %投递时间等于携带包个数X每个包的投递时间
                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                            drone(x,15,z)=T;%开始投递的时间
                                            drone(x,14,z)=T;%开始产生投递数据的时间
                                            subplot(2,3,1)%定位窗口位置
                                            b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                        end
                                    else
                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                        if drone(x,10,z)>0 %处于投递数据包时间
                                            drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                            drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻
                                            if drone(x,10,z)>0
                                                if T-drone(x,15,z)>=St_ttCinside
                                                    drone(x,15,z)=T;
                                                    drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            delete(drone(x,19,z)) %删除投递无线电波
                                                        end
                                                    else
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                            delete(drone(x,19,z)) %删除投递无线电波
                                                        end
                                                    end
                                                end
                                            else
                                                if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                            node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                            dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                            drone_node(x,i,z) =0; %node目录点置零
                                                            break;%已经传完一个包
                                                        end
                                                        i=i+1;
                                                    end
                                                end

                                                if x~=1 %非内环
                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                end
                                                drone(x,8,z) = 0;%携带包的个数置零
                                                drone(x,16,z) = T;%更新投递结束时间

                                                i=1;
                                                while i<=Node
                                                    if drone_node(x,i,z) ~=0
                                                        drone_node(x,i,z) =0; %node目录置零
                                                        break;
                                                    end
                                                    i=i+1;
                                                end
                                                delete(drone(x,19,z)) %删除投递无线电波
                                            end
                                        else
                                            if drone(x,8,z)>0
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,8,z)*St_ttCinside-St; %投递时间等于携带包个数X每个包的投递时间
                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                drone(x,15,z)=T-St;%开始投递的时间
                                                drone(x,14,z)=T-St;%开始产生投递数据的时间
                                                subplot(2,3,1)%定位窗口位置
                                                b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    else
                        if drone(x,17,z)==1 %被外环锁
                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                        else
                            if drone(x,9,z)>0%处于drone拍摄时间
                                drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                if drone(x,9,z)<=0 %drone拍摄时间结束
                                    i=Node;
                                    need=0;
                                    while i>=1
                                        if drone_node(x,i,z)~=0
                                            need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中
                                            if node(need,9)==1
                                                node(need,7)=T;%更新数据收集结束时间
                                                node(need,9)=0;%更新doom标志为零
                                                delete(node(need,6));%更新doom节点，令其消失
                                            else
                                                break;
                                            end
                                        end
                                        i=i-1;
                                    end

                                    drone(x,10,z)=drone(x,8,z)*St_ttCinside; %投递时间等于携带包个数X每个包的投递时间
                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                    drone(x,15,z)=T;%开始投递的时间
                                    drone(x,14,z)=T;%开始产生投递数据的时间
                                    subplot(2,3,1)%定位窗口位置
                                    b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                end
                            else
                                if drone(x,10,z)>0
                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                    drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                    drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻
                                    if drone(x,10,z)>0
                                        if T-drone(x,15,z)>=St_ttCinside
                                            drone(x,15,z)=T;
                                            drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                            if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                i=1;
                                                while i<=Node
                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                        node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                        dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                        drone_node(x,i,z) =0; %node目录点置零
                                                        break;%已经传完一个包
                                                    end
                                                    i=i+1;
                                                end
                                                if drone(x,8,z) ==0 %投递结束
                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                    drone(x,16,z) = T;%更新投递结束时间
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            else
                                                i=1;
                                                while i<=Node
                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                        drone_node(x,i,z) =0; %node目录点置零
                                                        break;%已经传完一个包
                                                    end
                                                    i=i+1;
                                                end
                                                if drone(x,8,z) ==0 %投递结束
                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                    drone(x,16,z) = T;%更新投递结束时间
                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            end
                                        end
                                    else
                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                            i=1;
                                            while i<=Node
                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                    drone_node(x,i,z) =0; %node目录点置零
                                                    break;%已经传完一个包
                                                end
                                                i=i+1;
                                            end
                                        end

                                        if x~=1 %非内环
                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                        end
                                        drone(x,8,z) = 0;%携带包的个数置零
                                        drone(x,16,z) = T;%更新投递结束时间

                                        i=1;
                                        while i<=Node
                                            if drone_node(x,i,z) ~=0
                                                drone_node(x,i,z) =0; %node目录置零
                                                break;
                                            end
                                            i=i+1;
                                        end
                                        delete(drone(x,19,z)) %删除投递无线电波
                                    end
                                else
                                    if drone(x,13,z)==1%顺时针方向标

                                        if drone(x,8,z)>0
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                            drone(x,10,z)=drone(x,8,z)*St_ttCinside-St; %投递时间等于携带包个数X每个包的投递时间
                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                            drone(x,15,z)=T-St;%开始投递的时间
                                            drone(x,14,z)=T-St;%开始产生投递数据的时间
                                            subplot(2,3,1)%定位窗口位置
                                            b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                        else

                                            drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                            drone(x,1,z)=drone(x,1,z)-dcell;%顺时针飞，更新现在位置
                                            subplot(2,3,1)
                                            b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                            delete(drone(x,4,z));%删除上一跳drone
                                            drone(x,4,z)=b;%更新现在跳点坐标位置
                                            drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                            if drone(x,1,z)<=zmin && zmin<=drone(x,2,z) %到达最右端
                                                drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                drone(x,13,z)=0;%c置零，顺时针没有数据

                                                i=1;
                                                while i<=Node
                                                    if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                        node(i,5)=T;%更新遭遇时间
                                                        drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                        drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                        j=1;
                                                        while j<=Node
                                                            if drone_node(x,j,z) ==0
                                                                drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                break
                                                            end
                                                            j=j+1;
                                                        end
                                                        %  break;
                                                    end
                                                    i=i+1;
                                                end
                                            end
                                        end
                                    else
                                        if drone(x,1,z) < drone(x,3,z) %处于右端逆时针往回飞状态

                                            if drone(x,8,z)>0
                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,8,z)*St_ttCinside-St; %投递时间等于携带包个数X每个包的投递时间
                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                drone(x,15,z)=T-St;%开始投递的时间
                                                drone(x,14,z)=T-St;%开始产生投递数据的时间
                                                subplot(2,3,1)%定位窗口位置
                                                b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                            else
                                                drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                drone(x,1,z)=drone(x,1,z)+dcell;%逆时针往回飞，更新现在位置
                                                subplot(2,3,1)
                                                b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                delete(drone(x,4,z));%删除上一跳drone
                                                drone(x,4,z)=b;%更新现在跳点坐标位置
                                                drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                if drone(x,2,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,1,z)%到达起点

                                                    drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置
                                                    if zmax==drone(x,3,z)  %最左端无目标节点

                                                        drone(x,11,z)=0;%a置零
                                                        drone(x,12,z)=0;%b置零
                                                    end
                                                end

                                                i=1;
                                                while i<=Node
                                                    if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                        node(i,5)=T;%更新遭遇时间
                                                        drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                        drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                        j=1;
                                                        while j<=Node
                                                            if drone_node(x,j,z) ==0
                                                                drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                break
                                                            end
                                                            j=j+1;
                                                        end
                                                        %  break;
                                                    end
                                                    i=i+1;
                                                end
                                            end
                                        else
                                            if drone(x,11,z) ==1%a为1，表明正在左端逆时针飞行

                                                if drone(x,8,z)>0
                                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                                    drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                    drone(x,10,z)=drone(x,8,z)*St_ttCinside-St; %投递时间等于携带包个数X每个包的投递时间
                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                    drone(x,15,z)=T-St;%开始投递的时间
                                                    drone(x,14,z)=T-St;%开始产生投递数据的时间
                                                    subplot(2,3,1)%定位窗口位置
                                                    b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                else

                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)+dcell;%更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    if drone(x,1,z)>=zmax && drone(x,2,z)<=zmax  %到达最左端
                                                        drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                        drone(x,11,z)=0;%a置零，逆时针方向已经没有数据
                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end
                                                    end
                                                end
                                            else
                                                if drone(x,8,z)>0
                                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                                    drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                    drone(x,10,z)=drone(x,8,z)*St_ttCinside-St; %投递时间等于携带包个数X每个包的投递时间
                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                    drone(x,15,z)=T-St;%开始投递的时间
                                                    drone(x,14,z)=T-St;%开始产生投递数据的时间
                                                    subplot(2,3,1)%定位窗口位置
                                                    b=plot([dr(x)*cos(drone(x,1,z)),0],[dr(x)*sin(drone(x,1,z)),0],'g-');%画出无线电传输线
                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                else
                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)-dcell;%更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    if drone(x,1,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,2,z) %到达原点
                                                        drone(x,1,z)=drone(x,3,z);
                                                        drone(x,12,z)=0;%b置零
                                                    end

                                                    i=1;
                                                    while i<=Node
                                                        if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                            node(i,5)=T;%更新遭遇时间
                                                            drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                            drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x,j,z) ==0
                                                                    drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                    break
                                                                end
                                                                j=j+1;
                                                            end
                                                            %  break;
                                                        end
                                                        i=i+1;
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                else %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    if mod(x,2)==0
                        a=drone(x,11,z);%a是逆时针方向标
                        zmin=drone(x,5,z);%最右端
                        zmax=drone(x,6,z);%最左端
                        if drone(x,1,z)==drone(x,3,z) %drone在中轴线
                            if drone(x,17,z)==1 %被外环锁
                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                            else
                                if a==1 %朝着逆时针方向
                                    if drone(x,3,z)==zmax %最左端等于初始位置
                                        drone(x,11,z)=0;%终结顺时针方向
                                        if drone(x,3,z)==zmin %最右端等于初始位置
                                            drone(x,13,z)=0;%c置零，c是顺时针方向标，顺时针没有数据
                                            drone(x,12,z)=0;%b置零，b是原点静止标志，已在投递节点，无须移动
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                        else
                                            drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                            drone(x,1,z)=drone(x,1,z)-dcell;%逆时针飞行，更新现在位置
                                            subplot(2,3,1)
                                            b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                            delete(drone(x,4,z));%删除上一跳drone
                                            drone(x,4,z)=b;%更新现在跳点坐标位置
                                            drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行能量减少


                                            if drone(x,2,z)>=zmin && zmin>=drone(x,1,z)  %到达最右端
                                                drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                drone(x,13,z)=0;%a置零，顺时针没有数据
                                                i=1;
                                                while i<=Node
                                                    if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                        node(i,5)=T;%更新遭遇时间
                                                        drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                        drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                        j=1;
                                                        while j<=Node
                                                            if drone_node(x,j,z) ==0
                                                                drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                break
                                                            end
                                                            j=j+1;
                                                        end
                                                        %  break;
                                                    end
                                                    i=i+1;
                                                end
                                            end
                                        end
                                    else
                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                        drone(x,1,z)=drone(x,1,z)+dcell;%顺时针飞行，更新现在位置
                                        subplot(2,3,1)
                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                        delete(drone(x,4,z));%删除上一跳drone
                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                        if drone(x,2,z)<=zmax && zmax<=drone(x,1,z) %到达最右端
                                            drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                            drone(x,11,z)=0;%c置零，顺时针没有数据
                                            i=1;
                                            while i<=Node
                                                if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                    node(i,5)=T;%更新遭遇时间
                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                    j=1;
                                                    while j<=Node
                                                        if drone_node(x,j,z) ==0
                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                            break
                                                        end
                                                        j=j+1;
                                                    end
                                                    %  break;
                                                end
                                                i=i+1;
                                            end
                                        end
                                    end
                                else
                                    if drone(x,13,z)==1%
                                        if drone(x,9,z)>0%处于drone拍摄时间
                                            drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                            drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                            if drone(x,9,z)<=0 %drone拍摄时间结束
                                                i=Node;
                                                need=0;
                                                while i>=1
                                                    if drone_node(x,i,z)~=0
                                                        need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中

                                                        if node(need,9)==1
                                                            node(need,7)=T;%更新数据收集结束时间
                                                            node(need,9)=0;%更新doom标志为零
                                                            delete(node(need,6));%更新doom节点，令其消失
                                                        else
                                                            break;
                                                        end
                                                    end
                                                    i=i-1;
                                                end

                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%更新开始投递产生时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        end
                                                    end
                                                end
                                            end
                                        else
                                            if drone(x,10,z)>0
                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                if drone(x,10,z)>0
                                                    if T-drone(x,15,z)>=St_ttCoutside
                                                        drone(x,15,z)=T;
                                                        drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                delete(drone(x,19,z)) %删除投递无线电波

                                                            end
                                                        else
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                delete(drone(x,19,z)) %删除投递无线电波
                                                            end
                                                        end
                                                    end
                                                else
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                    end

                                                    if x~=1 %非内环
                                                        drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                    end
                                                    drone(x,8,z) = 0;%携带包的个数置零
                                                    drone(x,16,z) = T;%更新投递结束时间

                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0
                                                            drone_node(x,i,z) =0; %node目录置零
                                                            break;
                                                        end
                                                        i=i+1;
                                                    end
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            else
                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包

                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%更新开始投递产生时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        else
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                        end
                                                    else

                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)-dcell;%顺时针飞行，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                        if drone(x,1,z)<=zmin && zmin<=drone(x,2,z)  %到达最右端
                                                            drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                            drone(x,13,z)=0;%a置零，顺时针没有数据

                                                            i=1;
                                                            while i<=Node
                                                                if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                    node(i,5)=T;%更新遭遇时间
                                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x,j,z) ==0
                                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                            break
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                    %  break;
                                                                end
                                                                i=i+1;
                                                            end
                                                        end
                                                    end
                                                else

                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)-dcell;%顺时针飞行，更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    wehave=0;
                                                    if drone(x,1,z)<=zmin && zmin<=drone(x,2,z)  %到达最左端
                                                        drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                        drone(x,13,z)=0;%a置零，顺时针没有数据

                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                wehave=1;
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end
                                                    end
                                                    if wehave ==0
                                                        if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                            if drone(x,8,z)>0 %判断是否有携带包

                                                                if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                    drone(x-1,17,z)=1;%锁异环
                                                                    drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                    drone(x,15,z)=T;%更新开始投递时间
                                                                    drone(x,14,z)=T;%更新开始投递产生时间
                                                                    drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                    i=1;
                                                                    while i<=Node
                                                                        if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                            break;
                                                                        else
                                                                            j=1;
                                                                            while j<=Node
                                                                                if drone_node(x-1,j,z) ==0
                                                                                    drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                    break;
                                                                                end
                                                                                j=j+1;
                                                                            end
                                                                        end
                                                                        i=i+1;
                                                                    end
                                                                    subplot(2,3,1)%定位窗口位置
                                                                    b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    else
                                        if drone(x,9,z)>0%处于drone拍摄时间
                                            drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                            drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                            if drone(x,9,z)<=0 %drone拍摄时间结束
                                                i=Node;
                                                need=0;
                                                while i>=1
                                                    if drone_node(x,i,z)~=0
                                                        need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中
                                                        if node(need,9)==1
                                                            node(need,7)=T;%更新数据收集结束时间
                                                            node(need,9)=0;%更新doom标志为零
                                                            delete(node(need,6));%更新doom节点，令其消失
                                                        else
                                                            break;
                                                        end
                                                    end
                                                    i=i-1;
                                                end

                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%开始产生投递数据的时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        end
                                                    end
                                                end
                                            end
                                        else
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                            if drone(x,10,z)>0 %处于投递数据包时间
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                if drone(x,10,z)>0
                                                    if T-drone(x,15,z)>=St_ttCoutside
                                                        drone(x,15,z)=T;
                                                        drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                delete(drone(x,19,z)) %删除投递无线电波

                                                            end
                                                        else
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                delete(drone(x,19,z)) %删除投递无线电波
                                                            end
                                                        end
                                                    end
                                                else
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                    end

                                                    if x~=1 %非内环
                                                        drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                    end
                                                    drone(x,8,z) = 0;%携带包的个数置零
                                                    drone(x,16,z) = T;%更新投递结束时间

                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0
                                                            drone_node(x,i,z) =0; %node目录置零
                                                            break;
                                                        end
                                                        i=i+1;
                                                    end
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            else

                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%更新开始投递产生时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        else
                            if drone(x,17,z)==1 %被外环锁
                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                            else
                                if drone(x,9,z)>0%处于drone拍摄时间
                                    drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                    drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                    if drone(x,9,z)<=0 %drone拍摄时间结束
                                        i=Node;
                                        need=0;
                                        while i>=1
                                            if drone_node(x,i,z)~=0
                                                need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中
                                                if node(need,9)==1
                                                    node(need,7)=T;%更新数据收集结束时间
                                                    node(need,9)=0;%更新doom标志为零
                                                    delete(node(need,6));%更新doom节点，令其消失
                                                else
                                                    break;
                                                end
                                            end
                                            i=i-1;
                                        end

                                        if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                            if drone(x,8,z)>0 %判断是否有携带包
                                                if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                    drone(x-1,17,z)=1;%锁异环
                                                    drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                    drone(x,15,z)=T;%更新开始投递时间
                                                    drone(x,14,z)=T;%开始产生投递数据的时间
                                                    drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                            break;
                                                        else
                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x-1,j,z) ==0
                                                                    drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                    break;
                                                                end
                                                                j=j+1;
                                                            end
                                                        end
                                                        i=i+1;
                                                    end
                                                    subplot(2,3,1)%定位窗口位置
                                                    b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                end
                                            end
                                        end
                                    end
                                else
                                    if drone(x,11,z)==1%逆时针方向标

                                        if drone(x,10,z)>0
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                            drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                            if drone(x,10,z)>0
                                                if T-drone(x,15,z)>=St_ttCoutside
                                                    drone(x,15,z)=T;
                                                    drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            delete(drone(x,19,z)) %删除投递无线电波

                                                        end
                                                    else
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                            delete(drone(x,19,z)) %删除投递无线电波
                                                        end
                                                    end
                                                end
                                            else
                                                if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                            node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                            dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                            drone_node(x,i,z) =0; %node目录点置零
                                                            break;%已经传完一个包
                                                        end
                                                        i=i+1;
                                                    end
                                                end

                                                if x~=1 %非内环
                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                end
                                                drone(x,8,z) = 0;%携带包的个数置零
                                                drone(x,16,z) = T;%更新投递结束时间

                                                i=1;
                                                while i<=Node
                                                    if drone_node(x,i,z) ~=0
                                                        drone_node(x,i,z) =0; %node目录置零
                                                        break;
                                                    end
                                                    i=i+1;
                                                end
                                                delete(drone(x,19,z)) %删除投递无线电波
                                            end
                                        else
                                            if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                if drone(x,8,z)>0 %判断是否有携带包
                                                    if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                        drone(x-1,17,z)=1;%锁异环
                                                        drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                        drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                        drone(x,15,z)=T;%更新开始投递时间
                                                        drone(x,14,z)=T;%开始产生投递数据的时间
                                                        drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                break;
                                                            else
                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x-1,j,z) ==0
                                                                        drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                        break;
                                                                    end
                                                                    j=j+1;
                                                                end
                                                            end
                                                            i=i+1;
                                                        end
                                                        subplot(2,3,1)%定位窗口位置
                                                        b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                        drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                    else
                                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                    end
                                                else
                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)+dcell;%逆时针飞，更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    if drone(x,2,z)<=zmax && zmax<=drone(x,1,z) %到达最左端
                                                        drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                        drone(x,11,z)=0;%a置零，逆时针没有数据
                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end
                                                    end
                                                end
                                            else
                                                drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                drone(x,1,z)=drone(x,1,z)+dcell;%顺时针飞，更新现在位置
                                                subplot(2,3,1)
                                                b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                delete(drone(x,4,z));%删除上一跳drone
                                                drone(x,4,z)=b;%更新现在跳点坐标位置
                                                drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                wehave=0;
                                                if drone(x,2,z)<=zmax && zmax<=drone(x,1,z) %到达最左端
                                                    drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                    drone(x,11,z)=0;%a置零，逆时针没有数据
                                                    i=1;
                                                    while i<=Node
                                                        if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                            wehave=1;
                                                            node(i,5)=T;%更新遭遇时间
                                                            drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                            drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x,j,z) ==0
                                                                    drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                    break
                                                                end
                                                                j=j+1;
                                                            end
                                                            %  break;
                                                        end
                                                        i=i+1;
                                                    end
                                                end
                                                if wehave==0
                                                    if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                        if drone(x,8,z)>0 %判断是否有携带包
                                                            if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                drone(x-1,17,z)=1;%锁异环
                                                                drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                drone(x,15,z)=T;%更新开始投递时间
                                                                drone(x,14,z)=T;%开始产生投递数据的时间
                                                                drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                        break;
                                                                    else
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x-1,j,z) ==0
                                                                                drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                break;
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                subplot(2,3,1)%定位窗口位置
                                                                b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                            end
                                                        end
                                                    end

                                                end
                                            end
                                        end
                                    else
                                        if drone(x,1,z) > drone(x,3,z) %处于左端顺时针往回飞状态
                                            if drone(x,10,z)>0
                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                if drone(x,10,z)>0
                                                    if T-drone(x,15,z)>=St_ttCoutside
                                                        drone(x,15,z)=T;
                                                        drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                delete(drone(x,19,z)) %删除投递无线电波

                                                            end
                                                        else
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                delete(drone(x,19,z)) %删除投递无线电波
                                                            end
                                                        end
                                                    end
                                                else
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                    end

                                                    if x~=1 %非内环
                                                        drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                    end
                                                    drone(x,8,z) = 0;%携带包的个数置零
                                                    drone(x,16,z) = T;%更新投递结束时间

                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0
                                                            drone_node(x,i,z) =0; %node目录置零
                                                            break;
                                                        end
                                                        i=i+1;
                                                    end
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            else
                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%开始产生投递数据的时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        else
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                        end
                                                    else
                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)-dcell;%顺时针往回飞，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                        if drone(x,1,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,2,z)%到达起点
                                                            drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置

                                                            if zmin==drone(x,3,z)  %最右端无目标节点
                                                                drone(x,13,z)=0;%a置零
                                                                drone(x,12,z)=0;%b置零
                                                            end

                                                        end
                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end
                                                    end
                                                else

                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)-dcell;%逆时针往回飞，更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    if drone(x,1,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,2,z)%到达起点
                                                        drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置

                                                        if zmin==drone(x,3,z)  %最右端无目标节点
                                                            drone(x,13,z)=0;%a置零
                                                            drone(x,12,z)=0;%b置零

                                                        end
                                                    end
                                                    wehave=0;

                                                    i=1;
                                                    while i<=Node
                                                        if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                            wehave=1;
                                                            node(i,5)=T;%更新遭遇时间
                                                            drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                            drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x,j,z) ==0
                                                                    drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                    break
                                                                end
                                                                j=j+1;
                                                            end
                                                            %  break;
                                                        end
                                                        i=i+1;
                                                    end

                                                    if wehave==0
                                                        if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                            if drone(x,8,z)>0 %判断是否有携带包
                                                                if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                    drone(x-1,17,z)=1;%锁异环
                                                                    drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                    drone(x,15,z)=T;%更新开始投递时间
                                                                    drone(x,14,z)=T;%开始产生投递数据的时间
                                                                    drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                    i=1;
                                                                    while i<=Node
                                                                        if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                            break;
                                                                        else
                                                                            j=1;
                                                                            while j<=Node
                                                                                if drone_node(x-1,j,z) ==0
                                                                                    drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                    break;
                                                                                end
                                                                                j=j+1;
                                                                            end
                                                                        end
                                                                        i=i+1;
                                                                    end
                                                                    subplot(2,3,1)%定位窗口位置
                                                                    b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        else
                                            if drone(x,13,z) ==1%c为1，表明正在右端顺时针飞行
                                                if drone(x,10,z)>0
                                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                    drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                    drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                    if drone(x,10,z)>0
                                                        if T-drone(x,15,z)>=St_ttCoutside
                                                            drone(x,15,z)=T;
                                                            drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                            if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                        dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    delete(drone(x,19,z)) %删除投递无线电波

                                                                end
                                                            else
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                                end
                                                            end
                                                        end
                                                    else
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                        end

                                                        if x~=1 %非内环
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                        end
                                                        drone(x,8,z) = 0;%携带包的个数置零
                                                        drone(x,16,z) = T;%更新投递结束时间

                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0
                                                                drone_node(x,i,z) =0; %node目录置零
                                                                break;
                                                            end
                                                            i=i+1;
                                                        end
                                                        delete(drone(x,19,z)) %删除投递无线电波
                                                    end
                                                else
                                                    if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                        if drone(x,8,z)>0 %判断是否有携带包
                                                            if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                                drone(x-1,17,z)=1;%锁异环
                                                                drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                drone(x,15,z)=T;%更新开始投递时间
                                                                drone(x,14,z)=T;%开始产生投递数据的时间
                                                                drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                        break;
                                                                    else
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x-1,j,z) ==0
                                                                                drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                break;
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                subplot(2,3,1)%定位窗口位置
                                                                b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                            else
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                            end
                                                        else
                                                            drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                            drone(x,1,z)=drone(x,1,z)-dcell;%逆时针往回飞，更新现在位置
                                                            subplot(2,3,1)
                                                            b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                            delete(drone(x,4,z));%删除上一跳drone
                                                            drone(x,4,z)=b;%更新现在跳点坐标位置
                                                            drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                            if drone(x,2,z)>=zmin && drone(x,1,z)<=zmin  %到达最右端
                                                                drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                                drone(x,13,z)=0;%c置零，顺时针方向已经没有数据
                                                                i=1;
                                                                while i<=Node
                                                                    if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                        node(i,5)=T;%更新遭遇时间
                                                                        drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                        drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x,j,z) ==0
                                                                                drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                                break
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                        %  break;
                                                                    end
                                                                    i=i+1;
                                                                end
                                                            end
                                                        end
                                                    else

                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)-dcell;%顺时针往回飞，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少


                                                        wehave=0;
                                                        if drone(x,2,z)>=zmin && drone(x,1,z)<=zmin  %到达最右端
                                                            drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                            drone(x,13,z)=0;%c置零，顺时针方向已经没有数据

                                                            i=1;
                                                            while i<=Node
                                                                if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                    wehave=1;
                                                                    node(i,5)=T;%更新遭遇时间
                                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x,j,z) ==0
                                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                            break
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                    %  break;
                                                                end
                                                                i=i+1;
                                                            end
                                                        end

                                                        if wehave==0
                                                            if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                                if drone(x,8,z)>0 %判断是否有携带包
                                                                    if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                        drone(x-1,17,z)=1;%锁异环
                                                                        drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                        drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                        drone(x,15,z)=T;%更新开始投递时间
                                                                        drone(x,14,z)=T;%开始产生投递数据的时间
                                                                        drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                        i=1;
                                                                        while i<=Node
                                                                            if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                                break;
                                                                            else
                                                                                j=1;
                                                                                while j<=Node
                                                                                    if drone_node(x-1,j,z) ==0
                                                                                        drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                        break;
                                                                                    end
                                                                                    j=j+1;
                                                                                end
                                                                            end
                                                                            i=i+1;
                                                                        end
                                                                        subplot(2,3,1)%定位窗口位置
                                                                        b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                        drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                    end
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            else
                                                if drone(x,10,z)>0
                                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                    drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                    drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                    if drone(x,10,z)>0
                                                        if T-drone(x,15,z)>=St_ttCoutside
                                                            drone(x,15,z)=T;
                                                            drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                            if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                        dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    delete(drone(x,19,z)) %删除投递无线电波

                                                                end
                                                            else
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                                end
                                                            end
                                                        end
                                                    else
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                        end

                                                        if x~=1 %非内环
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                        end
                                                        drone(x,8,z) = 0;%携带包的个数置零
                                                        drone(x,16,z) = T;%更新投递结束时间

                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0
                                                                drone_node(x,i,z) =0; %node目录置零
                                                                break;
                                                            end
                                                            i=i+1;
                                                        end
                                                        delete(drone(x,19,z)) %删除投递无线电波
                                                    end
                                                else
                                                    if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                        if drone(x,8,z)>0 %判断是否有携带包
                                                            if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                                drone(x-1,17,z)=1;%锁异环
                                                                drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                drone(x,15,z)=T;%更新开始投递时间
                                                                drone(x,14,z)=T;%开始产生投递数据的时间
                                                                drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                        break;
                                                                    else
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x-1,j,z) ==0
                                                                                drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                break;
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                subplot(2,3,1)%定位窗口位置
                                                                b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                            else
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                            end
                                                        else
                                                            drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                            drone(x,1,z)=drone(x,1,z)+dcell;%顺时针往回飞，更新现在位置
                                                            subplot(2,3,1)
                                                            b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                            delete(drone(x,4,z));%删除上一跳drone
                                                            drone(x,4,z)=b;%更新现在跳点坐标位置
                                                            drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                            if drone(x,2,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,1,z)%到达起点
                                                                drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置
                                                                drone(x,12,z)=0;%b置零
                                                            end

                                                            i=1;
                                                            while i<=Node
                                                                if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                    node(i,5)=T;%更新遭遇时间
                                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x,j,z) ==0
                                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                            break
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                    %  break;
                                                                end
                                                                i=i+1;
                                                            end
                                                        end
                                                    else

                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)+dcell;%顺时针往回飞，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                        if drone(x,2,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,1,z)%到达起点
                                                            drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置
                                                            drone(x,12,z)=0;%b置零

                                                        end

                                                        wehave=0;
                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                wehave=1;
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end

                                                        if wehave==0
                                                            if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                                if drone(x,8,z)>0 %判断是否有携带包
                                                                    if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                        drone(x-1,17,z)=1;%锁异环
                                                                        drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                        drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                        drone(x,15,z)=T;%更新开始投递时间
                                                                        drone(x,14,z)=T;%开始产生投递数据的时间
                                                                        drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                        i=1;
                                                                        while i<=Node
                                                                            if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                                break;
                                                                            else
                                                                                j=1;
                                                                                while j<=Node
                                                                                    if drone_node(x-1,j,z) ==0
                                                                                        drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                        break;
                                                                                    end
                                                                                    j=j+1;
                                                                                end
                                                                            end
                                                                            i=i+1;
                                                                        end
                                                                        subplot(2,3,1)%定位窗口位置
                                                                        b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                        drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                    end
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    else
                        c=drone(x,13,z);%c是顺时针方向标
                        zmin=drone(x,5,z);%最右端
                        zmax=drone(x,6,z);%最左端
                        if drone(x,1,z)==drone(x,3,z) %drone在中轴线
                            if drone(x,17,z)==1 %被外环锁
                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                            else
                                if c==1 %朝着顺时针方向
                                    if drone(x,3,z)==zmin %最右端等于初始位置
                                        drone(x,13,z)=0;%终结顺时针方向
                                        if drone(x,3,z)==zmax %最左端等于初始位置
                                            drone(x,11,z)=0;%a置零，a是逆时针方向标，逆时针没有数据
                                            drone(x,12,z)=0;%b置零，b是原点静止标志，已在投递节点，无须移动
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                        else
                                            drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                            drone(x,1,z)=drone(x,1,z)+dcell;%逆时针飞行，更新现在位置
                                            subplot(2,3,1)
                                            b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                            delete(drone(x,4,z));%删除上一跳drone
                                            drone(x,4,z)=b;%更新现在跳点坐标位置
                                            drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行能量减少


                                            if drone(x,1,z)>=zmax && zmax>=drone(x,2,z)  %到达最左端
                                                drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                drone(x,11,z)=0;%a置零，逆时针没有数据
                                                i=1;
                                                while i<=Node
                                                    if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                        node(i,5)=T;%更新遭遇时间
                                                        drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                        drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                        j=1;
                                                        while j<=Node
                                                            if drone_node(x,j,z) ==0
                                                                drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                break
                                                            end
                                                            j=j+1;
                                                        end
                                                        %  break;
                                                    end
                                                    i=i+1;
                                                end
                                            end
                                        end
                                    else
                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                        drone(x,1,z)=drone(x,1,z)-dcell;%顺时针飞行，更新现在位置
                                        subplot(2,3,1)
                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                        delete(drone(x,4,z));%删除上一跳drone
                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                        if drone(x,1,z)<=zmin && zmin<=drone(x,2,z) %到达最右端
                                            drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                            drone(x,13,z)=0;%c置零，顺时针没有数据
                                            i=1;
                                            while i<=Node
                                                if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                    node(i,5)=T;%更新遭遇时间
                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                    j=1;
                                                    while j<=Node
                                                        if drone_node(x,j,z) ==0
                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                            break
                                                        end
                                                        j=j+1;
                                                    end
                                                    %  break;
                                                end
                                                i=i+1;
                                            end
                                        end
                                    end
                                else
                                    if drone(x,11,z)==1%
                                        if drone(x,9,z)>0%处于drone拍摄时间
                                            drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                            drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                            if drone(x,9,z)<=0 %drone拍摄时间结束
                                                i=Node;
                                                need=0;
                                                while i>=1
                                                    if drone_node(x,i,z)~=0
                                                        need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中

                                                        if node(need,9)==1
                                                            node(need,7)=T;%更新数据收集结束时间
                                                            node(need,9)=0;%更新doom标志为零
                                                            delete(node(need,6));%更新doom节点，令其消失
                                                        else
                                                            break;
                                                        end
                                                    end
                                                    i=i-1;
                                                end

                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%更新开始投递产生时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        end
                                                    end
                                                end
                                            end
                                        else
                                            if drone(x,10,z)>0
                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                if drone(x,10,z)>0
                                                    if T-drone(x,15,z)>=St_ttCoutside
                                                        drone(x,15,z)=T;
                                                        drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                delete(drone(x,19,z)) %删除投递无线电波

                                                            end
                                                        else
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                delete(drone(x,19,z)) %删除投递无线电波
                                                            end
                                                        end
                                                    end
                                                else
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                    end

                                                    if x~=1 %非内环
                                                        drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                    end
                                                    drone(x,8,z) = 0;%携带包的个数置零
                                                    drone(x,16,z) = T;%更新投递结束时间

                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0
                                                            drone_node(x,i,z) =0; %node目录置零
                                                            break;
                                                        end
                                                        i=i+1;
                                                    end
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            else
                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包

                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%更新开始投递产生时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        else
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                        end
                                                    else

                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)+dcell;%逆时针飞行，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                        if drone(x,2,z)<=zmax && zmax<=drone(x,1,z)  %到达最左端
                                                            drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                            drone(x,11,z)=0;%a置零，逆时针没有数据

                                                            i=1;
                                                            while i<=Node
                                                                if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                    node(i,5)=T;%更新遭遇时间
                                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x,j,z) ==0
                                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                            break
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                    %  break;
                                                                end
                                                                i=i+1;
                                                            end
                                                        end
                                                    end
                                                else

                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)+dcell;%逆时针飞行，更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    wehave=0;
                                                    if drone(x,2,z)<=zmax && zmax<=drone(x,1,z)  %到达最左端
                                                        drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                        drone(x,11,z)=0;%a置零，逆时针没有数据

                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                wehave=1;
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end
                                                    end
                                                    if wehave==0

                                                        if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                            if drone(x,8,z)>0 %判断是否有携带包
                                                                if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                    drone(x-1,17,z)=1;%锁异环
                                                                    drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                    drone(x,15,z)=T;%更新开始投递时间
                                                                    drone(x,14,z)=T;%更新开始投递产生时间
                                                                    drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                    i=1;
                                                                    while i<=Node
                                                                        if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                            break;
                                                                        else
                                                                            j=1;
                                                                            while j<=Node
                                                                                if drone_node(x-1,j,z) ==0
                                                                                    drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                    break;
                                                                                end
                                                                                j=j+1;
                                                                            end
                                                                        end
                                                                        i=i+1;
                                                                    end
                                                                    subplot(2,3,1)%定位窗口位置
                                                                    b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    else
                                        if drone(x,9,z)>0%处于drone拍摄时间
                                            drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                            drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                            if drone(x,9,z)<=0 %drone拍摄时间结束
                                                i=Node;
                                                need=0;
                                                while i>=1
                                                    if drone_node(x,i,z)~=0
                                                        need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中
                                                        if node(need,9)==1
                                                            node(need,7)=T;%更新数据收集结束时间
                                                            node(need,9)=0;%更新doom标志为零
                                                            delete(node(need,6));%更新doom节点，令其消失
                                                        else
                                                            break;
                                                        end
                                                    end
                                                    i=i-1;
                                                end

                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%开始产生投递数据的时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        end
                                                    end
                                                end
                                            end
                                        else
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停则能量减少
                                            if drone(x,10,z)>0 %处于投递数据包时间
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                if drone(x,10,z)>0
                                                    if T-drone(x,15,z)>=St_ttCoutside
                                                        drone(x,15,z)=T;
                                                        drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                delete(drone(x,19,z)) %删除投递无线电波

                                                            end
                                                        else
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                delete(drone(x,19,z)) %删除投递无线电波
                                                            end
                                                        end
                                                    end
                                                else
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                    end

                                                    if x~=1 %非内环
                                                        drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                    end
                                                    drone(x,8,z) = 0;%携带包的个数置零
                                                    drone(x,16,z) = T;%更新投递结束时间

                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0
                                                            drone_node(x,i,z) =0; %node目录置零
                                                            break;
                                                        end
                                                        i=i+1;
                                                    end
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            else

                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%更新开始投递产生时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        else
                            if drone(x,17,z)==1 %被外环锁
                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                            else
                                if drone(x,9,z)>0%处于drone拍摄时间
                                    drone(x,9,z)=drone(x,9,z)-St;%拍摄数据倒计时减少St时刻；
                                    drone(x,7,z)=drone(x,7,z)-St_Pphoto;%拍摄数据则能量减少
                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                    if drone(x,9,z)<=0 %drone拍摄时间结束
                                        i=Node;
                                        need=0;
                                        while i>=1
                                            if drone_node(x,i,z)~=0
                                                need=drone_node(x,i,z);%找到收集中的node,已在drone_node的list中
                                                if node(need,9)==1
                                                    node(need,7)=T;%更新数据收集结束时间
                                                    node(need,9)=0;%更新doom标志为零
                                                    delete(node(need,6));%更新doom节点，令其消失
                                                else
                                                    break;
                                                end
                                            end
                                            i=i-1;
                                        end

                                        if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                            if drone(x,8,z)>0 %判断是否有携带包
                                                if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                    drone(x-1,17,z)=1;%锁异环
                                                    drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                    drone(x,15,z)=T;%更新开始投递时间
                                                    drone(x,14,z)=T;%开始产生投递数据的时间
                                                    drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                            break;
                                                        else
                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x-1,j,z) ==0
                                                                    drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                    break;
                                                                end
                                                                j=j+1;
                                                            end
                                                        end
                                                        i=i+1;
                                                    end
                                                    subplot(2,3,1)%定位窗口位置
                                                    b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                end
                                            end
                                        end
                                    end
                                else
                                    if drone(x,13,z)==1%顺时针方向标

                                        if drone(x,10,z)>0
                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                            drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                            drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                            if drone(x,10,z)>0
                                                if T-drone(x,15,z)>=St_ttCoutside
                                                    drone(x,15,z)=T;
                                                    drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            delete(drone(x,19,z)) %删除投递无线电波

                                                        end
                                                    else
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                        if drone(x,8,z) ==0 %投递结束
                                                            drone(x,10,z)=0;%drone投递时间刚好结束
                                                            drone(x,16,z) = T;%更新投递结束时间
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                            delete(drone(x,19,z)) %删除投递无线电波
                                                        end
                                                    end
                                                end
                                            else
                                                if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                            node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                            dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                            drone_node(x,i,z) =0; %node目录点置零
                                                            break;%已经传完一个包
                                                        end
                                                        i=i+1;
                                                    end
                                                end

                                                if x~=1 %非内环
                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                end
                                                drone(x,8,z) = 0;%携带包的个数置零
                                                drone(x,16,z) = T;%更新投递结束时间

                                                i=1;
                                                while i<=Node
                                                    if drone_node(x,i,z) ~=0
                                                        drone_node(x,i,z) =0; %node目录置零
                                                        break;
                                                    end
                                                    i=i+1;
                                                end
                                                delete(drone(x,19,z)) %删除投递无线电波
                                            end
                                        else
                                            if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                if drone(x,8,z)>0 %判断是否有携带包
                                                    if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                        drone(x-1,17,z)=1;%锁异环
                                                        drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                        drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                        drone(x,15,z)=T;%更新开始投递时间
                                                        drone(x,14,z)=T;%开始产生投递数据的时间
                                                        drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                break;
                                                            else
                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x-1,j,z) ==0
                                                                        drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                        break;
                                                                    end
                                                                    j=j+1;
                                                                end
                                                            end
                                                            i=i+1;
                                                        end
                                                        subplot(2,3,1)%定位窗口位置
                                                        b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                        drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                    else
                                                        drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                    end
                                                else
                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)-dcell;%顺时针飞，更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    if drone(x,1,z)<=zmin && zmin<=drone(x,2,z) %到达最右端
                                                        drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                        drone(x,13,z)=0;%c置零，顺时针没有数据
                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end
                                                    end
                                                end
                                            else
                                                drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                drone(x,1,z)=drone(x,1,z)-dcell;%顺时针飞，更新现在位置
                                                subplot(2,3,1)
                                                b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                delete(drone(x,4,z));%删除上一跳drone
                                                drone(x,4,z)=b;%更新现在跳点坐标位置
                                                drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                wehave=0;
                                                if drone(x,1,z)<=zmin && zmin<=drone(x,2,z) %到达最右端
                                                    drone(x,1,z)=zmin;%由于圆的航程是无尽小数，修正位置
                                                    drone(x,13,z)=0;%c置零，顺时针没有数据
                                                    i=1;
                                                    while i<=Node
                                                        if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                            wehave=1;
                                                            node(i,5)=T;%更新遭遇时间
                                                            drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                            drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x,j,z) ==0
                                                                    drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                    break
                                                                end
                                                                j=j+1;
                                                            end
                                                            %  break;
                                                        end
                                                        i=i+1;
                                                    end
                                                end
                                                if wehave==0
                                                    if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                        if drone(x,8,z)>0 %判断是否有携带包
                                                            if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                drone(x-1,17,z)=1;%锁异环
                                                                drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                drone(x,15,z)=T;%更新开始投递时间
                                                                drone(x,14,z)=T;%开始产生投递数据的时间
                                                                drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                        break;
                                                                    else
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x-1,j,z) ==0
                                                                                drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                break;
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                subplot(2,3,1)%定位窗口位置
                                                                b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    else
                                        if drone(x,1,z) < drone(x,3,z) %处于右端逆时针往回飞状态
                                            if drone(x,10,z)>0
                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                if drone(x,10,z)>0
                                                    if T-drone(x,15,z)>=St_ttCoutside
                                                        drone(x,15,z)=T;
                                                        drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                delete(drone(x,19,z)) %删除投递无线电波

                                                            end
                                                        else
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                            if drone(x,8,z) ==0 %投递结束
                                                                drone(x,10,z)=0;%drone投递时间刚好结束
                                                                drone(x,16,z) = T;%更新投递结束时间
                                                                drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                delete(drone(x,19,z)) %删除投递无线电波
                                                            end
                                                        end
                                                    end
                                                else
                                                    if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                drone_node(x,i,z) =0; %node目录点置零
                                                                break;%已经传完一个包
                                                            end
                                                            i=i+1;
                                                        end
                                                    end

                                                    if x~=1 %非内环
                                                        drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                    end
                                                    drone(x,8,z) = 0;%携带包的个数置零
                                                    drone(x,16,z) = T;%更新投递结束时间

                                                    i=1;
                                                    while i<=Node
                                                        if drone_node(x,i,z) ~=0
                                                            drone_node(x,i,z) =0; %node目录置零
                                                            break;
                                                        end
                                                        i=i+1;
                                                    end
                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                end
                                            else
                                                if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                    if drone(x,8,z)>0 %判断是否有携带包
                                                        if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                            drone(x-1,17,z)=1;%锁异环
                                                            drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                            drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                            drone(x,15,z)=T;%更新开始投递时间
                                                            drone(x,14,z)=T;%开始产生投递数据的时间
                                                            drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                    break;
                                                                else
                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x-1,j,z) ==0
                                                                            drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                            break;
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                end
                                                                i=i+1;
                                                            end
                                                            subplot(2,3,1)%定位窗口位置
                                                            b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                            drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                        else
                                                            drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                        end
                                                    else
                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)+dcell;%逆时针往回飞，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                        if drone(x,2,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,1,z)%到达起点
                                                            drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置

                                                            if zmax==drone(x,3,z)  %最左端无目标节点
                                                                drone(x,11,z)=0;%a置零
                                                                drone(x,12,z)=0;%b置零
                                                            end

                                                        end
                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end
                                                    end
                                                else

                                                    drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                    drone(x,1,z)=drone(x,1,z)+dcell;%逆时针往回飞，更新现在位置
                                                    subplot(2,3,1)
                                                    b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                    delete(drone(x,4,z));%删除上一跳drone
                                                    drone(x,4,z)=b;%更新现在跳点坐标位置
                                                    drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                    if drone(x,2,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,1,z)%到达起点
                                                        drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置

                                                        if zmax==drone(x,3,z)  %最左端无目标节点
                                                            drone(x,11,z)=0;%a置零
                                                            drone(x,12,z)=0;%b置零

                                                        end
                                                    end
                                                    wehave=0;

                                                    i=1;
                                                    while i<=Node
                                                        if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                            wehave=1;
                                                            node(i,5)=T;%更新遭遇时间
                                                            drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                            drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                            j=1;
                                                            while j<=Node
                                                                if drone_node(x,j,z) ==0
                                                                    drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                    break
                                                                end
                                                                j=j+1;
                                                            end
                                                            %  break;
                                                        end
                                                        i=i+1;
                                                    end

                                                    if wehave==0
                                                        if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                            if drone(x,8,z)>0 %判断是否有携带包
                                                                if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                    drone(x-1,17,z)=1;%锁异环
                                                                    drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                    drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                    drone(x,15,z)=T;%更新开始投递时间
                                                                    drone(x,14,z)=T;%开始产生投递数据的时间
                                                                    drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                    i=1;
                                                                    while i<=Node
                                                                        if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                            break;
                                                                        else
                                                                            j=1;
                                                                            while j<=Node
                                                                                if drone_node(x-1,j,z) ==0
                                                                                    drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                    break;
                                                                                end
                                                                                j=j+1;
                                                                            end
                                                                        end
                                                                        i=i+1;
                                                                    end
                                                                    subplot(2,3,1)%定位窗口位置
                                                                    b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                    drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        else
                                            if drone(x,11,z) ==1%a为1，表明正在左端逆时针飞行
                                                if drone(x,10,z)>0
                                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                    drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                    drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                    if drone(x,10,z)>0
                                                        if T-drone(x,15,z)>=St_ttCoutside
                                                            drone(x,15,z)=T;
                                                            drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                            if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                        dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    delete(drone(x,19,z)) %删除投递无线电波

                                                                end
                                                            else
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                                end
                                                            end
                                                        end
                                                    else
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                        end

                                                        if x~=1 %非内环
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                        end
                                                        drone(x,8,z) = 0;%携带包的个数置零
                                                        drone(x,16,z) = T;%更新投递结束时间

                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0
                                                                drone_node(x,i,z) =0; %node目录置零
                                                                break;
                                                            end
                                                            i=i+1;
                                                        end
                                                        delete(drone(x,19,z)) %删除投递无线电波
                                                    end
                                                else
                                                    if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                        if drone(x,8,z)>0 %判断是否有携带包
                                                            if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                                drone(x-1,17,z)=1;%锁异环
                                                                drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                drone(x,15,z)=T;%更新开始投递时间
                                                                drone(x,14,z)=T;%开始产生投递数据的时间
                                                                drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                        break;
                                                                    else
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x-1,j,z) ==0
                                                                                drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                break;
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                subplot(2,3,1)%定位窗口位置
                                                                b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                            else
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                            end
                                                        else
                                                            drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                            drone(x,1,z)=drone(x,1,z)+dcell;%逆时针往回飞，更新现在位置
                                                            subplot(2,3,1)
                                                            b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                            delete(drone(x,4,z));%删除上一跳drone
                                                            drone(x,4,z)=b;%更新现在跳点坐标位置
                                                            drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                            if drone(x,1,z)>=zmax && drone(x,2,z)<=zmax  %到达最左端
                                                                drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                                drone(x,11,z)=0;%a置零，逆时针方向已经没有数据
                                                                i=1;
                                                                while i<=Node
                                                                    if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                        node(i,5)=T;%更新遭遇时间
                                                                        drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                        drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x,j,z) ==0
                                                                                drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                                break
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                        %  break;
                                                                    end
                                                                    i=i+1;
                                                                end
                                                            end
                                                        end
                                                    else

                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)+dcell;%逆时针往回飞，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少


                                                        wehave=0;
                                                        if drone(x,1,z)>=zmax && drone(x,2,z)<=zmax  %到达最左端
                                                            drone(x,1,z)=zmax;%由于圆的航程是无尽小数，修正位置
                                                            drone(x,11,z)=0;%a置零，逆时针方向已经没有数据

                                                            i=1;
                                                            while i<=Node
                                                                if (drone(x,2,z)<=node(i,1) && node(i,1)<=drone(x,1,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                    wehave=1;
                                                                    node(i,5)=T;%更新遭遇时间
                                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x,j,z) ==0
                                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                            break
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                    %  break;
                                                                end
                                                                i=i+1;
                                                            end
                                                        end

                                                        if wehave==0
                                                            if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                                if drone(x,8,z)>0 %判断是否有携带包
                                                                    if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                        drone(x-1,17,z)=1;%锁异环
                                                                        drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                        drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                        drone(x,15,z)=T;%更新开始投递时间
                                                                        drone(x,14,z)=T;%开始产生投递数据的时间
                                                                        drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                        i=1;
                                                                        while i<=Node
                                                                            if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                                break;
                                                                            else
                                                                                j=1;
                                                                                while j<=Node
                                                                                    if drone_node(x-1,j,z) ==0
                                                                                        drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                        break;
                                                                                    end
                                                                                    j=j+1;
                                                                                end
                                                                            end
                                                                            i=i+1;
                                                                        end
                                                                        subplot(2,3,1)%定位窗口位置
                                                                        b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                        drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                    end
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            else
                                                if drone(x,10,z)>0
                                                    drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                    drone(x,7,z)=drone(x,7,z)-St_PTransmit;%传输数据则能量减少
                                                    drone(x,10,z)=drone(x,10,z)-St;%传输数据倒计时减少St时刻

                                                    if drone(x,10,z)>0
                                                        if T-drone(x,15,z)>=St_ttCoutside
                                                            drone(x,15,z)=T;
                                                            drone(x,8,z)=drone(x,8,z)-1;%携带包的个数减一
                                                            if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                        dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    delete(drone(x,19,z)) %删除投递无线电波

                                                                end
                                                            else
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                        drone_node(x,i,z) =0; %node目录点置零
                                                                        break;%已经传完一个包
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                if drone(x,8,z) ==0 %投递结束
                                                                    drone(x,10,z)=0;%drone投递时间刚好结束
                                                                    drone(x,16,z) = T;%更新投递结束时间
                                                                    drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                                    delete(drone(x,19,z)) %删除投递无线电波
                                                                end
                                                            end
                                                        end
                                                    else
                                                        if x==1 %判断是否是内环，内环则更新节点的投递结束时间
                                                            i=1;
                                                            while i<=Node
                                                                if drone_node(x,i,z) ~=0 %node被投递到目的地
                                                                    node(drone_node(x,i,z),8)=T;%更新投递结束时间
                                                                    dread = [dread;drone_node(x,i,z)];%记录总共投递成功节点数量
                                                                    drone_node(x,i,z) =0; %node目录点置零
                                                                    break;%已经传完一个包
                                                                end
                                                                i=i+1;
                                                            end
                                                        end

                                                        if x~=1 %非内环
                                                            drone(x-1,17,z) =0;%被外环锁定标志置零，释放异环
                                                        end
                                                        drone(x,8,z) = 0;%携带包的个数置零
                                                        drone(x,16,z) = T;%更新投递结束时间

                                                        i=1;
                                                        while i<=Node
                                                            if drone_node(x,i,z) ~=0
                                                                drone_node(x,i,z) =0; %node目录置零
                                                                break;
                                                            end
                                                            i=i+1;
                                                        end
                                                        delete(drone(x,19,z)) %删除投递无线电波
                                                    end
                                                else
                                                    if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                        if drone(x,8,z)>0 %判断是否有携带包
                                                            if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                                drone(x-1,17,z)=1;%锁异环
                                                                drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                drone(x,15,z)=T;%更新开始投递时间
                                                                drone(x,14,z)=T;%开始产生投递数据的时间
                                                                drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                i=1;
                                                                while i<=Node
                                                                    if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                        break;
                                                                    else
                                                                        j=1;
                                                                        while j<=Node
                                                                            if drone_node(x-1,j,z) ==0
                                                                                drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                break;
                                                                            end
                                                                            j=j+1;
                                                                        end
                                                                    end
                                                                    i=i+1;
                                                                end
                                                                subplot(2,3,1)%定位窗口位置
                                                                b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                            else
                                                                drone(x,7,z)=drone(x,7,z)-St_Ph;%悬停能量减少
                                                            end
                                                        else
                                                            drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                            drone(x,1,z)=drone(x,1,z)-dcell;%逆时针往回飞，更新现在位置
                                                            subplot(2,3,1)
                                                            b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                            delete(drone(x,4,z));%删除上一跳drone
                                                            drone(x,4,z)=b;%更新现在跳点坐标位置
                                                            drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                            if drone(x,1,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,2,z)%到达起点
                                                                drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置
                                                                drone(x,12,z)=0;%b置零
                                                            end

                                                            i=1;
                                                            while i<=Node
                                                                if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                    node(i,5)=T;%更新遭遇时间
                                                                    drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                    drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                    j=1;
                                                                    while j<=Node
                                                                        if drone_node(x,j,z) ==0
                                                                            drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                            break
                                                                        end
                                                                        j=j+1;
                                                                    end
                                                                    %  break;
                                                                end
                                                                i=i+1;
                                                            end
                                                        end
                                                    else

                                                        drone(x,2,z)=drone(x,1,z);%更新上一跳位置
                                                        drone(x,1,z)=drone(x,1,z)-dcell;%逆时针往回飞，更新现在位置
                                                        subplot(2,3,1)
                                                        b=plot(dr(x)*cos(drone(x,1,z)),dr(x)*sin(drone(x,1,z)),'^','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone最新位置
                                                        delete(drone(x,4,z));%删除上一跳drone
                                                        drone(x,4,z)=b;%更新现在跳点坐标位置
                                                        drone(x,7,z)=drone(x,7,z)-St_P(x);%飞行则能量减少

                                                        if drone(x,1,z)<=drone(x,3,z) && drone(x,3,z)<=drone(x,2,z)%到达起点
                                                            drone(x,1,z)=drone(x,3,z);%由于圆的航程是无尽小数，修正位置
                                                            drone(x,12,z)=0;%b置零

                                                        end

                                                        wehave=0;
                                                        i=1;
                                                        while i<=Node
                                                            if (drone(x,1,z)<=node(i,1) && node(i,1)<=drone(x,2,z)) && node(i,3)==z && node(i,4)==x  && node(i,9)==1
                                                                wehave=1;
                                                                node(i,5)=T;%更新遭遇时间
                                                                drone(x,8,z)=drone(x,8,z)+1;%更新携带数据包个数
                                                                drone(x,9,z)=St_tphoto;%更新拍摄数据倒计时

                                                                j=1;
                                                                while j<=Node
                                                                    if drone_node(x,j,z) ==0
                                                                        drone_node(x,j,z)=i;%更新drone_node的list中的包
                                                                        break
                                                                    end
                                                                    j=j+1;
                                                                end
                                                                %  break;
                                                            end
                                                            i=i+1;
                                                        end

                                                        if wehave==0
                                                            if((drone(x,2,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,1,z))||(drone(x,1,z)<=drone(x-1,1,z) && drone(x-1,1,z)<=drone(x,2,z)))%假如遭遇异环
                                                                if drone(x,8,z)>0 %判断是否有携带包
                                                                    if drone(x-1,9,z)<=0 && drone(x-1,10,z)<=0 %异环无数据传输操作，处于自由状态
                                                                        drone(x-1,17,z)=1;%锁异环
                                                                        drone(x,10,z)=drone(x,8,z)*St_ttCoutside;%外环投递倒计时等于携带包个数X每个包的投递时间
                                                                        drone(x,18,z)=drone(x,18,z)+drone(x,8,z);%累加已经投递过的包
                                                                        drone(x,15,z)=T;%更新开始投递时间
                                                                        drone(x,14,z)=T;%开始产生投递数据的时间
                                                                        drone(x-1,8,z)=drone(x-1,8,z)+drone(x,8,z);%更新异环的携带包数量
                                                                        i=1;
                                                                        while i<=Node
                                                                            if drone_node(x,i,z) ==0 %drone_node的list中的包传输结束
                                                                                break;
                                                                            else
                                                                                j=1;
                                                                                while j<=Node
                                                                                    if drone_node(x-1,j,z) ==0
                                                                                        drone_node(x-1,j,z)=drone_node(x,i,z);%将drone_node的list中的包传给异环
                                                                                        break;
                                                                                    end
                                                                                    j=j+1;
                                                                                end
                                                                            end
                                                                            i=i+1;
                                                                        end
                                                                        subplot(2,3,1)%定位窗口位置
                                                                        b=plot([dr(x)*cos(drone(x,1,z)),dr(x-1)*cos(drone(x-1,1,z))],[dr(x)*sin(drone(x,1,z)),dr(x-1)*sin(drone(x-1,1,z))],'g-');%画出无线电传输线
                                                                        drone(x,19,z)=b;%无线电传输线存储，方便投递结束删除
                                                                    end
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
                while drone(x,7,z) <=0 %能量耗尽，新无人机接力
                    drone(x,7,z) = drone(x,7,z)+Jer;%更新能量
                    dronenumber=dronenumber+1;% 已用无人机数量+1
                    if drone(x,7,z)>0
                        break;
                    end
                end
                x=x+1;
            end

            z=z+1;
        end


        i=1;
        while i<=Node
            if node(i,8) ~=0
                if node(i,8)==T%有新节点到达目的地
                    dhave=[dhave;i];

                    if t ~=1
                        if t~=kklcopy
                            kklcopy=t;
                            kkl=kkl+kkj;
                        end
                    end

                    j=1;
                    kk=0;
                    kkj=0;
                    while  j <= Node     %查找每个节点
                        if node(j,8)>0 %投递结束
                            kkj=kkj+(node(j,8)-node(j,5));
                            kk=kk+1;
                        end
                        j=j+1;
                    end

                    kk=(kklcopy-1)*Node+kk;
                    Node_aoi=(kkl+kkj)/kk;
                    subplot(2,3,2) % 定位窗口
                    plot([Node_aoi_T*S_St/St,T*S_St/St],[Node_aoi_copy*S_St/St,Node_aoi*S_St/St],'g-');% 绘制已到达目的地节点在到达目的地时刻的平均Aoi
                    Node_aoi_copy=Node_aoi;
                    Node_aoi_T=T;

                    if T_aoi_node ==0%首个节点到达目的地
                        subplot(2,3,3) % 定位窗口
                        plot([0,T*S_St/St],[0,(T-T_aoi_node)*S_St/St],'g-');% 绘制即时Aoi
                        plot([T*S_St/St,T*S_St/St],[(T-T_aoi_node)*S_St/St,(T-node(i,5))*S_St/St],'g-');% 绘制即时Aoi
                        T_aoi=T;
                        T_aoi_node=node(i,5);
                    else
                        subplot(2,3,3) % 定位窗口
                        plot([T_aoi*S_St/St,T*S_St/St],[(T_aoi-T_aoi_node)*S_St/St,(T-T_aoi_node)*S_St/St],'g-');% 绘制即时Aoi
                        plot([T*S_St/St,T*S_St/St],[(T-T_aoi_node)*S_St/St,(T-node(i,5))*S_St/St],'g-');% 绘制即时Aoi
                        T_aoi=T;
                        T_aoi_node=node(i,5);
                    end

                end
            end
            i=i+1;
        end


        if dronenumbercope~=dronenumber
            dronenumbercope=dronenumber;
            subplot(2,3,5) % 定位窗口
            plot(t,dronenumber,'*','color',[0 0 1], 'MarkerSize', 5 );% 绘制已用无人机数量
            DD=[DD;dronenumber];
        end
        if t ~=1
            if t~=tcopy
                tcopy= t;
                subplot(2,3,5) % 定位窗口
                plot(t,dronenumber,'*','color',[0 0 1], 'MarkerSize', 5 );% 绘制已用无人机数量
                DD=[DD;dronenumber];
            end
        end

        number=0;
        o=1;
        while  o <= Node     %查找每个节点
            if node(o,8)>0 %投递结束
                number=number+1;
            end
            o=o+1;
        end
        ddao=0;
        oo=1;
        while  oo <= Sm     %查找每个扇
            ss=1;
            while  ss <= circle     %查找每个环
                if drone(ss,1,oo)==drone(ss,3,oo) %无人机归位
                    ddao=ddao+1;
                end
                ss=ss+1;
            end
            oo=oo+1;
        end
        if number ==Node && ddao==Sm*circle
            time_each_turn=[time_each_turn;T];
            subplot(2,3,4) % 定位窗口
            plot([t,t],[(T-RT)*S_St/St,0],'r-');% 绘制周期耗时
            plot(t,(T-RT)*S_St/St,'*','color',[0 0 1], 'MarkerSize', 5 );% 绘制周期耗时
            TD=[TD;(T-RT)*S_St/St];
            RT=T;

            i=1;
            energy=0;
            while  i <= Sm     %查找每个扇区
                j=1;
                while  j <= circle     %查找每个环
                    energy=energy+(Jer-drone(j,7,i));%累加每个环的drone已经使用的能量
                    j=j+1;
                end
                i=i+1;
            end
            energy=energy+(dronenumber-Sm*circle)*Jer;
            energy_mid=energy;
            energy=energy-energy_before;
            subplot(2,3,6) % 定位窗口
            plot(t,energy,'*','color',[0 0 1], 'MarkerSize', 5 );% 绘制drone已用能量
            ED=[ED;energy];
            energy_before=energy_mid;

            break; %上一轮收集数据完毕，准备进入下一轮
        end

    end

    if t<=turn-1
        i=1;
        while i<=Sm
            j=1;
            while j<=circle
                delete(drone(j,4,i))
                j=j+1;
            end
            i=i+1;
        end
    end
    dy=[dy;node];
    t=t+1;
end

save Drones_Trajectory_new A air_dencity avr b bandwidth blade_angular_velocity c cc cell Cinside circle Coutside dcell dg dhave dp dr drag dread drone drone_node dronenumber dronenumbercope dx dy ee energy energy_before energy_mid fuselage_drag_ratio g Gr Gt i j Jer k kkj kk kkl kklcopy liuliang m max_ni min_shun n neck need node Node Node_aoi Node_aoi_copy Node_aoi_T Noise_power number o P0 Ph Pi Pphoto pr profile_drag_coefficient Pt Pv r rotor_radius rotor_solidity RT S_St shan SL Sm Sr SR St St_P St_Ph St_Pphoto St_PTransmit St_tphoto St_ttCinside St_ttCoutside t T T_aoi T_aoi_node tCinside tcopy tCoutside theta time_each_turn tphoto turn u Utip V V0 w W Wavelength x z zmax zmin TD ED DD
end
