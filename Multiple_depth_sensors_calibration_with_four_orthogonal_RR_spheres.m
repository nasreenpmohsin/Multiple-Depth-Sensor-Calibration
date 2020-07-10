% Author: Nasreen Mohsin
% YEar: 2018
% Calibration of three Kinect Depth Sensors thorugh four retro-reflective
% spheres.
% The data (RGB, Depth, IR, registered (RGB-depth)) is captured and
% recorded from three Kinect sensors through libfreenect2. The data is
% saved in the folder set66_Prpexp
close all;
clear workspace;
% load('C:\Users\Nachu\Documents\thesis papers\journal2017_v3\set60_non_mov_exp3\CalibrationResults\extracted_images_into_array.mat');
load('C:\Users\Nachu\Documents\thesis papers\journal2017_v3\set66_Prpexp\CalibrationResults\exp_dep_ir_reg.mat');
global C
global C1
% global C1
% global disY
% global disX
% global disZ
% global tradius

% 4 spheres were kept on big board with wheels. 
% The retro-reflective strips around sphere should be parallel to tuntable plane. 
% The turnable table spawns differnt positions for the calibration setup.

% set='set66_Prpexp'; % set folder
%% extracting data into workspace
%%depth images
c={'cam0','cam1','cam2'};
filerange=1:257;%file selecting set of data from 3 sensors
srcFiles = strsplit(num2str(filerange),' ');
dep = cell(2,length(srcFiles));
ir = cell(2,length(srcFiles));
reg = cell(2,length(srcFiles));
Prefixe_folder='Multiple_depth_sensor_calibration';
for cam=1:3
% the folder in which ur images exists
Resultados=[Prefixe_folder '\' set '\' c{cam} '\depth1\'];
Resultados1=[Prefixe_folder '\' set '\' c{cam} '\ir1\'];
mkdir(Resultados);
mkdir(Resultados1);
for i = 1 : length(srcFiles)%4
    %Depth
    filename = strcat([Prefixe_folder '\' set '\' c{cam} '\depth\'],srcFiles{i},'.txt');
    fid=fopen(filename,'r');
    dep{cam,i}=(fscanf(fid,'%u',[512,424]))';
    fclose(fid);
    baseFileName = sprintf('%d.png', i); % e.g. "1.png"
    fullFileName = fullfile(Resultados, baseFileName); 
    imwrite(dep{cam,i}./4500, fullFileName);
    %IR
    filename1 = strcat([Prefixe_folder '\' set '\' c{cam} '\ir\'],srcFiles{i},'.txt');
    fid1=fopen(filename1,'r');
    ir{cam,i}=(fscanf(fid1,'%u',[512,424]))';  
    fclose(fid1);
    fullFileName1 = fullfile(Resultados1, baseFileName); 
    imwrite(ir{cam,i}./65000, fullFileName1);
    %Registerd depth image
    filename2 = strcat([Prefixe_folder '\' set '\' c{cam} '\registered\'],srcFiles{i},'.jpg');
    reg{cam,i} = imread(filename2);
end
end

%%%%%%%%%%%%%%%%%%
%%%%diameter Origin, X, Y, Z
 d3D=mean([47.4,48.2,47.3,47.9])/pi;% taking average of diameters of four spheres (nearly same diameter)
 d3D=d3D/100;%converting from cm to m
 %% Intrinsic parameters in three kinect sensors
fx=[367.214508057,365.7945861816406,365.377105713];
fy=[367.214508057,365.7945861816406,365.377105713];
cx=[256.831695557,255.1743927001953,258.203308105];
cy=[208.951904297,206.6826934814453,205.853302002];

ob_set =[4,100,214,241]; % selected framesets
ob_l=length(ob_set);%No of selected framesets
Spheres=cell(4,3,ob_l);%Point Cloud of four spheres viewed from three Kinect senosrs over ob_l framesets
T=cell(4,3,ob_l);
sIR=cell(4,3,ob_l);%IR images of four spheres viewed from three Kinect senosrs over ob_l framesets
im_d=zeros(4,1,3,ob_l); % diameter mesurments (pixels) of white strips of spheres in IR images from 3 kinect sensors
C_pos=zeros(4,3,3,ob_l);% set of 3D positions of spherical centres extracted from IR data
cm=zeros(4,2,3,ob_l); % set of 2D positions of spherical centres extracted from IR data
sDep=cell(4,3,ob_l);% set of segmented depth information of spheres from each frame of each kinect sensor
DC_pos=zeros(4,3,3,ob_l); % set of 3D positions of spherical centres extracted from deoth data
dcm=zeros(4,2,3,ob_l); % set of 2D positions of spherical centres extracted from deoth data
dia=zeros(4,1,3,ob_l); % set of estimated diameters of spheres from depth information through MNSAC
thresh=0.5; % threshold for 
thresh1=0.2;
%% start of loop of sets
% dep=dep./4500;
D=cell(3);R=cell(3);Rg=cell(3);
for i=1:ob_l;
D{1}=dep{1,ob_set(i)}./1000;D{2}=dep{2,ob_set(i)}./1000;D{3}=dep{3,ob_set(i)}./1000;
R{1}=ir{1,ob_set(i)}./65536;R{2}=ir{2,ob_set(i)}./65536; R{3}=ir{3,ob_set(i)}./65536; 
Rg{1}=reg{1,ob_set(i)};Rg{2}=reg{2,ob_set(i)};Rg{3}=reg{3,ob_set(i)};
% figure(1);imshow(D{1}./4.5);
%  figure(2);imshow(D{2}./4.5);
%  figure(3);imshow(D{3}./4.5);
% figure(4);imshow(R{1});
% figure(5);imshow(R{2});
% figure(6);imshow(R{3});
% figure(7);imshow(Rg{1});
% figure(8);imshow(Rg{2});
% figure(9);imshow(Rg{3});
 se1 = strel('disk',1);
for c=1:3
    Rd=imguidedfilter(R{c},'NeighborhoodSize',[2 2]); %medfilt2(R, [3 2]) 
    se = strel('disk',10);
    Rd = imclose(Rd,se);
    % figure(5);imshow(Rd);
    Td=Rd;
% T1(T1<200)=0;T1(T1>200)=255;
    Tbw=Td>thresh;% segmentation of RR strip in IR image through thresholding: 
    Tbw = imfill(Tbw, 'holes');
    Tc = bwconncomp(Tbw);
    numPixels = cellfun(@numel,Tc.PixelIdxList);
    Tbw = bwlabel(Tbw, 8);
%     figure(10);imshow(Tbw);
    [group,idx] = sort(numPixels,'descend');
    sidx=sort(idx(1:4),'ascend');
%  figure;
  for j=1:4
    T{j,c,i}=Tbw==sidx(j);%idx(j);
    sIR{j,c,i}=Rd;
    sIR{j,c,i}(~T{j,c,i})=0;
%     [dy,dx,vl]=find(sIR{j,c,i});
    sIR{j,c,i}=imdilate(sIR{j,c,i},se1);
    % corner dection to extract the strip ends 
    Crn_curve=corner(sIR{j,c,i},2); 
%     figure;imshow(sIR{j,c,i});hold on; plot(Crn_curve(:,1),Crn_curve(:,2),'b*');
    dm= pdist(Crn_curve);%max(dx)-min(dx);
    im_d(j,1,c,i)=dm;
%     xm=min(dx)+dm/2;
%     ym=min(dy)+(max(dy)-min(dy))/2;
%mid-point and distance between strip ends act as projected centre and diameter of spherical projection for IR-based process
      cm(j,1:2,c,i) =sum(Crn_curve,1)./2; % mid-point of strip as projected cntre of sphere
%       hold on; plot(cm(j,1,c,i),cm(j,2,c,i),'r*');hold off;
% figure();imshow(T{j,c,i});
% hold on; plot(xm,ym,'r*');
% hold off;
    Z_c=d3D*(fx(c))/dm; % assuming pin-hole camera setup
    X_c=(cm(j,1,c,i)-cx(c))*Z_c/fx(c);
    Y_c=(cm(j,2,c,i)-cy(c))*Z_c/fy(c);
    C_pos(j,1:3,c,i)=[X_c,Y_c,Z_c];
%     cm(j,1:2,c,i)=[xm,ym];

%Segment the sphere
    S=sIR{j,c,i};
    %formation of circular binary mask from RR strip
    S=rgb2gray(insertShape(S,'FilledCircle',[cm(j,1,c,i) cm(j,2,c,i) dm/2],'Color','white' ));
    S=S>thresh1;%segmentation of spherical projection in depth data through binary mask
    sDep{j,c,i}=D{c};
    sDep{j,c,i}(~S)=0; %  masking
 
    [row,col,v] = find(sDep{j,c,i});
    X=((col-cx(c)).*v)./fx(c);
    Y=((row-cy(c)).*v)./fy(c);
    Z=v;
    temp=pcdenoise(pointCloud([X(:),Y(:),Z(:)]),'NumNeighbors',50,'Threshold',0.7);
    X=temp.Location(:,1);Y=temp.Location(:,2);Z=temp.Location(:,3);
    % estimate spherical centre from 2.5 depth information
    [estimateCenter,inliers_set,outliers_set,mnsac_center,mnsac_radius]=spherefit(X(:),Y(:),Z(:),d3D/2);
    dia(j,1,c,i)=mnsac_radius*2;
    DC_pos(j,1:3,c,i)=estimateCenter;
    Cx=((estimateCenter(1).*fx(c))./estimateCenter(3))+cx(c);
    Cy=((estimateCenter(2).*fy(c))./estimateCenter(3))+cy(c);
    dcm(j,1:2,c,i)=[Cx,Cy];
%      hold on;
%      imshow(sDep{j,c,i}./4.5);
%     hold on; plot(Cx,Cy,'r*');
    Spheres{j,c,i}=[X(inliers_set),Y(inliers_set),Z(inliers_set)];
  end
  hold off;
end
end

%%% MANAUL XYZO SELECTION for method through depth images
%
SpherePos=zeros(4,3,3,ob_l);
SpherePosIR=zeros(4,3,3,ob_l);
Spheres1=cell(4,3,ob_l);

pk=[2,3,4,4,2,1,3,1,3,1,4,2;
    2,3,3,3,1,4,1,4,2,4,2,1;
    4,2,3,1,4,1,3,3,2,2,1,4;
    3,2,2,1,4,3,4,3,1,2,1,4];
%set66
% 4:
% O-2,X-4,Y-3,Z-1
% O-3,X-2,Y-1,Z-4
% O-4,X-1,Y-3,Z-2
% 100:
% O-2,X-3,Y-1,Z-4
% O-3,X-1,Y-4,Z-2
% O-3,X-4,Y-2,Z-1
% 214:
% O-4,X-1,Y-3,Z-2
% O-2,X-4,Y-3,Z-1
% O-3,X-1,Y-2,Z-4
% 241:
% O-3,X-1,Y-4,Z-2
% O-2,X-4,Y-3,Z-1
% O-2,X-3,Y-1,Z-4

for w=1:ob_l
    i=1;
    for j=1:4
        for c=1:3
                SpherePos(j,1:3,c,w)=DC_pos(pk(w,i),1:3,c,w);
%                 SpherePosIR(j,1:2,c,w)=C_pos(pk(w,i),1:2,c,w);
%                 SpherePosIR(j,3,c,w)=DC_pos(pk(w,i),3,c,w);
                SpherePosIR(j,1:3,c,w)=C_pos(pk(w,i),1:3,c,w);
                Spheres1{j,c,w}=Spheres{pk(w,i),c,w};
                i=i+1;
        end
    end
end

% 
% for j=1:2
%     C1=cell(4,1);
%     % points
%     for k=1:4
%         C1{k}=Spheres1{k,j};
%         %co-efficeints
%         sph=mean([SpherePosR(k,1:3,j);SpherePosD(k,1:3,j)],1);
%         Z=[Z,sph];
%     end
%             options = optimset('Algorithm','levenberg-marquardt','TolX',0.000000001,'TolFun',0.000000001,  ... 
%          'Jacobian','off','MaxIter',60000,'MaxFunEvals',60000,'Display','off'); 
%         Z=double(Z);
%         [NLS,RESNORM,RESIDUAL_i] = lsqnonlin(@compare_dist,Z,[],[],options);
%         for k=1:4
%         SpherePos(k,1:3,j)=NLS((k-1)*3+1:(k-1)*3+3);
%         end
% end


Rot=cell(3,ob_l);
t=cell(3,ob_l);
Ortho=zeros(3,ob_l);

Rot_IR=cell(3,ob_l);
t_IR=cell(3,ob_l);
Ortho_IR=zeros(3,ob_l);

%%
for w=1:ob_l
for c=1:3
% figure();
% plot3(SpherePos(1:2,1,c,w),SpherePos(1:2,2,c,w),SpherePos(1:2,3,c,w),'-g');
% hold on;
% plot3([SpherePos(1,1,c,w);SpherePos(3,1,c,w)],[SpherePos(1,2,c,w);SpherePos(3,2,c,w)],[SpherePos(1,3,c,w);SpherePos(3,3,c,w)],'-r');
% hold on;
% plot3([SpherePos(1,1,c,w);SpherePos(4,1,c,w)],[SpherePos(1,2,c,w);SpherePos(4,2,c,w)],[SpherePos(1,3,c,w);SpherePos(4,3,c,w)],'-b');
% xlabel('X');ylabel('Y');zlabel('Z');
% title(['3 axes from target - Green-X, Red-Y, Blue-Z in Sensor frame ',int2str(c),' for set ' ,int2str(w)]);
% tradius=d3D/2;
% 
%     x0=[];
%     C1=cell(4,1);
%     % points
%     for k=1:4
%         C1{k}=Spheres1{k,j};
%         
%         sph=SpherePos(k,1:3,j);
%         x0=[x0,sph];
%     end
%             options = optimset('Algorithm','levenberg-marquardt','TolX',0.000000001,'TolFun',0.000000001,  ... 
%          'Jacobian','off','MaxIter',600000,'MaxFunEvals',600000,'Display','off'); 
%         %Z=double(Z);
%         x0=double(x0(:));
% %         [NLS,Residual,res1]=lsqnonlin(@ortho_fun,x0,[],[],options);
%         [NLS,Residual,res1]=fminsearch(@ortho_fun,x0,options);
%         for k=1:4
%         SpherePos(k,1:3,j)=NLS((k-1)*3+1:(k-1)*3+3);
%         end
%%Depth
P1=[SpherePos(1,1,c,w);SpherePos(1,2,c,w);SpherePos(1,3,c,w)];
P2=SpherePos(2,1:3,c,w)';
P3=SpherePos(3,1:3,c,w)';
P4=SpherePos(4,1:3,c,w)';
d = P1;
u= P2-d;u=u/norm(u);
v= P3-d;v=v/norm(v);
w1=P4-d;w1=w1/norm(w1);
Rot{c,w} = [u,v,w1]; % Rotation matrix
t{c,w}=d;
Ortho(c,w)=abs(dot(u,v))+ abs(dot(v,w1))+ abs(dot(w1,u));

%%IR
P1=[SpherePosIR(1,1,c,w);SpherePosIR(1,2,c,w);SpherePosIR(1,3,c,w)];
P2=SpherePosIR(2,1:3,c,w)';
P3=SpherePosIR(3,1:3,c,w)';
P4=SpherePosIR(4,1:3,c,w)';
d = P1;
u= P2-d;u=u/norm(u);
v= P3-d;v=v/norm(v);
w1=P4-d;w1=w1/norm(w1);
Rot_IR{c,w} = [u,v,w1]; % Rotation matrix
t_IR{c,w}=d;
Ortho_IR(c,w)=abs(dot(u,v))+ abs(dot(v,w1))+ abs(dot(w1,u));
end
end
% into common frame
Ri=cell(3,1);ti=cell(3,1);
Ri_=cell(3,1);ti_=cell(3,1);
Ri{1}=Rot{1,1};Ri{2}=Rot{2,1};Ri{3}=Rot{3,1};
ti{1}=t{1,1};ti{2}=t{2,1};ti{3}=t{3,1};
Min_orth=Ortho(1:3,1);
C_tilda=zeros(3,4,ob_l);

RiIR=cell(3,1);tiIR=cell(3,1);
RiIR_=cell(3,1);tiIR_=cell(3,1);
RiIR{1}=Rot_IR{1,1};RiIR{2}=Rot_IR{2,1};RiIR{3}=Rot_IR{3,1};
tiIR{1}=t_IR{1,1};tiIR{2}=t_IR{2,1};tiIR{3}=t_IR{3,1};
Min_orthIR=Ortho_IR(1:3,1);
C_tildaIR=zeros(3,4,ob_l);

for c=1:3
    for w=1:ob_l
        if Ortho(c,w)<Min_orth(c)
            Ri{c}=Rot{c,w};
            ti{c}=t{c,w};
            Min_orth(c)=Ortho(c,w);
        end
        if Ortho_IR(c,w)<Min_orthIR(c)
            RiIR{c}=Rot_IR{c,w};
            tiIR{c}=t_IR{c,w};
            Min_orthIR(c)=Ortho_IR(c,w);
        end
    end
    Ri_{c}=Ri{c}';
    ti_{c}=-Ri{c}'*ti{c};
    RiIR_{c}=RiIR{c}';
    tiIR_{c}=-RiIR{c}'*tiIR{c};
end

for w=1:ob_l
    for j=1:4
        for c=1:3
         C_tilda(:,j,w)=C_tilda(:,j,w)+Ri_{c}*SpherePos(j,1:3,c,w)'+ti_{c};
         C_tildaIR(:,j,w)=C_tildaIR(:,j,w)+RiIR_{c}*SpherePosIR(j,1:3,c,w)'+tiIR_{c};
        end
        C_tilda(:,j,w)=C_tilda(:,j,w)./3;
        C_tildaIR(:,j,w)=C_tildaIR(:,j,w)./3;
    end
end
% % c2->P->C1
% t_=-Rot{2}'*t{2};
% R_=Rot{2}';
% % 
% 
% Ri=Rot{1}*R_;
% ti=Rot{1}*t_+t{1};
% 
% %c1->P->C2
% t_1=-Rot{1}'*t{1};
% R_1=Rot{1}';
% % 
% Ri1=Rot{2}*R_1;
% ti1=Rot{2}*t_1+t{2};
% C_tilda=zeros(3,4);
% 
%  %%%
% %for cam1
% DR1=eye(3);Dt1=[0 0 0]';
% %for cam2
% DA=SpherePos(:,:,2)';DB=SpherePos(:,:,1)';
% [regParams,Bfit,ErrorStats]=absor(DA,DB,'doTrans',true);
% Ri=regParams.R;ti=regParams.t;
% 
% for i=1:4
%     C_tilda(:,i)=SpherePos(i,1:3,1)'+((Ri*SpherePos(i,1:3,2)')+ti);
%     C_tilda(:,i)=C_tilda(:,i)./2;
%     
% end
C_cap=C_tilda(:);
C_capIR=C_tildaIR(:);


%%
C=zeros(3,4,3,ob_l);
C1=zeros(3,4,3,ob_l);
for w=1:ob_l
C(:,:,1,w)=SpherePos(:,:,1,w)';
C(:,:,2,w)=SpherePos(:,:,2,w)';
C(:,:,3,w)=SpherePos(:,:,3,w)';

C1(:,:,1,w)=SpherePosIR(:,:,1,w)';
C1(:,:,2,w)=SpherePosIR(:,:,2,w)';
C1(:,:,3,w)=SpherePosIR(:,:,3,w)';
end
Eu=[];tt=[];
EuIR=[];ttIR=[];
for c=1:3
Eu(c,1:3)=SpinCalc('DCMtoEA321',Ri_{c},0.1,0);
tt(1:3,c)=ti_{c};

EuIR(c,1:3)=SpinCalc('DCMtoEA321',RiIR_{c},0.1,0);
ttIR(1:3,c)=tiIR_{c};
end
Eu=Eu(:);
tt=tt(:);

EuIR=EuIR(:);
ttIR=ttIR(:);
% Eu=SpinCalc('DCMtoQ',Ri,0.01,1);

%variables
 Z=[Eu;tt;C_cap];%.*(pi/180)
 ZIR=[EuIR;ttIR;C_capIR];
% Z=[Ri(:);tt;C_cap];
Z=double(Z);
ZIR=double(ZIR);
% options = optimset('LevenbergMarquardt','on','TolX',1e-6,'TolFun',1e-6,  ...
%         'Jacobian','off','MaxIter',400,'MaxFunEvals',700,'Display','iter');

options = optimset('Algorithm','levenberg-marquardt','TolX',0.00000001,'TolFun',0.00000001,  ... 
         'Jacobian','off','MaxIter',100000000,'MaxFunEvals',1000000000,'Display','off'); 
NLS=[];RESNORM=[];RESIDUAL_i=[];NLS_IR=[];RESNORM_IR=[];RESIDUAL_iIR=[];
[NLS,RESNORM,RESIDUAL_i] = lsqnonlin(@euclidian_dist,Z,[],[],options);
options = optimset('Algorithm','levenberg-marquardt','TolX',0.00000001,'TolFun',0.00000001,  ... 
         'Jacobian','off','MaxIter',10000000,'MaxFunEvals',100000000,'Display','off'); 
[NLS_IR,RESNORM_IR,RESIDUAL_iIR] = lsqnonlin(@euclidian_distIR,ZIR,[],[],options);
for i=1:9
if(NLS(i)>90)
    NLS(i)=360-NLS(i);
elseif(NLS(i)<-90)
    NLS(i)=360+NLS(i);
end
end
Eu1=reshape(NLS(1:9),[3,3]);
Rii=SpinCalc('EA321toDCM',Eu1,0.1,1);
tii = NLS(10:18);
tii=reshape(tii,[3,3]);
Ct=NLS(19:end);
num_c=length(Ct)/(3*4);
Ct=reshape(Ct,[3,4,num_c]);

RiiIR=zeros(3,3,3);
tiiIR=zeros(3,3);
if ~isempty(NLS_IR)
    for i=1:9

        if(NLS_IR(i)>90)
            NLS_IR(i)=360-NLS_IR(i);
        elseif(NLS_IR(i)<-90)
            NLS_IR(i)=360+NLS_IR(i);
        end
    end
    EuIR1=reshape(NLS_IR(1:9),[3,3]);
    RiiIR=SpinCalc('EA321toDCM',EuIR1,0.1,1); 
    tiiIR = NLS_IR(10:18);
    tiiIR=reshape(tiiIR,[3,3]);
    CtIR=NLS_IR(19:end);
    num_cIR=length(CtIR)/(3*4);
    CtIR=reshape(CtIR,[3,4,num_cIR]);
else
    for c=1:3
    RiiIR(1:3,1:3,c)=RiIR_{c};
    tiiIR(1:3,c)=tiIR_{c};
    CtIR=C_tildaIR;
    end
end

%%
cm_t=zeros(4,2,3,ob_l);
dcm_t=zeros(4,2,3,ob_l);
temp=zeros(1,3);
rep_IR=zeros(4,3,ob_l);
rep_dep=zeros(4,3,ob_l);
for w=1:ob_l
    for i=1:4 
        for c=1:3
            temp=Rii(:,:,c)'*(Ct(:,i,w)-tii(:,c));
            dcm_t(i,1,c,w)=((temp(1).*fx(c))./temp(3))+cx(c);
            dcm_t(i,2,c,w)=((temp(2).*fy(c))./temp(3))+cy(c);
%             temp=zeros(1,3);
            temp=RiiIR(:,:,c)'*(CtIR(:,i,w)-tiiIR(:,c));
            cm_t(i,1,c,w)=((temp(1).*fx(c))./temp(3))+cx(c);
            cm_t(i,2,c,w)=((temp(2).*fy(c))./temp(3))+cy(c);
            rep_IR(i,c,w)=pdist([cm_t(i,:,c,w);cm(i,:,c,w)],'euclidean');
            rep_dep(i,c,w)=pdist([dcm_t(i,:,c,w);dcm(i,:,c,w)],'euclidean');
        end
    end
end
% 
% Rii=SpinCalc('QtoDCM',[NLS(1),NLS(2),NLS(3),NLS(4)],0.01,1);%reshape(NLS(1:9)',[3,3]);%SpinCalc('EA321toDCM',[NLS(1),NLS(2),NLS(3)],0.000001,1);
% % Rii=Rii';
% tii=[NLS(5);NLS(6);NLS(7)];%%[NLS(10);NLS(11);NLS(12)];
% Ct=NLS(8:end);
% num_c=length(Ct)/3;
% Ct=reshape(Ct,[3,num_c]);
% tii=ti;Rii=Ri;
%%%
% [d,Z,Transform] = procrustes(SpherePos(:,:,1),SpherePos(:,:,2),'scaling',false);
% Ri=Transform.T;ti=Transform.c(1,:)';
 %%%
% %for cam1
% DR1=eye(3);Dt1=[0 0 0]';
% %for cam2
% DA=SpherePos(:,:,2)';DB=SpherePos(:,:,1)';
% [regParams,Bfit,ErrorStats]=absor(DA,DB,'doTrans',true);
% Ri=regParams.R;ti=regParams.t;
%%%
figure();
Clor={[0.5,0,0],[0,0.5,0],[0,0,0.5]};
for w=1:ob_l
    for c=1:3
        figure();
        for i=1:4
            pcshow(Spheres1{i,c,w},Clor{c},'MarkerSize',5); hold on;
            plot3(SpherePos(i,1,c,w),SpherePos(i,2,c,w),SpherePos(i,3,c,w),'r*');hold on;
            plot3(SpherePosIR(i,1,c,w),SpherePosIR(i,2,c,w),SpherePosIR(i,3,c,w),'g*');hold on;
        end
        
        plot3(SpherePos(1:2,1,c,w),SpherePos(1:2,2,c,w),SpherePos(1:2,3,c,w),'--g');
        hold on;
        plot3([SpherePos(1,1,c,w);SpherePos(3,1,c,w)],[SpherePos(1,2,c,w);SpherePos(3,2,c,w)],[SpherePos(1,3,c,w);SpherePos(3,3,c,w)],'--r');
        hold on;
        plot3([SpherePos(1,1,c,w);SpherePos(4,1,c,w)],[SpherePos(1,2,c,w);SpherePos(4,2,c,w)],[SpherePos(1,3,c,w);SpherePos(4,3,c,w)],'--b');
        hold off;
        xlabel('X');ylabel('Y');zlabel('Z'); grid off;
        title(['Point Cloud of calibration object from the Camera ',num2str(c),'set ',num2str(w)]);
    end;
end
%%% fusion
%%Depth
clor1={'r','g','b'};
for w=1:ob_l
    figure();
    for i=1:4 
        for c=1:3
                W1=(bsxfun(@plus,(Rii(:,:,c)*Spheres1{i,c,w}')',tii(:,c)'));
                WW1=(Rii(:,:,c)*SpherePos(i,1:3,c,w)')+tii(:,c);
%                 W1=(bsxfun(@plus,(Ri_{c}*Spheres1{i,c,w}')',ti_{c}'));
%                 WW1=(Ri_{c}*SpherePos(i,1:3,c,w)')+ti_{c};
%     W2=(bsxfun(@plus,(R_P2*Spheres1{i,2}')',t_P2'));
%     WW1=(R_P1*SpherePos(i,1:3,1)')+t_P1;
%     WW2=(R_P2*SpherePos(i,1:3,2)')+t_P2;
                pcshow([W1(:,1),W1(:,2),W1(:,3)],Clor{c},'MarkerSize',5); hold on;
                plot3(WW1(1),WW1(2),WW1(3),'color',clor1{c},'marker','*');hold on;
        end
        plot3(Ct(1,i,w),Ct(2,i,w),Ct(3,i,w),'color','k','marker','*');hold on;
    end;
            Cp=Ct(:,:,w);Cp=Cp';
        hold on; 
        plot3(Cp(1:2,1),Cp(1:2,2),Cp(1:2,3),'--g');
        hold on;
        plot3([Cp(1,1);Cp(3,1)],[Cp(1,2);Cp(3,2)],[Cp(1,3);Cp(3,3)],'--r');
        hold on;
        plot3([Cp(1,1);Cp(4,1)],[Cp(1,2);Cp(4,2)],[Cp(1,3);Cp(4,3)],'--b');
        hold off;
        xlabel('X');ylabel('Y');zlabel('Z'); grid off;
        title(['Fusion Point Cloud of calibration object in common frame for set ',num2str(w)]);
end
%IR
for w=1:ob_l
    figure();
    for i=1:4 
        for c=1:3
                W1IR=(bsxfun(@plus,(RiiIR(:,:,c)*Spheres1{i,c,w}')',tiiIR(:,c)'));
                WW1IR=(RiiIR(:,:,c)*SpherePosIR(i,1:3,c,w)')+tiiIR(:,c);
%                 W1=(bsxfun(@plus,(Ri_{c}*Spheres1{i,c,w}')',ti_{c}'));
%                 WW1=(Ri_{c}*SpherePos(i,1:3,c,w)')+ti_{c};
%     W2=(bsxfun(@plus,(R_P2*Spheres1{i,2}')',t_P2'));
%     WW1=(R_P1*SpherePos(i,1:3,1)')+t_P1;
%     WW2=(R_P2*SpherePos(i,1:3,2)')+t_P2;
                pcshow([W1IR(:,1),W1IR(:,2),W1IR(:,3)],Clor{c},'MarkerSize',5); hold on;
                plot3(WW1IR(1),WW1IR(2),WW1IR(3),'color',clor1{c},'marker','*');hold on;
        end
        plot3(CtIR(1,i,w),CtIR(2,i,w),CtIR(3,i,w),'color','k','marker','*');hold on;
    end;
            Cp=CtIR(:,:,w);Cp=Cp';
        hold on; 
        plot3(Cp(1:2,1),Cp(1:2,2),Cp(1:2,3),'--g');
        hold on;
        plot3([Cp(1,1);Cp(3,1)],[Cp(1,2);Cp(3,2)],[Cp(1,3);Cp(3,3)],'--r');
        hold on;
        plot3([Cp(1,1);Cp(4,1)],[Cp(1,2);Cp(4,2)],[Cp(1,3);Cp(4,3)],'--b');
        hold off;
        xlabel('X');ylabel('Y');zlabel('Z'); grid off;
        title(['Fusion Point Cloud of calibration object (IR images) in common frame for set ',num2str(w)]);
end
% C2<-P<-C1
R_21_dep=Rii(:,:,2)'*Rii(:,:,1);t_21_dep=Rii(:,:,2)'*tii(:,1)-Rii(:,2)'*tii(:,2);
% R_12_dep=R_21_dep';t_12_dep=-R_21_dep'*t_21_dep;
%Rii_P1'*Rii_Pc+Rii_P1'*tii_Pc-Rii_P1'*tii_P1
P_R_1=Rii(:,:,1);Two_R_P=Rii(:,:,2)';P_t_1=tii(:,1);Two_t_P=-Rii(:,:,2)'*tii(:,2);
R_21_dep=Two_R_P*P_R_1;t_21_dep=Two_R_P*P_t_1+Two_t_P;
P_R_1=Rii(:,:,1);Three_R_P=Rii(:,:,3)';P_t_1=tii(:,1);Three_t_P=-Rii(:,:,3)'*tii(:,3);
R_31_dep=Three_R_P*P_R_1;t_31_dep=Three_R_P*P_t_1+Three_t_P;
P_R_3=Rii(:,:,3);Two_R_P=Rii(:,:,2)';P_t_3=tii(:,3);Two_t_P=-Rii(:,:,2)'*tii(:,2);
R_23_dep=Two_R_P*P_R_3;t_23_dep=Two_R_P*P_t_3+Two_t_P;
R_13_dep=R_31_dep';t_13_dep=-R_13_dep*t_31_dep;
R_12_dep=R_21_dep';t_12_dep=-R_12_dep*t_21_dep;
R_32_dep=R_23_dep';t_32_dep=-R_32_dep*t_23_dep;

% R_12_dep=Rii(:,:,1)'*Rii(:,:,2);t_12_dep=Rii(:,:,1)'*tii(:,2)-Rii(:,1)'*tii(:,1);
% R_23_dep=Rii(:,:,2)'*Rii(:,:,3);t_23_dep=Rii(:,:,2)'*tii(:,3)-Rii(:,2)'*tii(:,2);
% R_13_dep=Rii(:,:,1)'*Rii(:,:,3);t_13_dep=Rii(:,:,1)'*tii(:,3)-Rii(:,1)'*tii(:,1);
% R_32_dep=Rii(:,:,3)'*Rii(:,:,2);t_32_dep=Rii(:,:,3)'*tii(:,2)-Rii(:,3)'*tii(:,3);
% R_31_dep=Rii(:,:,3)'*Rii(:,:,1);t_31_dep=Rii(:,:,3)'*tii(:,1)-Rii(:,3)'*tii(:,3);
% C2<-P<-C1
R_21_ir=RiiIR(:,:,2)'*RiiIR(:,:,1);t_21_ir=RiiIR(:,:,2)'*tiiIR(:,1)-RiiIR(:,2)'*tiiIR(:,2);
% R_12_ir=R_21_ir';t_12_ir=-R_21_ir'*t_21_ir;
%Rii_P1'*Rii_Pc+Rii_P1'*tii_Pc-Rii_P1'*tii_P1
P_R_1IR=RiiIR(:,:,1);Two_R_PIR=RiiIR(:,:,2)';P_t_1IR=tiiIR(:,1);Two_t_PIR=-RiiIR(:,:,2)'*tiiIR(:,2);
R_21_ir=Two_R_PIR*P_R_1IR;t_21_ir=Two_R_PIR*P_t_1IR+Two_t_PIR;
P_R_1IR=RiiIR(:,:,1);Three_R_PIR=RiiIR(:,:,3)';P_t_1IR=tiiIR(:,1);Three_t_PIR=-RiiIR(:,:,3)'*tiiIR(:,3);
R_31_ir=Three_R_PIR*P_R_1IR;t_31_ir=Three_R_PIR*P_t_1IR+Three_t_PIR;
P_R_3IR=RiiIR(:,:,3);Two_R_PIR=RiiIR(:,:,2)';P_t_3IR=tiiIR(:,3);Two_t_PIR=-RiiIR(:,:,2)'*tiiIR(:,2);
R_23_ir=Two_R_PIR*P_R_3IR;t_23_ir=Two_R_PIR*P_t_3IR+Two_t_PIR;
R_13_ir=R_31_ir';t_13_ir=-R_13_ir*t_31_ir;
R_12_ir=R_21_ir';t_12_ir=-R_12_ir*t_21_ir;
R_32_ir=R_23_ir';t_32_ir=-R_32_ir*t_23_ir;


%%Depth in frame of Cam 1
clor1={'r','g','b'};
for w=1:ob_l
    figure();
    for i=1:4 
                W1=Spheres1{i,1,w};
                W2=(bsxfun(@plus,(R_12_dep*Spheres1{i,2,w}')',t_12_dep'));
                W3=(bsxfun(@plus,(R_13_dep*Spheres1{i,3,w}')',t_13_dep'));

                pcshow(W1,Clor{1},'MarkerSize',5); hold on;
                pcshow(W2,Clor{2},'MarkerSize',5); hold on;
                pcshow(W3,Clor{3},'MarkerSize',5); hold on;
    end;
        title(['Fusion Point Cloud of calibration object in common frame for set ',num2str(w)]);
end

%%IR in frame of Cam 1
clor1={'r','g','b'};
for w=1:ob_l
    figure();
    for i=1:4 
                W1=Spheres1{i,1,w};
                W2=(bsxfun(@plus,(R_12_ir*Spheres1{i,2,w}')',t_12_ir'));
                W3=(bsxfun(@plus,(R_13_ir*Spheres1{i,3,w}')',t_13_ir'));

                pcshow(W1,Clor{1},'MarkerSize',5); hold on;
                pcshow(W2,Clor{2},'MarkerSize',5); hold on;
                pcshow(W3,Clor{3},'MarkerSize',5); hold on;
    end;
        title(['Fusion Point Cloud of calibration object in common frame for set ',num2str(w)]);
end
%% Depth appraoch on camera poses
point1 = [1,0,0];
point2 = [0,1,0];
point3 = [0 0 1];
origin = [0,0,0];

pt1_2= R_12_dep*point1'+t_12_dep;%R_12_dep(:,1);
pt2_2= R_12_dep*point2'+t_12_dep;%R_12_dep(:,2);
pt3_2= R_12_dep*point3'+t_12_dep;%R_12_dep(:,3);
origin2=t_12_dep;

pt1_3= R_13_dep*point1'+t_13_dep;%R_13_dep(:,1);
pt2_3= R_13_dep*point2'+t_13_dep;%R_13_dep(:,2);
pt3_3= R_13_dep*point3'+t_13_dep;%R_13_dep(:,3);
origin3=t_13_dep;

figure;hold on;
arrow_size=0.4;
%Cam1
dir_vec1=point1-origin;dir_vec2=point2-origin;dir_vec3=point3-origin;
% plot3([origin(1) point1(1)],[origin(2) point1(2)],[origin(3) point1(3)],'g','LineWidth',1);hold on;
% plot3([origin(1) point2(1)],[origin(2) point2(2)],[origin(3) point2(3)],'r','LineWidth',1);hold on;
% plot3([origin(1) point3(1)],[origin(2) point3(2)],[origin(3) point3(3)],'b','LineWidth',1);hold on;
quiver3(origin(1),origin(2),origin(3),dir_vec1(1),dir_vec1(2),dir_vec1(3),'Color','r','MaxHeadSize',arrow_size);hold on;
quiver3(origin(1),origin(2),origin(3),dir_vec2(1),dir_vec2(2),dir_vec2(3),'Color','r','MaxHeadSize',arrow_size);hold on;
quiver3(origin(1),origin(2),origin(3),dir_vec3(1),dir_vec3(2),dir_vec3(3),'Color','r','MaxHeadSize',arrow_size);hold on;
text(origin(1),origin(2),origin(3),'Sensor 1','HorizontalAlignment','left','FontSize',8);

%Cam2
dir_vec1=R_12_dep(:,1);dir_vec2=R_12_dep(:,2);dir_vec3=R_12_dep(:,3);
% dir_vec1=pt1_2-origin2;dir_vec2=pt2_2-origin2;dir_vec3=pt3_2(3)-origin2;
% plot3([origin2(1) pt1_2(1)],[origin2(2) pt1_2(2)],[origin2(3) pt1_2(3)],'g','LineWidth',1);hold on;
% plot3([origin2(1) pt2_2(1)],[origin2(2) pt2_2(2)],[origin2(3) pt2_2(3)],'r','LineWidth',1);hold on;
% plot3([origin2(1) pt3_2(1)],[origin2(2) pt3_2(2)],[origin2(3) pt3_2(3)],'b','LineWidth',1);hold on;
quiver3(origin2(1),origin2(2),origin2(3),dir_vec1(1),dir_vec1(2),dir_vec1(3),'Color','r','MaxHeadSize',arrow_size);hold on;
quiver3(origin2(1),origin2(2),origin2(3),dir_vec2(1),dir_vec2(2),dir_vec2(3),'Color','r','MaxHeadSize',arrow_size);hold on;
quiver3(origin2(1),origin2(2),origin2(3),dir_vec3(1),dir_vec3(2),dir_vec3(3),'Color','r','MaxHeadSize',arrow_size);hold on;

text(origin2(1),origin2(2),origin2(3),'Sensor 2','HorizontalAlignment','left','FontSize',8);
%Cam3
dir_vec1=R_13_dep(:,1);dir_vec2=R_13_dep(:,2);dir_vec3=R_13_dep(:,3);
% dir_vec1=pt1_3-origin3;dir_vec2=pt2_3-origin3;dir_vec3=pt3_3(3)-origin3;
% plot3([origin3(1) pt1_3(1)],[origin3(2) pt1_3(2)],[origin3(3) pt1_3(3)],'g','LineWidth',1);hold on;
% plot3([origin3(1) pt2_3(1)],[origin3(2) pt2_3(2)],[origin3(3) pt2_3(3)],'r','LineWidth',1);hold on;
% plot3([origin3(1) pt3_3(1)],[origin3(2) pt3_3(2)],[origin3(3) pt3_3(3)],'b','LineWidth',1);hold on;
quiver3(origin3(1),origin3(2),origin3(3),dir_vec1(1),dir_vec1(2),dir_vec1(3),'Color','r','MaxHeadSize',arrow_size);hold on;
quiver3(origin3(1),origin3(2),origin3(3),dir_vec2(1),dir_vec2(2),dir_vec2(3),'Color','r','MaxHeadSize',arrow_size);hold on;
quiver3(origin3(1),origin3(2),origin3(3),dir_vec3(1),dir_vec3(2),dir_vec3(3),'Color','r','MaxHeadSize',arrow_size);hold on;

text(origin3(1),origin3(2),origin3(3),'Sensor 3','HorizontalAlignment','left','FontSize',8);
% figure();
% cam1 = plotCamera('Location',-[0,0,0]*eye(3,3),'Orientation',eye(3,3),'Opacity',0,'Size',0.1,'Visible',true,'AxesVisible',true,'Label','Camera 1');
% hold on;
% cam2 = plotCamera('Location',-t_12_dep'*R_12_dep','Orientation',R_12_dep','Opacity',0,'Size',0.1,'Visible',true,'AxesVisible',true,'Label','Camera 2');
% hold on;
% cam3 = plotCamera('Location',-t_13_dep'*R_13_dep','Orientation',R_13_dep','Opacity',0,'Size',0.1,'Visible',true,'AxesVisible',true,'Label','Camera 3');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-3 3]);
ylim([-2 2]);
zlim([-5 5]);
% set(gca,'CameraPosition',[2 2 2]);
%% IR approach on camera poses
point1 = [1,0,0];
point2 = [0,1,0];
point3 = [0 0 1];
origin = [0,0,0];

pt1_2= R_12_ir*point1'+t_12_ir;%R_12_dep(:,1);
pt2_2= R_12_ir*point2'+t_12_ir;%R_12_dep(:,2);
pt3_2= R_12_ir*point3'+t_12_ir;%R_12_dep(:,3);
origin2=t_12_ir;

pt1_3= R_13_ir*point1'+t_13_ir;%R_13_dep(:,1);
pt2_3= R_13_ir*point2'+t_13_ir;%R_13_dep(:,2);
pt3_3= R_13_ir*point3'+t_13_ir;%R_13_dep(:,3);
origin3=t_13_ir;
% figure;
hold on;
arrow_size=0.4;
%Cam1
dir_vec1=point1-origin;dir_vec2=point2-origin;dir_vec3=point3-origin;
quiver3(origin(1),origin(2),origin(3),dir_vec1(1),dir_vec1(2),dir_vec1(3),'Color','b','MaxHeadSize',arrow_size);hold on;
quiver3(origin(1),origin(2),origin(3),dir_vec2(1),dir_vec2(2),dir_vec2(3),'Color','b','MaxHeadSize',arrow_size);hold on;
quiver3(origin(1),origin(2),origin(3),dir_vec3(1),dir_vec3(2),dir_vec3(3),'Color','b','MaxHeadSize',arrow_size);hold on;
text(origin(1),origin(2),origin(3),'Sensor 1','HorizontalAlignment','left','FontSize',8);

%Cam2
dir_vec1=R_12_ir(:,1);dir_vec2=R_12_ir(:,2);dir_vec3=R_12_ir(:,3);
quiver3(origin2(1),origin2(2),origin2(3),dir_vec1(1),dir_vec1(2),dir_vec1(3),'Color','b','MaxHeadSize',arrow_size);hold on;
quiver3(origin2(1),origin2(2),origin2(3),dir_vec2(1),dir_vec2(2),dir_vec2(3),'Color','b','MaxHeadSize',arrow_size);hold on;
quiver3(origin2(1),origin2(2),origin2(3),dir_vec3(1),dir_vec3(2),dir_vec3(3),'Color','b','MaxHeadSize',arrow_size);hold on;

text(origin2(1),origin2(2),origin2(3),'Sensor 2','HorizontalAlignment','left','FontSize',8);
%Cam3
dir_vec1=R_13_ir(:,1);dir_vec2=R_13_ir(:,2);dir_vec3=R_13_ir(:,3);
quiver3(origin3(1),origin3(2),origin3(3),dir_vec1(1),dir_vec1(2),dir_vec1(3),'Color','b','MaxHeadSize',arrow_size);hold on;
quiver3(origin3(1),origin3(2),origin3(3),dir_vec2(1),dir_vec2(2),dir_vec2(3),'Color','b','MaxHeadSize',arrow_size);hold on;
quiver3(origin3(1),origin3(2),origin3(3),dir_vec3(1),dir_vec3(2),dir_vec3(3),'Color','b','MaxHeadSize',arrow_size);hold on;

text(origin3(1),origin3(2),origin3(3),'Sensor 3','HorizontalAlignment','left','FontSize',8);
% figure();
% cam1 = plotCamera('Location',-[0,0,0]*eye(3,3),'Orientation',eye(3,3),'Opacity',0,'Size',0.1,'Visible',true,'AxesVisible',true,'Label','Camera 1');
% hold on;
% cam2 = plotCamera('Location',-t_12_dep'*R_12_dep','Orientation',R_12_dep','Opacity',0,'Size',0.1,'Visible',true,'AxesVisible',true,'Label','Camera 2');
% hold on;
% cam3 = plotCamera('Location',-t_13_dep'*R_13_dep','Orientation',R_13_dep','Opacity',0,'Size',0.1,'Visible',true,'AxesVisible',true,'Label','Camera 3');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-3 3]);
ylim([-2 2]);
zlim([-5 5]);
% set(gca,'CameraPosition',[2 2 2]);

% %%%
% figure();
% SpherePos_2=zeros(4,3);
% for i=1:4
%     pcshow(Spheres1{i,1},Spheres1_Rg{i,1});hold on;
%     W=(bsxfun(@plus,(Rii*Spheres1{i,2}')',tii'));
%     W1=(Rii*SpherePos(i,1:3,2)')+tii;
%     SpherePos_2(i,1:3)=W1';
% %     W=(Ri'*(bsxfun(@minus,Spheres{i,2},ti'))')';
%     pcshow([W(:,1),W(:,2),W(:,3)],Spheres1_Rg{i,2});hold on;
%      plot3(SpherePos(i,1,1),SpherePos(i,2,1),SpherePos(i,3,1),'r*');hold on;
%      plot3(W1(1),W1(2),W1(3),'g*');hold on;
%      plot3(Ct(1,i),Ct(2,i),Ct(3,i),'b*');hold on;
%     xlabel('X');ylabel('Y');zlabel('Z');
%     title('Fusion of Points Clouds in frame of Camera 1');
% end;
% plot3(SpherePos(1:2,1,1),SpherePos(1:2,2,1),SpherePos(1:2,3,1),'--g');
% hold on;
% plot3([SpherePos(1,1,1);SpherePos(3,1,1)],[SpherePos(1,2,1);SpherePos(3,2,1)],[SpherePos(1,3,1);SpherePos(3,3,1)],'--r');
% hold on;
% plot3([SpherePos(1,1,1);SpherePos(4,1,1)],[SpherePos(1,2,1);SpherePos(4,2,1)],[SpherePos(1,3,1);SpherePos(4,3,1)],'--b');
% hold on; 
% plot3(SpherePos_2(1:2,1),SpherePos_2(1:2,2),SpherePos_2(1:2,3),'--g');
% hold on;
% plot3([SpherePos_2(1,1);SpherePos_2(3,1)],[SpherePos_2(1,2);SpherePos_2(3,2)],[SpherePos_2(1,3);SpherePos_2(3,3)],'--r');
% hold on;
% plot3([SpherePos_2(1,1);SpherePos_2(4,1)],[SpherePos_2(1,2);SpherePos_2(4,2)],[SpherePos_2(1,3);SpherePos_2(4,3)],'--b');
% 
% %%%
% [row,col,v] = find(D);
% X=((col-cx).*v)./fx;
% Y=((row-cy).*v)./fy;
% Z=v; 
% grayR=[];for f=1:length(row);grayR=[grayR ;Rg(row(f),col(f),1)];end;
% grayG=[];for f=1:length(row);grayG=[grayG ;Rg(row(f),col(f),2)];end;
% grayB=[];for f=1:length(row);grayB=[grayB ;Rg(row(f),col(f),3)];end;
% figure(); pcshow(pointCloud([X(:),Y(:),Z(:)],'Color',[grayR(:),grayG(:),grayB(:)]));
% xlabel('X');ylabel('Y');zlabel('Z');
% title('Point cloud from Camera 1');
% %%%
% [row1,col1,v1] = find(D1);
% X1=((col1-cx1).*v1)./fx1;
% Y1=((row1-cy1).*v1)./fy1;
% Z1=v1; 
% grayR1=[];for f=1:length(row1);grayR1=[grayR1 ;Rg1(row1(f),col1(f),1)];end;
% grayG1=[];for f=1:length(row1);grayG1=[grayG1 ;Rg1(row1(f),col1(f),2)];end;
% grayB1=[];for f=1:length(row1);grayB1=[grayB1 ;Rg1(row1(f),col1(f),3)];end;
% figure(); pcshow(pointCloud([X1(:),Y1(:),Z1(:)],'Color',[grayR1(:),grayG1(:),grayB1(:)]));
% xlabel('X');ylabel('Y');zlabel('Z');
% title('Point cloud from Camera 2');
% %%%
% figure(); pcshow(pointCloud([X(:),Y(:),Z(:)],'Color',[grayR(:),grayG(:),grayB(:)]));
% hold on;
% W1=(bsxfun(@plus,(Rii*[X1(:)';Y1(:)';Z1(:)'])',tii'));
% pcshow(pointCloud([W1(:,1),W1(:,2),W1(:,3)],'Color',[grayR1(:),grayG1(:),grayB1(:)]));
% xlabel('X');ylabel('Y');zlabel('Z');
% title('Fusion Point Cloud');
% 
% %clean fused point cloud
% % cleanptcld=pcdenoise(pointCloud([X1(:),Y1(:),Z1(:);W1(:,1),W1(:,2),W1(:,3)],'Color',[grayR(:),grayG(:),grayB(:);grayR1(:),grayG1(:),grayB1(:)]));
% % figure();
% % pcshow(cleanptcld);
% 
% %%%
% %C1_T_C2
% R_12=Rii;t_12=tii;
% %C2_T_C1
% R_21=Rii';t_21=-Rii'*tii;
% %C1_T_P
% P1=Ct(:,1);
% P2=Ct(:,2);
% P3=Ct(:,3);
% P4=Ct(:,4);
% d = P1;
% u= P2-d;u=u/norm(u);%^i
% v=P3-d;v=v/norm(v);%^j
% w=P4-d;w=w/norm(w);%^k
% R_1P = [u,v,w]; % Rotation matrix
% t_1P=d;
% %P_T_C1
% R_P1=R_1P';t_P1=-R_1P'*t_1P;
% %C2_T_P
% R_2P=R_21*R_1P;t_2P=R_21*t_1P+t_21;
% %P_T_C2
% R_P2=R_2P';t_P2=-R_2P'*t_2P;
% %
% %% cloud fusion in cam2 frame
% Ctt=zeros(3,4);
% figure();
% 
%     for j=1:4
%         Ctt(:,j)=R_12'*(Ct(:,j)-t_12);
% 
%     end
% 
% 
% SpherePos_1=zeros(4,3);
% for i=1:4
%     pcshow(Spheres1{i,2},Spheres1_Rg{i,2});hold on;
%     W=(bsxfun(@plus,(R_21*Spheres1{i,1}')',t_21'));
%     W1=(R_21*SpherePos(i,1:3,1)')+t_21;
%     SpherePos_1(i,1:3)=W1';
% %     W=(Ri'*(bsxfun(@minus,Spheres{i,2},ti'))')';
%     pcshow([W(:,1),W(:,2),W(:,3)],Spheres1_Rg{i,1});hold on;
%      plot3(SpherePos(i,1,2),SpherePos(i,2,2),SpherePos(i,3,2),'g*');hold on;
%      plot3(W1(1),W1(2),W1(3),'r*');hold on;
%      plot3(Ctt(1,i),Ctt(2,i),Ctt(3,i),'b*');hold on;
%     xlabel('X');ylabel('Y');zlabel('Z');
%     title('Fusion of Points Clouds in frame of Camera 2');
% end;
% plot3(SpherePos(1:2,1,2),SpherePos(1:2,2,2),SpherePos(1:2,3,2),'--g');
% hold on;
% plot3([SpherePos(1,1,2);SpherePos(3,1,2)],[SpherePos(1,2,2);SpherePos(3,2,2)],[SpherePos(1,3,2);SpherePos(3,3,2)],'--r');
% hold on;
% plot3([SpherePos(1,1,2);SpherePos(4,1,2)],[SpherePos(1,2,2);SpherePos(4,2,2)],[SpherePos(1,3,2);SpherePos(4,3,2)],'--b');
% hold on; 
% plot3(SpherePos_1(1:2,1),SpherePos_1(1:2,2),SpherePos_1(1:2,3),'--g');
% hold on;
% plot3([SpherePos_1(1,1);SpherePos_1(3,1)],[SpherePos_1(1,2);SpherePos_1(3,2)],[SpherePos_1(1,3);SpherePos_1(3,3)],'--r');
% hold on;
% plot3([SpherePos_1(1,1);SpherePos_1(4,1)],[SpherePos_1(1,2);SpherePos_1(4,2)],[SpherePos_1(1,3);SpherePos_1(4,3)],'--b');
% 
% 
% %% cloud fusion in Prop frame
% SpherePos_1=zeros(4,3);
% SpherePos_2=zeros(4,3);
% W1=[];W2=[] ;WW1=[];WW2=[];
% Cp=zeros(3,4);
% figure();
% 
%     for j=1:4
%         Cp(:,j)=R_1P'*(Ct(:,j)-t_1P);
% 
%     end
% 
% for i=1:4
%     
%     W1=(bsxfun(@plus,(R_P1*Spheres1{i,1}')',t_P1'));
%     W2=(bsxfun(@plus,(R_P2*Spheres1{i,2}')',t_P2'));
%     WW1=(R_P1*SpherePos(i,1:3,1)')+t_P1;
%     WW2=(R_P2*SpherePos(i,1:3,2)')+t_P2;
%     SpherePos_1(i,1:3)=WW1';
%     SpherePos_2(i,1:3)=WW1';
% %     W=(Ri'*(bsxfun(@minus,Spheres{i,2},ti'))')';
%     pcshow([W1(:,1),W1(:,2),W1(:,3)],Spheres1_Rg{i,1});hold on;
%     pcshow([W2(:,1),W2(:,2),W2(:,3)],Spheres1_Rg{i,2});hold on;
%      
%      plot3(WW1(1),WW1(2),WW1(3),'r*');hold on;
%      plot3(WW2(1),WW2(2),WW2(3),'g*');hold on;
%      plot3(Cp(1,i),Cp(2,i),Cp(3,i),'b*');hold on;
%     xlabel('X');ylabel('Y');zlabel('Z');
%     title('Fusion of Points Clouds in frame of Prop');
% end;
% Cp=Cp';
% hold on; 
% plot3(Cp(1:2,1),Cp(1:2,2),Cp(1:2,3),'--g');
% hold on;
% plot3([Cp(1,1);Cp(3,1)],[Cp(1,2);Cp(3,2)],[Cp(1,3);Cp(3,3)],'--r');
% hold on;
% plot3([Cp(1,1);Cp(4,1)],[Cp(1,2);Cp(4,2)],[Cp(1,3);Cp(4,3)],'--b');

% Journal evaluation
Clor={[0.5,0,0],[0,0.5,0],[0,0,0.5]};cl=[1,0,0;0,1,0;0,0,1];
for w=2
    for c=1:3
        figure();
        for i=1:4
            pcshow(Spheres1{i,c,w},Clor{c},'MarkerSize',10); hold on;
         p= plot3(SpherePos(i,1,c,w),SpherePos(i,2,c,w),SpherePos(i,3,c,w),'*','Color',cl(c,:));hold on;
%             plot3(SpherePosIR(i,1,c,w),SpherePosIR(i,2,c,w),SpherePosIR(i,3,c,w),'g*');hold on;
        end
        
        px=plot3(SpherePos(1:2,1,c,w),SpherePos(1:2,2,c,w),SpherePos(1:2,3,c,w),'--g');
        hold on;
        py=plot3([SpherePos(1,1,c,w);SpherePos(3,1,c,w)],[SpherePos(1,2,c,w);SpherePos(3,2,c,w)],[SpherePos(1,3,c,w);SpherePos(3,3,c,w)],'--r');
        hold on;
        pz=plot3([SpherePos(1,1,c,w);SpherePos(4,1,c,w)],[SpherePos(1,2,c,w);SpherePos(4,2,c,w)],[SpherePos(1,3,c,w);SpherePos(4,3,c,w)],'--b');
        hold off;
        l=legend([p,px,py,pz],['Estimated spherical centre from ToF sensor ',num2str(c)],'X axis','Y axis','Z axis');
        title(l,'Target Frame');
        xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)'); grid on;
        title(['Point cloud of calibration target from sensor ',num2str(c)]);%,' set ',num2str(w)]);
    end;
end

clor1={'r','g','b'};pp1=zeros(1,3);p2=[];
for w=2
    figure();
    for i=1:4 
        for c=1:3
                W1=(bsxfun(@plus,(Rii(:,:,c)*Spheres1{i,c,w}')',tii(:,c)'));
                WW1=(Rii(:,:,c)*SpherePos(i,1:3,c,w)')+tii(:,c);
%                 W1=(bsxfun(@plus,(Ri_{c}*Spheres1{i,c,w}')',ti_{c}'));
%                 WW1=(Ri_{c}*SpherePos(i,1:3,c,w)')+ti_{c};
%     W2=(bsxfun(@plus,(R_P2*Spheres1{i,2}')',t_P2'));
%     WW1=(R_P1*SpherePos(i,1:3,1)')+t_P1;
%     WW2=(R_P2*SpherePos(i,1:3,2)')+t_P2;
               pp1= pcshow([W1(:,1),W1(:,2),W1(:,3)],Clor{c},'MarkerSize',10); hold on;
%                 plot3(WW1(1),WW1(2),WW1(3),'color',clor1{c},'marker','*');hold on;
        end
        pp1(c)= pcshow([W1(:,1),W1(:,2),W1(:,3)],Clor{c},'MarkerSize',10); hold on;
        p2 = plot3(Ct(1,i,w),Ct(2,i,w),Ct(3,i,w),'color','b','marker','*');hold on;
    end;
            Cp=Ct(:,:,w);Cp=Cp';
            pp1
        hold on; 
        px = plot3(Cp(1:2,1),Cp(1:2,2),Cp(1:2,3),'--g');
        hold on;
        py = plot3([Cp(1,1);Cp(3,1)],[Cp(1,2);Cp(3,2)],[Cp(1,3);Cp(3,3)],'--r');
        hold on;
        pz = plot3([Cp(1,1);Cp(4,1)],[Cp(1,2);Cp(4,2)],[Cp(1,3);Cp(4,3)],'--b');
        hold on;
        l=legend([p2,px,py,pz],...%'Transformed Point cloud from sensor 1');%,'Transformed Point cloud from sensor 2');%});,'Transformed Point cloud from sensor 2','Transformed Point cloud from sensor 3'});%,...
            'Final spherical centres viewed from all sensors w.r.t Target frame',...
            'X axis','Y axis','Z axis');
        title(l,'Target Frame')
        xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)'); grid on;
        title(['Fusion Point Cloud of calibration object in common frame for set ',num2str(w)]);
end
