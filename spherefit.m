function [estimateCenter1,inliers_set,outliers_set,estimateCenter,estimateRadius]=spherefit(X,Y,Z,radius)
% close all;
global PointsForNLS
global true_radius
pt=[X(:),Y(:),Z(:)];
ptcld=pointCloud([X(:),Y(:),Z(:)]);
% figure;pcshow(ptcld);
% title('Point Cloud');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
  [model,inlierIndices,outlierIndices,rmse] = pcfitsphere(ptcld,0.01,'MaxNumTrials',100000);
  estimateCenter=model.Center;estimateRadius=model.Radius;
   
%  XYZ=[X(inlierIndices),Y(inlierIndices),Z(inlierIndices)];
% Points=[X';Y';Z'];
% [center11, estimateRadius, residual1, inliers11, outliers11, indicesReal11] = f_sphereFit_points2Sphere(Points,0.5);
% % 
%  estimateCenter=center11';

%
%  f = @(C) sum((sum(bsxfun(@minus,XYZ,C).^2,2)-(radius)^2).^2);
%  opts = optimset('Display', 'none', 'TolFun', 1e-8, 'TolX', 1e-8);
%  estimateCenter = fminsearch(f, double([model.Center]));
% % function err=fun(pt)
% % end
% err=(sum(bsxfun(@minus,pt,estimateCenter).^2,2)-(radius*10)^2).^2;
% globe = select(ptcld,inlierIndices);
% model1 =sphereModel([estimateCenter radius]);
% figure;pcshow(ptcld);zlim([1800 2500]);hold on;plot(model);
% figure;pcshow(ptcld);zlim([1800 2500]);
% hold on ; plot(model1);
% figure;
% pcshow(globe);zlim([1800 3000]);
% title('Globe Point Cloud');estimateRadius
% end

% Hough Transform
%Accu=zeros(,1000,3000);
% cx=[];cy=[];cz=[];
% for i=1:length(X(:))
%     for theta=0:2*pi
%         for phi=0:pi
%             cxd= ceil(X(i)-radius*cos(theta)*sin(phi));
%             cyd=ceil(Y(i)-radius*sin(theta)*sin(phi));
%             czd=ceil(Z(i)-radius*cos(phi));
%              if czd>min(Z(:)) && czd<3500 %&& cxd>0 && cyd>0
%               cx=[cx cxd];
%               cy=[cy cyd];
%               cz=[cz czd];
%             end
%         end
%     end
% end
% cx1=cx+abs(min(cx))+1;cy1=cy+abs(min(cy))+1; cz1=cz;
% Accu=ndSparse.build([double(cx1') double(cy1') double(cz1')],[0]);
% for i=1:length(X(:))
%     for theta=0:2*pi
%         for phi=0:pi
%             cxd= ceil(X(i)-radius*cos(theta)*sin(phi))+abs(min(cx))+1;
%             cyd=ceil(Y(i)-radius*sin(theta)*sin(phi))+abs(min(cy))+1;
%             czd=ceil(Z(i)-radius*cos(phi));
%             if czd>min(Z(:)) && czd<3500 
%             Accu(cxd,cyd,czd)=Accu(cxd,cyd,czd)+1;
%             end
%         end
%     end
%  end
% maxind=[0 0 0];vot=0;er=6e+20;
% % for i=1:2186
% %     [ro,co,vo]=find(Accu(:,:,i));
% %     if ~isempty(vo)
% %         %[l,ind]=max(vo);
% %         
% %         if l>=vot
% %             if l==vot
% %             err=sum((sum(bsxfun(@minus,pt,[ro(ind),co(ind),i]).^2,2)-(radius)^2).^2);
% %                 if err<er
% %                   er=err;  
% %                   vot=l;
% %                   maxind=[ro(ind),co(ind),i];    
% %                 end
% %             else
% %               vot=l;
% %               maxind=[ro(ind),co(ind),i];  
% %             end
% %         end;
% %     end
% % end;
% [p1,q1,r1] = ind2sub(size(Accu),find(Accu > 1));
% p1=p1-1-abs(min(cx));q1=q1-1-abs(min(cy));
% for i=1:length(p1)
%     err=sum((sum(bsxfun(@minus,pt,[p1(i),q1(i),r1(i)]).^2,2)-(radius)^2).^2);
%     if err<er
%         er=err;
%         maxind=[p1(i),q1(i),r1(i)]; 
%     end
% end
% %inliers
% if maxind==[0 0 0]
%     maxind=estimateCenter;
% end;
% err=abs(radius-sqrt(sum(bsxfun(@minus,pt,maxind).^2,2)));
% inliers=0 ;inliers_set=[];
% for j=1:length(err)
%      if err(j)<30
%          inliers=inliers+1;
%          inliers_set=[inliers_set j];
%      end
% end
% XYZ=[X(inliers_set)';Y(inliers_set)';Z(inliers_set)'];
XYZ=[X(inlierIndices)';Y(inlierIndices)';Z(inlierIndices)'];
% XYZ=inliers11;
 %XYZ=[X';Y';Z'];
% Points on the sphere
PointsForNLS = XYZ;
true_radius = radius;
% The variables to be minizied
% Z = double([center11(1); center11(2); center11(3)]);
Z = double([estimateCenter(1); estimateCenter(2); estimateCenter(3)]);

options = optimset('Algorithm','levenberg-marquardt','TolX',0.000001,'TolFun',0.000001,  ... 
                      'Jacobian','off','MaxIter',6000,'MaxFunEvals',6000,'Display','off');

 [F,~]=f_minSphereFixedRadius(Z);
[NLS,RESNORM,RESIDUAL_i] = lsqnonlin(@f_minSphereFixedRadius,Z,[],[],options);


estimateCenter1 = [NLS(1),NLS(2),NLS(3)];


% C=[0 0 0];
% f = @(C) sum((sum(bsxfun(@minus,XYZ,C).^2,2)-(radius)^2).^2);
% opts = optimset('Display', 'none', 'TolFun', 1e-8, 'TolX', 1e-8);
% estimateCenter1 = fminsearch(f, double(maxind));
%inliers
err=abs(radius-sqrt(sum(bsxfun(@minus,pt,estimateCenter1).^2,2)));
inliers=0 ;inliers_set=[];outliers_set=[];
for j=1:length(err)
     if err(j)<10
         inliers=inliers+1;
         inliers_set=[inliers_set j];
     else
         outliers_set=[outliers_set j];
     end
end

% model2=sphereModel([estimateCenter1 radius]);
% figure;pcshow(ptcld);zlim([1800 2500]);
% hold on ; plot(model2);
% title('Fitted Sphere after Hough Tranform and Non-linear Optimization');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% globe = select(ptcld,inliers_set);
% figure;
% pcshow(globe);zlim([1800 25000]);
% title('Inlier Point Cloud');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
end