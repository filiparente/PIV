d=dir('D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_*'); %getting all the file in this directory
% O stor disse que ia mudar os nomes da imagem para dar por ordem!

%getting the background image

ims1=[];
ims2=[];
imsd1=[];
imsd2=[];
for i=1:length(d),
    im1=rgb2gray(imread((['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_' d(i).name(12:end-3) 'png'])));
    ims1=[ims1 im1(:)];
    im2=rgb2gray(imread((['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\rgb_image2_' d(i).name(12:end-3) 'png'])));
    ims2=[ims2 im2(:)];
    load(['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])
    imsd1=[imsd1 depth_array(:)];
    load(['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\depth2_' d(i).name(12:end-3) 'mat'])
    imsd2=[imsd2 depth_array(:)];
end

medim1=median(double(ims1),2);
medim2=median(double(ims2),2);
% medim1=quantile(double(ims1),0.75,2);
% medim2=quantile(double(ims2),0.75,2);
imgray1=(uint8(reshape(medim1,[480 640])));
imgray2=(uint8(reshape(medim2,[480 640])));

meddep1=median(double(imsd1),2);
bgimd1=reshape(meddep1,[480 640]);
meddep2=median(double(imsd2),2);
bgimd2=reshape(meddep2,[480 640]);

%%
%MEXER NOS PARAMETROS PARA VER O QUE DA MELHOR
peak_thresh=0; % increase to limit ; default is 0
edge_thresh=10; %decrease to limit ; default is 10

[F1,d1]=vl_sift(single(imgray1),'PeakThresh', peak_thresh,'edgethresh', edge_thresh);
[F2,d2]=vl_sift(single(imgray2),'PeakThresh', peak_thresh,'edgethresh', edge_thresh);

%%
imshow(imgray1);
perm = randperm(size(F2,2)) ;
sel = perm(1:50) ;
h1 = vl_plotframe(F2(:,sel)) ;
h2 = vl_plotframe(F2(:,sel)) ;
set(h1,'color','k','linewidth',3) ;
set(h2,'color','y','linewidth',2) ;

%%
%finding matches 

match_thresh=1.5;%increase to limit matches ; default is 1.5
[matches, scores]= vl_ubcmatch(d1,d2,match_thresh);

%%
figure(2) ; clf ;
imagesc(cat(2, imgray1,imgray2)) ;

u1 = F1(1,matches(1,:)) ;
u2 = F2(1,matches(2,:)) + size(imgray1,2) ;
v1 = F1(2,matches(1,:)) ;
v2 = F2(2,matches(2,:)) ;

hold on ;
h = line([u1 ; u2], [v1 ; v2]) ;
set(h,'linewidth', 1, 'color', 'b') ;

vl_plotframe(F1(:,matches(1,:))) ;
F2(1,:) = F2(1,:) + size(imgray1,2) ;
vl_plotframe(F2(:,matches(2,:))) ;
axis image off ;

% %% ESTA MAL
% 
% %encontrar coordenadas 3d a partir de f1 e f2.
% u1 = round(F1(1,matches(1,:))) ;
% u2 = round(F2(1,matches(2,:))) ;
% v1 = round(F1(2,matches(1,:))) ;
% v2 = round(F2(2,matches(2,:))) ;
% K=Depth_cam.K;
% 
% %3d coordinates of keypoints in cam1
% xyz_keypts1=zeros(length(matches(1,:)),3);
% %passar para 3D
% for ind=1:length(matches(1,:))
%     %xyz_keypts1(ind,3) =imgs1(v1(ind),u1(ind),i)*0.001; % Convert to meters
%     xyz_keypts1(ind,3) =bgimd1(v1(ind),u1(ind))*0.001; % Convert to meters
% end
% 
% xyz_keypts1(:,1)= (xyz_keypts1(:,3)/K(1,1))' .*(u1-K(1,3)) ;
% xyz_keypts1(:,2) = (xyz_keypts1(:,3)/K(2,2))' .*(v1-K(2,3)) ;
% 
% %3d coordinates of keypoints in cam2
% xyz_keypts2=zeros(length(matches(2,:)),3);
% 
% %passar para 3D
% for ind=1:length(matches(2,:))
%     %xyz_keypts2(ind,3) =imgs2(v2(ind),u2(ind),i)*0.001; % Convert to meters
%     xyz_keypts2(ind,3) =bgimd2(v2(ind),u2(ind))*0.001; % Convert to meters
% end
% xyz_keypts2(:,1)= (xyz_keypts2(:,3)/K(1,1))' .*(u2-K(1,3)) ;
% xyz_keypts2(:,2) = (xyz_keypts2(:,3)/K(2,2))' .*(v2-K(2,3)) ;


%%
load('CalibrationData.mat');
load('calib_asus.mat');
%encontrar coordenadas 2D na camara rgb a partir de f1 e f2.
uv1rgb(:,1) = F1(1,matches(1,:)) ;
uv2rgb(:,1)= F2(1,matches(2,:)) ;
uv1rgb(:,2)= F1(2,matches(1,:)) ;
uv2rgb(:,2) = F2(2,matches(2,:)) ;

xyz1 = get_xyzasus(meddep1,[480 640],1:640*480,Depth_cam.K,1,0);
xyz2 = get_xyzasus(meddep2,[480 640],1:640*480,Depth_cam.K,1,0);

P3d_1=[xyz1(:,1)'; xyz1(:,2)'; xyz1(:,3)'; ones(1,length(xyz1(:,1)))]; 
P2d_1=RGB_cam.K*[R_d_to_rgb T_d_to_rgb]*P3d_1;    
uv_fromd1(:,1)=(P2d_1(1,:)./P2d_1(3,:))';
uv_fromd1(:,2)=(P2d_1(2,:)./P2d_1(3,:))';

P3d_2=[xyz2(:,1)'; xyz2(:,2)'; xyz2(:,3)'; ones(1,length(xyz2(:,1)))]; 
P2d_2=RGB_cam.K*[R_d_to_rgb T_d_to_rgb]*P3d_2;    
uv_fromd2(:,1)=(P2d_2(1,:)./P2d_2(3,:))';
uv_fromd2(:,2)=(P2d_2(2,:)./P2d_2(3,:))';

bestuv1=zeros(length(uv1rgb(:,1)),1);
bestuv2=zeros(length(uv2rgb(:,1)),1);

for i=1:length(uv1rgb(:,1))
    [~,bestuv1(i)]=min(sqrt((uv_fromd1(:,1)-uv1rgb(i,1)).^2+(uv_fromd1(:,2)-uv1rgb(i,2)).^2));
    [~,bestuv2(i)]=min(sqrt((uv_fromd2(:,1)-uv2rgb(i,1)).^2+(uv_fromd2(:,2)-uv2rgb(i,2)).^2));
    
end

xyz_keypts1=xyz1(bestuv1,:);
xyz_keypts2=xyz2(bestuv2,:);

%%
%how to find the threshold??
ransac_thresh=0.1; %using 10 cm for now
Niter_ransac=100;

best_inliers=[];
inliers=[];
aux=fix(rand(4*Niter_ransac,1)*length(xyz_keypts1))+1;
for k=1:Niter_ransac-4

    %get N random groups of 4 points
%     sample_index=randsample(length(xyz_keypts1),4);
%     xyz1_sample=xyz_keypts1(sample_index,:);
%     xyz2_sample=xyz_keypts2(sample_index,:);
    xyz1_sample=xyz_keypts1(aux(4*k:4*k+3),:);
    xyz2_sample=xyz_keypts2(aux(4*k:4*k+3),:);

   %for each group:
    
    %compute R and T from cam2 to cam 1 using procrustes 
    [d,xx,tr_trial]=procrustes(xyz1_sample,xyz2_sample,'scaling',false,'reflection',false); %computes transformation without scale and reflection
   
    %count the nr of inliers
    dist=xyz_keypts1-(xyz_keypts2*tr_trial.T+ones(length(xyz_keypts2),1)*tr_trial.c(1,:)); %distance between all keypoints
    inliers=find(sqrt(sum(dist.^2,2))<ransac_thresh);
    %keep tranformation and inliers if there are more inliers than the
    %maximum find until now
    if length(inliers)>length(best_inliers)
        best_inliers=inliers;
    end 
    inliers=[];
 
end

%check if the best inliers in 3d are good in the 2d rgb image
for i=1:length(best_inliers)
    P3d_2=[xyz_keypts2(best_inliers(i),1); xyz_keypts2(best_inliers(i),2); xyz_keypts2(best_inliers(i),3); 1]; 
    P2d_2=RGB_cam.K*[R_d_to_rgb T_d_to_rgb]*P3d_2;    
    uv_fromd2g(i,1)=P2d_2(1)/P2d_2(3);
    uv_fromd2g(i,2)=P2d_2(2)/P2d_2(3);
end
%eck if the best inliers in 3d are good in the 2d rgb image
for i=1:length(best_inliers)
    P3d_2=[xyz_keypts1(best_inliers(i),1); xyz_keypts1(best_inliers(i),2); xyz_keypts1(best_inliers(i),3); 1]; 
    P2d_2=RGB_cam.K*[R_d_to_rgb T_d_to_rgb]*P3d_2;    
    uv_fromd1g(i,1)=P2d_2(1)/P2d_2(3);
    uv_fromd1g(i,2)=P2d_2(2)/P2d_2(3);
end

figure
imshow(imgray1);
hold on;
scatter(uv_fromd1g(:,1),uv_fromd1g(:,2),'*');

figure
imshow(imgray2);
hold on;
scatter(uv_fromd2g(:,1),uv_fromd2g(:,2),'*');
%%
%recompute R and T using the model with more inliers?
[d,xx,tr]=procrustes(xyz_keypts1(best_inliers,:),xyz_keypts2(best_inliers,:),'scaling',false,'reflection',false);

%%
%ver se isto est� alguma coisa de jeito
d=dir('D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_*'); 
i=1;
im1=imread(['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_' d(i).name(12:end-3) 'png']);
    im2=imread(['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\rgb_image2_' d(i).name(12:end-3) 'png']);
    load(['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])
    dep1=depth_array;
    load(['D:\MEEC\4�ano\PIV\Project\NewData\maizena4\data_rgb\depth2_' d(i).name(12:end-3) 'mat'])
    dep2=depth_array;
    xyz1_=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz2_=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    rgbd1 = get_rgbd(xyz1_, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    rgbd2 = get_rgbd(xyz2_, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2_),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));

    figure;
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;

    