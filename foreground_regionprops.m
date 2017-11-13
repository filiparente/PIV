d=dir('D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_*'); %getting all the file in this directory
% O stor disse que ia mudar os nomes da imagem para dar por ordem!

imgs1 = zeros(480,640,length(d));% para guardar os depth_arrays 
imgs2 = zeros(480,640,length(d));
% dep1= zeros(480,640,length(d),'uint16');
% dep2 = zeros(480,640,length(d),'uint16');
% im1 = zeros(480,640,3,length(d),'uint8');
% im2 = zeros(480,640,3,length(d)'uint8');

bg1_threshold=40;
bg2_threshold=40 ;
region_counts=2500;

for i=1:length(d),
    im1=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_' d(i).name(12:end-3) 'png']);
    im2=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image2_' d(i).name(12:end-3) 'png']);
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])
    dep1=depth_array;
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\depth2_' d(i).name(12:end-3 ) 'mat'])
    dep2=depth_array;
    imgs1(:,:,i) = double(dep1);%/1000 <-- caso quisessemos já obter em metros;
    imgs2(:,:,i) = double(dep2);%/1000;
end

bg1=median(imgs1,3);
bg2=median(imgs2,3);

%%% --> ver imfill

figure(1);
imagesc(bg1);
figure(2);
imagesc(bg2);


% TIRAR OS ZEROS DO BACKGROUND - INTERPOLAÇAO

for i = 1:length(d)
    % obter o foreground em cada imagem e a bounding box de cada objeto
    fg1(:,:,i) = abs(imgs1(:,:,i)-bg1)> bg1_threshold; %returns 0 or 1
    fg2(:,:,i) = abs(imgs2(:,:,i)-bg2)> bg2_threshold;
    
    lbls1=bwlabel(fg1(:,:,i));
    lbls2=bwlabel(fg2(:,:,i)); % descobrir regiões conexas
    counts1=histcounts(lbls1(:),max(lbls1(:)));
    counts2=histcounts(lbls2(:),max(lbls2(:)));
    
    objs1=ismember(lbls1,find(counts1(2:end)>region_counts));
    objs2=ismember(lbls2,find(counts2(2:end)>region_counts));
    stats1(:,i)=regionprops(objs1,'BoundingBox');
    stats2(:,i)=regionprops(objs2,'BoundingBox');
    
    % arranjar maneira de passar a box para a point cloud
    
    
    
    
    % show all point clouds not merged
%     dep1_aux=dep1(:,:,i);
%     dep2_aux=dep2(:,:,i);
%     xyz1=get_xyzasus(dep1_aux(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
%     xyz2=get_xyzasus(dep2_aux(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
%     
%     im1=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_' d(i).name(12:end-3) 'png']);
%     im2=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image2_' d(i).name(12:end-3) 'png']);
%     rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
%     rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
%     
%     pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
%     pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));
%     figure(3);hold off;
%     showPointCloud(pc1)
%     drawnow;
%     figure(4);%hold off;
%     showPointCloud(pc2)
%     drawnow;
        
end

im1=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_7.png']);
figure
imshow(im1);
hold on;
rectangle('Position',stats1(1,9).BoundingBox, 'EdgeColor', 'r');


%%
% só para testar com uma imagem
im1=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_3.png']);
load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\depth1_3.mat']);


xyz=get_xyzasus(depth_array(:),[480 640],1:640*480,Depth_cam.K,1,0);
%Compute "virtual image" aligned with depth
rgbd=get_rgbd(xyz,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
cl=reshape(rgbd,480*640,3);
p=pointCloud(xyz,'Color',cl);
figure
showPointCloud(p);

%points=round(bbox2points(stats(1,5).BoundingBox));

umin=round(stats(1,5).BoundingBox(1));
umax=round(stats(1,5).BoundingBox(1)+stats(1,5).BoundingBox(3));
vmin=round(stats(1,5).BoundingBox(2));
vmax=round(stats(1,5).BoundingBox(2)+stats(1,5).BoundingBox(4));

xmin=double(depth_array(vmin,umin))*0.001/Depth_cam.K(1,1) * (umin-Depth_cam.K(1,3));
ymin=double(depth_array(vmin,umin))*0.001/Depth_cam.K(2,2) * (vmin-Depth_cam.K(2,3));
zmin=double(depth_array(vmin,umin))*0.001;
xmax=double(depth_array(vmax,umax))*0.001/Depth_cam.K(1,1) * (umax-Depth_cam.K(1,3));
ymax=double(depth_array(vmax,umax))*0.001/Depth_cam.K(2,2) * (vmax-Depth_cam.K(2,3));
zmax=double(depth_array(vmax,umax))*0.001;

figure



%%
region=[xyz;
        xmin ymin zmin;
        xmin ymin zmax;
        xmin ymax zmin;
        xmin ymax zmax;
        xmax ymin zmin;
        xmax ymin zmax;
        xmax ymax zmin;
        xmax ymax zmax];
region_color=[255 0 0];
color=[cl;
       repmat(region_color,8,1)];
p_region=pointCloud(region, 'Color', color);

figure
showPointCloud(p_region);