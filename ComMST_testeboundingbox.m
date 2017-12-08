d=dir('D:\MEEC\4�ano\PIV\Project\NewData\lab1\rgb_image1_*'); %getting all the file in this directory
% O stor disse que ia mudar os nomes da imagem para dar por ordem!

%NAO ESQUECER DE MUDAR DIRETORIA CONSOANTE O PC

imgs1 = zeros(480,640,length(d));% para guardar os depth_arrays 
imgs2 = zeros(480,640,length(d));

for i=1:length(d),
    load(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\depth1_' d(i).name(12:end-3) 'mat'])
    imgs1(:,:,i) = double(depth_array);
    imgs1(:,:,i)=ReplaceZeros(imgs1(:,:,i)); % remover zeros 
    load(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\depth2_' d(i).name(12:end-3 ) 'mat'])
    imgs2(:,:,i) = double(depth_array);
    imgs2(:,:,i)=ReplaceZeros(imgs2(:,:,i));        
end

bg1q=quantile(imgs1,0.75,3);
bg2q=quantile(imgs2,0.75,3);
%%
d=dir('D:\MEEC\4�ano\PIV\Project\NewData\lab1\rgb_image1_*'); %getting all the file in this directory

    ims1=[];
    ims2=[];
    imgs1=[];
    imgs2=[];
    for i=1:length(d),
        im1=rgb2gray(imread(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\rgb_image1_' d(i).name(12:end-3) 'png']));
        ims1=[ims1 im1(:)];
        im2=rgb2gray(imread(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\rgb_image2_' d(i).name(12:end-3) 'png']));
        ims2=[ims2 im2(:)];
        load(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\depth1_' d(i).name(12:end-3) 'mat'])
        imgs1(:,:,i) = double(depth_array);
        load(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\depth2_' d(i).name(12:end-3) 'mat'])
        imgs2(:,:,i) = double(depth_array);
    end

    %medim1=median(double(ims1),2);
    %medim2=median(double(ims2),2);
    medim1=quantile(double(ims1),0.6,2);
    medim2=quantile(double(ims2),0.6,2);
    imgray1=(uint8(reshape(medim1,[480 640])));
    imgray2=(uint8(reshape(medim2,[480 640])));

    bg1q=quantile(imgs1,0.6,3);
    bg2q=quantile(imgs2,0.6,3);

%%
%variables declaration 
%values for part1
bg_threshold=70; %threshold use to remove background
region_counts=2600; %threshold used to eliminate objects with small nr of pixels
z_cut=5000; 
knn=10; %number of nearest neighbours used in knnsearch
edgethresh=0.1; %ou 0.1?? %threshold used to remove big edges in the mst
gradient_factor=0.1;    %threhsold used in the gradient approach
mst_maxsize=50000; %maximum size to use mst approach
mst_grid=0.007; %get one point per "cube" of mst_grid mm

%values for part2
peak_thresh=0; % increase to limit ; default is 0
edge_thresh=20; %decrease to limit ; default is 10
match_thresh=1.5;
ransac_thresh=0.05;
Ninter_ransac=100;



load('CalibrationData.mat');
load('calib_asus.mat');

xyz = {};
row = {};
col={};
goodxyz = {};
limits=cell(length(d),2); %limits of the box (xmin,xmax,ymin,ymax,zmin,zmax)

%%
[R T]= RT_c2toc1(peak_thresh,edge_thresh, match_thresh, ransac_thresh, Ninter_ransac,imgray1,imgray2, bg1q, bg2q)


%%
%for i = 1:length(d)
i=5;
    fg1q = abs(imgs1(:,:,i)-bg1q)> bg_threshold; %returns 0 or 1   
    fg2q = abs(imgs2(:,:,i)-bg2q)> bg_threshold; %returns 0 or 1
    
    %removing z>3000 and erasing small objs
    bw1=RemoveSmallObjs(imgs1(:,:,i),fg1q,z_cut);
    bw2=RemoveSmallObjs(imgs2(:,:,i),fg2q,z_cut);
    
    L1 = bwlabel(bw1);  
    
    counts1=histcounts(L1(:));  % counts for the labels
    %finding object with counts above region_counts
    labelobj1=find(counts1(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background
 
    % the 3D coordinates of each pixel 
    for k=1:length(labelobj1)
        [row{k} col{k} xyz{k}]=Obj3DCoord(labelobj1,k,L1,imgs1(:,:,i),Depth_cam.K);            
    end        
    %clearing unnecessary variables
    clear labelobj1;
    clear counts1;

    %getting the 3D coords of each object without overlapping and background
    for j=1:length(xyz)
%             if length(xyz{j})<mst_maxsize
                goodxyz=[goodxyz ConnComponents(xyz{j},knn,edgethresh,region_counts,mst_grid)]; %adding cells
%             else
%                 goodxyz=[goodxyz gradient(row{j},col{j},imgs1(:,:,i),gradient_factor, region_counts, Depth_cam.K)];
%             end
    end
    
    %getting the limits of the box for each object
    cam=1;
    for k=1:length(goodxyz)
        
            [~,cornerpoints,~,~,~] = minboundbox(goodxyz{k}(:,1),goodxyz{k}(:,2),goodxyz{k}(:,3))
            limits{i,cam}{k}=cornerpoints;
%         
%             limits{i,cam}{k}(1)=min(goodxyz{k}(:,1));
%             limits{i,cam}{k}(2)=max(goodxyz{k}(:,1));
%             limits{i,cam}{k}(3)=min(goodxyz{k}(:,2));
%             limits{i,cam}{k}(4)=max(goodxyz{k}(:,2));
%             limits{i,cam}{k}(5)=min(goodxyz{k}(:,3));
%             limits{i,cam}{k}(6)=max(goodxyz{k}(:,3));
    end
   
    %clearing variables
    xyz=[];
    goodxyz=[]; 
    row=[];
    col=[];
   
    L2 = bwlabel(bw2);
    counts2=histcounts(L2(:));  % counts for the labels
    %finding object with counts above region_counts
    labelobj2=find(counts2(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background
    
%     
% %     % to see the images while the program is running
%     imagesc(L2);
%     %colormap(gray);
%     pause(1);
    
    % the 3D coordinates of each pixel 
    for k=1:length(labelobj2) %starting in index 2 because 1 corresponds to the background
        [row{k} col{k} xyz{k}]=Obj3DCoord(labelobj2,k,L2,imgs2(:,:,i),Depth_cam.K);            
    end
        
    %clearing unnecessary variables
    clear labelobj2;
    clear counts2;
   
    %getting the 3D coords of each object without overlapping and background
    for j=1:length(xyz)
           % if length(xyz{j})<mst_maxsize
                goodxyz=[goodxyz ConnComponents(xyz{j},knn,edgethresh,region_counts,mst_grid)]; %adding cells
           % else
           %     goodxyz=[goodxyz gradient(row{j},col{j},imgs2(:,:,i),gradient_factor, region_counts, Depth_cam.K)];
           % end
    end
    
    %getting the limits of the box for each object
    cam=2;
    for k=1:length(goodxyz)

            [~,cornerpoints,~,~,~] = minboundbox(goodxyz{k}(:,1),goodxyz{k}(:,2),goodxyz{k}(:,3))
            limits{i,cam}{k}=cornerpoints;
            
%             limits{i,cam}{k}(1)=min(goodxyz{k}(:,1));
%             limits{i,cam}{k}(2)=max(goodxyz{k}(:,1));
%             limits{i,cam}{k}(3)=min(goodxyz{k}(:,2));
%             limits{i,cam}{k}(4)=max(goodxyz{k}(:,2));
%             limits{i,cam}{k}(5)=min(goodxyz{k}(:,3));
%             limits{i,cam}{k}(6)=max(goodxyz{k}(:,3));
    end
   
    %clearing variables
    xyz=[];
    goodxyz=[]; 
    row=[];
    col=[];   
%  
%     clear xyz;
%     clear row;
%     clear col;
%     clear goodxyz;
    

    
%end





%% testing
img=10;
cam=1;
obj=1;  
i=img;
xmin=limits{img,cam}{1,obj}(1);
            xmax=limits{img,cam}{1,obj}(2);
            ymin=limits{img,cam}{1,obj}(3);
            ymax=limits{img,cam}{1,obj}(4);
            zmin=limits{img,cam}{1,obj}(5);
            zmax=limits{img,cam}{1,obj}(6);


            %testando

            im1=imread(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\rgb_image1_' d(i).name(12:end-3) 'png']);
            load(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\depth1_' d(i).name(12:end-3) 'mat'])

            xyz6=get_xyzasus(depth_array(:),[480 640],1:640*480,Depth_cam.K,1,0);
            %Compute "virtual image" aligned with depth
            rgbd6=get_rgbd(xyz6,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
            cl6=reshape(rgbd6,480*640,3);
            p6=pointCloud(xyz6,'Color',cl6);


            %p=pointCloud(goodxyz{img}{obj});

            region=[ xmin ymin zmin;
                    xmin ymin zmax;
                    xmin ymax zmin;
                    xmin ymax zmax;
                    xmax ymin zmin;
                    xmax ymin zmax;
                    xmax ymax zmin;
                    xmax ymax zmax];
            region_color=[255 0 0];
            color=[%cl;
            repmat(region_color,8,1)];
            p_region=pointCloud(region, 'Color', color);

            figure
            axis equal
            plot(alphaShape(region),'FaceColor', 'none', 'FaceAlpha',1.0);
            hold on

            pcshow(p6);
           % pcshow(pcmerge(p6,p_region,0.001));

%%
% ao descobrir os limites temos que ter em aten��o que podem haver imagens
% sem nada!!
%for cam=1:2
cam=1;
    for img=1:length(d)
   %img=10;
        figure
         im1=imread(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\rgb_image1_' d(img).name(12:end-3) 'png']);
            load(['D:\MEEC\4�ano\PIV\Project\NewData\lab1\depth1_' d(img).name(12:end-3) 'mat'])

            xyz6=get_xyzasus(depth_array(:),[480 640],1:640*480,Depth_cam.K,1,0);
            %Compute "virtual image" aligned with depth
            rgbd6=get_rgbd(xyz6,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
            cl6=reshape(rgbd6,480*640,3);
            p6=pointCloud(xyz6,'Color',cl6);
            pcshow(p6);
            hold on;
            
        for obj=1:length(limits{img,cam})
            xmin=limits{img,cam}{1,obj}(1);
            xmax=limits{img,cam}{1,obj}(2);
            ymin=limits{img,cam}{1,obj}(3);
            ymax=limits{img,cam}{1,obj}(4);
            zmin=limits{img,cam}{1,obj}(5);
            zmax=limits{img,cam}{1,obj}(6);

            region=[ xmin ymin zmin;
                    xmin ymin zmax;
                    xmin ymax zmin;
                    xmin ymax zmax;
                    xmax ymin zmin;
                    xmax ymin zmax;
                    xmax ymax zmin;
                    xmax ymax zmax];
            region_color=[255 0 0];
            color=[%cl;
            repmat(region_color,8,1)];
            %p_region=pointCloud(region, 'Color', color);

           % figure
           % axis equal
            plot(alphaShape(region),'FaceColor', 'none', 'FaceAlpha',1.0);
            hold on;
           % hold on
               
%             if(obj==1)
%                 p=pcmerge(p6,p_region,0.001); %merge da sala com a caixa do primeiro objeto
%             end
%                 
%             if(length(limits{img,cam})>1 && obj~=1) %se houver mais do que um objeto, faz merge
%                 p=pcmerge(p_region, p_ant, 0.001);
%             end
%                 
%             p_ant=p;
            
            pause(1); 
            end
       
   end
%end

