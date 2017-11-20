d=dir('D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_*'); %getting all the file in this directory
% O stor disse que ia mudar os nomes da imagem para dar por ordem!

%NAO ESQUECER DE MUDAR DIRETORIA CONSOANTE O PC

imgs1 = zeros(480,640,length(d));% para guardar os depth_arrays 
imgs2 = zeros(480,640,length(d));



for i=1:length(d),
    load(['D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])
    dep1=depth_array;
    load(['D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\depth2_' d(i).name(12:end-3 ) 'mat'])
    dep2=depth_array;
    imgs1(:,:,i) = double(dep1);
    imgs1(:,:,i)=ReplaceZeros(imgs1(:,:,i)); % remover zeros 
    imgs2(:,:,i) = double(dep2);
    imgs2(:,:,i)=ReplaceZeros(imgs2(:,:,i));
    
end

bg1q=quantile(imgs1,0.75,3);
bg2q=quantile(imgs2,0.75,3);

%%
% GET R AND T BETWEEN THE CAMERA AND THE WORLD REFERENCE FRAME USING
% PROCRUSTES
% we choose one of the cameras reference systems as the world system -
% camera 1 in this case

%Procrustes is applied to a pair of images:
im1=imread('D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_3.png');
im2=imread('D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\rgb_image2_3.png');
load('D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\depth1_3.mat')
dep1=depth_array;
load('D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\depth2_3.mat')
dep2=depth_array;

transf=GetProcrustes(im1,im2,dep1,dep2,Depth_cam.K, RGB_cam.K, R_d_to_rgb,T_d_to_rgb); %returns a structure with the transformation
%rotation:transf.T 
%translation:transf.c


%%

bg_threshold=70; 
region_counts=3500; 


xyz1=cell(1,length(d));% usar cell arrays para conseguirmos obter xyz de todos os objetos em todas as imagens 
                      %porque o tamanho vai variar consoante a imagem
                      
xyz2=cell(1,length(d));
xyz21=cell(1,length(d)); %camera 2 points in camera 1 reference system
%background=0
for i = 1:length(d)
    
    fg1q(:,:,i) = abs(imgs1(:,:,i)-bg1q)> bg_threshold; %returns 0 or 1   
    fg2q(:,:,i) = abs(imgs2(:,:,i)-bg2q)> bg_threshold; %returns 0 or 1
  
    
    %removing z>3000 and erasing small objs
    bw1=RemoveSmallObjs(imgs1(:,:,i),fg1q(:,:,i),3000);
    bw2=RemoveSmallObjs(imgs2(:,:,i),fg2q(:,:,i),3000);
    
    L1 = bwlabel(bw1);
    counts1=histcounts(L1(:));  % counts for the labels
    %finding object with counts above region_counts
    labelobj1=find(counts1(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background
    
    % the 3D coordinates of each pixel
    xyz1{i}=cell(length(labelobj1),1); 
            for k=1:length(labelobj1)
                xyz1{i}{k}=Obj3DCoord(labelobj1,k,L1,imgs1(:,:,i),Depth_cam.K); 
            end
   
    L2 = bwlabel(bw2);
    counts2=histcounts(L2(:));  % counts for the labels
    %finding object with counts above region_counts
    labelobj2=find(counts2(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background
    
    % getting the 3D coordinates of each pixel
    xyz2{i}=cell(length(labelobj2),1); 
    xyz21{i}=cell(length(labelobj2),1); 
            for k=1:length(labelobj2)
                xyz2{i}{k}=Obj3DCoord(labelobj2,k,L2,imgs2(:,:,i),Depth_cam.K);
               %xyz21{i}{k}=xyz2{i}{k}*transf.T+ones(length(xyz2{i}{k}),1)*transf.c(1,:); %transform coordinates in camera2 to camera1 reference system
            end

    % to see the images while the program is running
    imagesc(bw2);
    colormap(gray);
    pause(1); 
end

%% Histogram

clear depth
depth=cell(1,length(d));
Threshold=20;
Threshold = 20;
good_inds = 1;

for j=1:length(d) 
    depth{j}=cell(1,length(xyz2{j}));
    for k=1:length(xyz2{j}) %for each object
        nbins=(max(xyz2{j}{k}(:,3))-min(xyz2{j}{k}(:,3)))*100;
for j=1:length(d)  % for each image
    depth{j}=cell(1,length(xyz2{j})); %each image has a cell which size is the number of objects
    for k=1:length(xyz2{j}) %for each object; k also represents how many histograms
        
        nbins=(max(xyz2{j}{k}(:,3))-min(xyz2{j}{k}(:,3)))*100; %number of columns for each histogram depends on width
   
        [counts,edges]=histcounts(xyz2{j}{k}(:,3),round(nbins));
        %obj=1;
        [counts,edges]=histcounts(xyz2{j}{k}(:,3),round(nbins)); %histogram; counts is the number of pixels; edges are the limits on the depth for each column
        
        if(max(counts)*0.0001 > Threshold)  %adaptative threshold, with maximum ???
            Threshold = max(counts)*0.0001;
            
        flag=0; % no object found
        
        for i=1:length(counts)

            if counts(i) > Threshold && flag==0
                depth{j}{k}(1) = edges(i); %depth(1)= z_min
                depth{j}{k}{good_inds}(1) = edges(i); %depth(1)= z_min
                flag=1; %object found

            end

            if flag && (counts(i) < Threshold || i==length(counts)) %object found and below threshold
                 if i==length(counts)
                     depth{j}{k}(2) = edges(end);
                     depth{j}{k}{good_inds}(2) = edges(end);
                 else
                    depth{j}{k}(2) = edges(i); %depth(2) = z_max
                    depth{j}{k}{good_inds}(2) = edges(i); %depth(2) = z_max
                 end
                flag=0; %object is over
                k=k+1;
                good_inds=good_inds+1;
            end  
        end  
    end

  % pause(0.5)
end

%%
t_size=300;%threshold para eliminar objs demasiado pequenos que sobrevivem ao histograma
t_size=300; %threshold to ignore objects too small that survived to the histogram
clear limits
limits=cell(1,length(d));

for i=1:length(d) %for every image   
    good_ind=0;
    for j=1:length(xyz2{i}) %objects      
        for k=1:length(depth{i}) %% VERIFICAR ISTO
    for j=1:length(xyz2{i}) %histograms      
        for k=1:length(depth{i}{j})
            
           ind1 = find(xyz2{i}{j}(:,3)>=depth{i}{k}(1));
           ind2 = find(xyz2{i}{j}(:,3)<= depth{i}{k}(2));
           ind = intersect(ind1,ind2); % save the labels of the xyz with z coordinates between the max and the min found before
           ind1 = find( xyz2{i}{j}(:,3) >= depth{i}{j}{k}(1) ); %depth{i}{j}{k} means in image i, histogram j and identified object k; (1) is z_min for that object
           ind2 = find( xyz2{i}{j}(:,3) <= depth{i}{j}{k}(2) ); %(2) is zmax
           ind  = intersect(ind1,ind2); % save the labels of the xyz with z coordinates between the max and the min found before

           if(size(ind)<t_size)
           if(size(ind) < t_size)
               continue;
           end

           good_ind=good_ind+1; % s� queremos adicionar aos limits se o size for suficientemente grande
           xinto=xyz2{i}{j}(ind,1);
           good_ind=good_ind+1; % we only add to limits the objects that satisfy the requirement for the threshold size          xinto=xyz2{i}{j}(ind,1);
           yinto=xyz2{i}{j}(ind,2);     

           limits{i}{good_ind}(1)=min(xinto);
           limits{i}{good_ind}(2)=max(xinto);
           limits{i}{good_ind}(3)=min(yinto);
           limits{i}{good_ind}(4)=max(yinto);
           limits{i}{good_ind}(5)=depth{i}{k}(1);
           limits{i}{good_ind}(6)=depth{i}{k}(2);
           limits{i}{good_ind}(1)=min(xinto); %xmin 
           limits{i}{good_ind}(2)=max(xinto); %xmax
           limits{i}{good_ind}(3)=min(yinto); %ymin
           limits{i}{good_ind}(4)=max(yinto); %ymax
           limits{i}{good_ind}(5)=depth{i}{j}{k}(1); %zmin
           limits{i}{good_ind}(6)=depth{i}{j}{k}(2); %zmax
           
        end
   end
end

%%
% s� para testar com uma imagem
im1=imread(['D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_10.png']);
load(['D:\MEEC\4�ano\PIV\Project\NewData\maizena2\data_rgb\depth1_10.mat']);


xyz6=get_xyzasus(depth_array(:),[480 640],1:640*480,Depth_cam.K,1,0);
%Compute "virtual image" aligned with depth
rgbd6=get_rgbd(xyz6,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
cl6=reshape(rgbd6,480*640,3);
p6=pointCloud(xyz6,'Color',cl6);
% figure
% showPointCloud(p);

%points=round(bbox2points(stats(1,5).BoundingBox));

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

pcshow(pcmerge(p6,p_region,0.001));





