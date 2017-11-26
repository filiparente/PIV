d=dir('D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_*'); %getting all the file in this directory
% O stor disse que ia mudar os nomes da imagem para dar por ordem!

%NAO ESQUECER DE MUDAR DIRETORIA CONSOANTE O PC

imgs1 = zeros(480,640,length(d));% para guardar os depth_arrays 
imgs2 = zeros(480,640,length(d));



for i=1:length(d),
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])
    dep1=depth_array;
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\depth2_' d(i).name(12:end-3 ) 'mat'])
    dep2=depth_array;
    imgs1(:,:,i) = double(dep1);
    imgs1(:,:,i)=ReplaceZeros(imgs1(:,:,i)); % remover zeros 
    imgs2(:,:,i) = double(dep2);
    imgs2(:,:,i)=ReplaceZeros(imgs2(:,:,i));
    
end

bg1q=quantile(imgs1,0.75,3);
bg2q=quantile(imgs2,0.75,3);

%%

bg_threshold=70; 
region_counts=2000; %%como definir o nr de counts??
knn=20; 
edgethresh=0.05;
%gradient_threshold=0.05;
gradient_factor=0.1;

xyz=cell(length(d),2);% usar cell arrays para conseguirmos obter xyz de todos os objetos em todas as imagens 
                      %porque o tamanho vai variar consoante a imagem
                   
% xyz21=cell(1,length(d)); %camera 2 points in camera 1 reference system
%background=0
goodxyz=cell(length(d),2);
limits=cell(length(d),2); %limits of the box (xmin,xmax,ymin,ymax,zmin,zmax)
col=cell(length(d),2);
row=cell(length(d),2);


% POR XYZ COMO CELL

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
    xyz{i,1}=cell(length(labelobj1),1); 
            for k=1:length(labelobj1)
                [row{i,1}{k} col{i,1}{k} xyz{i,1}{k}]=Obj3DCoord(labelobj1,k,L1,imgs1(:,:,i),Depth_cam.K); 
            
            end
   
    L2 = bwlabel(bw2);
    counts2=histcounts(L2(:));  % counts for the labels
    %finding object with counts above region_counts
    labelobj2=find(counts2(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background
    
    % getting the 3D coordinates of each pixel
    xyz{i,2}=cell(length(labelobj2),1); 
   
            for k=1:length(labelobj2)
                [row{i,2}{k} col{i,2}{k} xyz{i,2}{k}]=Obj3DCoord(labelobj2,k,L2,imgs2(:,:,i),Depth_cam.K);
            end

     
    for cam=1:2       
        for j=1:length(xyz{i,cam})
            if length(xyz{i,cam}{j})<17000
                goodxyz{i,cam}=[goodxyz{i,cam} ConnComponents(xyz{i,cam}{j},knn,edgethresh,region_counts)]; %adding cells
            else
                if cam==1
                    img=imgs1(:,:,i);
                else
                    img=imgs2(:,:,i);
                end
                goodxyz{i,cam}=[goodxyz{i,cam} gradient(row{i,cam}{j},col{i,cam}{j},img,gradient_factor, region_counts, Depth_cam.K)]
            end
        end
        
         for k=1:length(goodxyz{i,cam})
            limits{i,cam}{k}(1)=min(goodxyz{i,cam}{k}(:,1));
            limits{i,cam}{k}(2)=max(goodxyz{i,cam}{k}(:,1));
            limits{i,cam}{k}(3)=min(goodxyz{i,cam}{k}(:,2));
            limits{i,cam}{k}(4)=max(goodxyz{i,cam}{k}(:,2));
            limits{i,cam}{k}(5)=min(goodxyz{i,cam}{k}(:,3));
            limits{i,cam}{k}(6)=max(goodxyz{i,cam}{k}(:,3));
        end
    end

 
    % to see the images while the program is running
%     imagesc(bw1);
%     colormap(gray);
%     pause(1); 
    
end





%%

% ao descobrir os limites temos que ter em atenção que podem haver imagens
% sem nada!!
for cam=1:2
    for img=1:length(d)
        for obj=1:length(limits{img,cam})
            xmin=limits{img,cam}{1,obj}(1);
            xmax=limits{img,cam}{1,obj}(2);
            ymin=limits{img,cam}{1,obj}(3);
            ymax=limits{img,cam}{1,obj}(4);
            zmin=limits{img,cam}{1,obj}(5);
            zmax=limits{img,cam}{1,obj}(6);


            %testando

            im1=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_' d(i).name(12:end-3) 'png']);
            load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])

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
               
            if(obj==1)
                p=pcmerge(p6,p_region,0.001)); %merge da sala com a caixa do primeiro objeto
                
            if(length(limits{img,cam})>1 && obj~=1) %se houver mais do que um objeto, faz merge
                p=pcmerge(p, p_ant, 0.001);
                
            p_ant=p;
            
            pause(1); 
            end
        pcshow(p);
    end
end

