d=dir('D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\rgb_image1_*'); %getting all the file in this directory
% O stor disse que ia mudar os nomes da imagem para dar por ordem!

imgs1 = zeros(480,640,length(d));% para guardar os depth_arrays 
imgs2 = zeros(480,640,length(d));

bg1_threshold=40;
bg2_threshold=40;
region_counts=2500;

for i=1:length(d),
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])
    dep1=depth_array;
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena2\data_rgb\depth2_' d(i).name(12:end-3 ) 'mat'])
    dep2=depth_array;
    imgs1(:,:,i) = double(dep1);%/1000 <-- caso quisessemos já obter em metros;
    imgs1(:,:,i)=regionfill(imgs1(:,:,i),imgs1(:,:,i) == 0);
    imgs2(:,:,i) = double(dep2);%/1000;  
    
end

bg1q=quantile(imgs1,0.75,3);
bg2q=quantile(imgs2,0.75,3);

xyz=cell(1,length(d));% para conseguirmos obter xyz de todos os objetos em todas as imagens
%background=0
for i = 1:length(d)
    
    fg1q(:,:,i) = abs(imgs1(:,:,i)-bg1q)> bg1_threshold; %returns 0 or 1   
    fg2q(:,:,i) = abs(imgs2(:,:,i)-bg2q)> bg2_threshold; %returns 0 or 1
  
    % if it is too far from the camera (Z>3m) then it's not an object
    [yz,xz] = find(imgs1(:,:,i)>3000);
    for j=1:length(xz)
        fg1q(yz(j),xz(j),i)=0; 
    end
  
    clear xz;
    clear yz;
    
    
%   REMOVING SMALL OBJECTS
%     bw=bwmorph(fg1q(:,:,i),'open');
   
%    se = strel('disk',5); % ou rectangle[5 5]?
%    afterOpening = imopen(fg1q(:,:,i),se);
%    bw = imclose(afterOpening,se);

    SE = strel('disk',5);
    bw= imerode(fg1q(:,:,i),SE);
    bw= imdilate(bw,SE);
    
    L = bwlabel(bw);
    counts1=histcounts(L(:),max(L(:)));  % counts for the labels
    %finding object with counts above region_counts
    labelobj=find(counts1(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background
    
    xyz{i}=cell(3,length(labelobj));
    for k=1:length(labelobj)
        
        r=cell(1,length(labelobj)); 
        c=cell(1,length(labelobj));
        [r{k},c{k}]=find(L==labelobj(k)); %getting the coordinates of each object pixel

       %passar para 3D 
        for l=1:length(r{k})
            xyz{i}{3,k}(l) =imgs1(r{k}(l),c{k}(l),5)*0.001; % Convert to meters
        end
        
        xyz{i}{3,k}=xyz{i}{3,k}';
        xyz{i}{1,k} = (xyz{i}{3,k}/Depth_cam.K(1,1)) .*(c{k}-Depth_cam.K(1,3)) ;
        xyz{i}{2,k} = (xyz{i}{3,k}/Depth_cam.K(2,2)) .*(r{k}-Depth_cam.K(2,3)) ;  
        
        clear r;
    	clear c;     
        
    end
 
    % to see the images while the program is running
%     imagesc(bw);
%     colormap(gray);
%     pause(1);
   
end
