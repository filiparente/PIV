d=dir('D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_*'); %getting all the file in this directory
% O stor disse que ia mudar os nomes da imagem para dar por ordem!

%NAO ESQUECER DE MUDAR DIRETORIA CONSOANTE O PC

imgs1 = zeros(480,640,length(d));% para guardar os depth_arrays 
imgs2 = zeros(480,640,length(d));

for i=1:length(d),
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\depth1_' d(i).name(12:end-3) 'mat'])
    imgs1(:,:,i) = double(depth_array);
    imgs1(:,:,i)=ReplaceZeros(imgs1(:,:,i)); % remover zeros 
    load(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\depth2_' d(i).name(12:end-3 ) 'mat'])
    imgs2(:,:,i) = double(depth_array);
    imgs2(:,:,i)=ReplaceZeros(imgs2(:,:,i));        
end

%%

bg1q=quantile(imgs1,0.75,3);
bg2q=quantile(imgs2,0.75,3);

%%
%usar estas imagens ou o background a partir do rgb??
i=1;
imrgb1=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\rgb_image1_' d(i).name(12:end-3) 'png']);
imrgb2=imread(['D:\MEEC\4ºano\PIV\Project\NewData\maizena4\data_rgb\rgb_image2_' d(i).name(12:end-3) 'png']);
imgray1=rgb2gray(imrgb1);
imgray2=rgb2gray(imrgb2);
%%
%MEXER NOS PARAMETROS PARA VER O QUE DA MELHOR
peak_thresh=2;
edge_thresh=10;
[F1,d1]=vl_sift(single(imgray1));%'PeakThresh', peak_thresh,'edgethresh', edge_thresh);
[F2,d2]=vl_sift(single(imgray2));%,'PeakThresh', peak_thresh,'edgethresh', edge_thresh);


%%
%finding matches 

match_thresh=1.5;%default
[matches, scores]= vl_ubcmatch(d1,d2);%match_thresh);

%%

figure(2) ; clf ;
imagesc(cat(2, imrgb1, imrgb2)) ;

u1 = F1(1,matches(1,:)) ;
v1 = F2(1,matches(2,:)) + size(imrgb2,2) ;
u2 = F1(2,matches(1,:)) ;
v2 = F2(2,matches(2,:)) ;

hold on ;
h = line([u1 ; v1], [u2 ; v2]) ;
set(h,'linewidth', 1, 'color', 'b') ;

vl_plotframe(F1(:,matches(1,:))) ;
F2(1,:) = F2(1,:) + size(imrgb2,2) ;
vl_plotframe(F2(:,matches(2,:))) ;
axis image off ;

%%

%encontrar coordenadas 3d a partir de f1 e f2.


    
