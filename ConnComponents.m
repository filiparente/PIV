function [newxyz] = ConnComponents(xyz,knn,edgethreshold,counts_size,mst_grid) 
% xyz - vector of 3D coordinates
% knn - number of nearest neighbors
% edgethreshold - threshold for the edge size
% counts_size - threshold for the number of points in each object

p=pointCloud(xyz);
paux=pcdownsample(p,'gridAverage',mst_grid);  
xyzaux=paux.Location;
% length(xyzaux)

minsize=counts_size*length(xyzaux)/length(xyz);%new min size 

[IDX,Dist] = knnsearch(xyzaux,xyzaux,'K',knn); %finding the nearest neighbors of each point and the corresponding distance

Adj=zeros(length(IDX(:,1))); % adjacency matrix
for i=1:length(IDX(:,1))
   for j=2:knn %começa em 2 porque o primeiro vizinho é o proprio ponto?
        %filling the matrix with distances
        Adj(i,IDX(i,j))=Dist(i,j);%norm(xyz(i,:)-xyz(IDX(i,j),:));
        Adj(IDX(i,j),i)=Adj(i,IDX(i,j)); %because the matrix is symmetric
   end   
end

clear IDX;
clear Dist;
tf = issymmetric(Adj)
gdist=graph(Adj); %obtaining the graph

[Tree,pred]=minspantree(gdist,'Method','sparse','Type','forest'); %type forest to calculate the minimum spanning tree of all connected components in the graph

clear gdist;
clear Adj;

junk=find(Tree.Edges.Weight >edgethreshold);
if(junk)
    Tree=rmedge(Tree,junk); %removing big edges 
end
binx=conncomp(Tree,'OutputForm','cell');   %bin numbers indicate which component each node in the graph belongs to

clear Tree;
%removing small connected components
for(i=1:length(binx))
    if length(binx{i})<minsize
        binx{i}=[];
    end
end
binx(cellfun('isempty',binx)) = []; %apagar cells vazias

newxyz=cell(1,length(binx)); % cell array contaning the xyz coordinates of each object
                          % each cell corresponds to one object
for obj=1:length(binx)
    newxyz{obj}=xyzaux(binx{1,obj},:);
end


end