function [newxyz] = ConnComponents(xyz,knn,edgethreshold,counts_size) 
% xyz - vector of 3D coordinates
% knn - number of nearest neighbors
% edgethreshold - threshold for the edge size
% counts_size - threshold for the number of points in each object


% if length(xyz(:,1))> 15000
%     xyzaux=zeros(round(length(xyz)/2),3);
%     xyzaux=xyz(1:2:end,:);
% else
% end


[IDX,Dist] = knnsearch(xyz,xyz,'K',knn); %finding the nearest neighbors of each point and the corresponding distance
Adj=zeros(length(IDX(:,1))); % adjacency matrix
for i=1:length(IDX(:,1))
   for j=2:knn %começa em 2 porque o primeiro vizinho é o proprio ponto?
        %filling the matrix with distances
        Adj(i,IDX(i,j))=Dist(i,j);%norm(xyz(i,:)-xyz(IDX(i,j),:));
        Adj(IDX(i,j),i)=Adj(i,IDX(i,j)); %because the matrix is symmetric
   end   
end
length(IDX(:,1))
gdist=graph(Adj); %obtaining the graph

[Tree,pred]=minspantree(gdist,'Method','sparse','Type','forest'); %type forest to calculate the minimum spanning tree of all connected components in the graph
junk=find(Tree.Edges.Weight >edgethreshold);
if(junk)
    Tree=rmedge(Tree,junk); %removing big edges 
end
binx=conncomp(Tree,'OutputForm','cell');   %bin numbers indicate which component each node in the graph belongs to

%removing small connected components
for(i=1:length(binx))
    if length(binx{i})<counts_size
        binx{i}=[];
    end
end
binx(cellfun('isempty',binx)) = []; %apagar cells vazias

newxyz=cell(1,length(binx)); % cell array contaning the xyz coordinates of each object
                          % each cell corresponds to one object
for obj=1:length(binx)
    newxyz{obj}=xyz(binx{1,obj},:);
end

clear Adj;
clear gdist;

end