function [assign,unassignedTracks,unassignedDetections] = Tracking(cost_matrix, cost)


% with hungarian
ncols=length(cost_matrix(1,:));
nrows=length(cost_matrix(:,1));
unassignedDetections=[];
unassignedTracks=[];
assign=[];
unassignedTracks_h=[];
unassignedDetections_h=[];

for i=1:ncols
    idx=[];
    idx=find(cost_matrix(:,i)<cost);
    if length(idx)==nrows %new object
        unassignedDetections = [unassignedDetections  i];
    end
end

for j=1:nrows
    idx=[];
    idx=find(cost_matrix(j,:)<cost);
    if length(idx)==ncols %object that disappear
        unassignedTracks = [unassignedTracks  j];
    end
end

for k=1:length(unassignedDetections)
    cost_matrix(:,unassignedDetections(k))=cost_matrix(:,unassignedDetections(k))*0;
end

for k=1:length(unassignedTracks)
    cost_matrix(unassignedTracks(k),:)=cost_matrix(unassignedTracks(k),:)*0;
end


for i=1:ncols
    for j=1:nrows
        if cost_matrix(j,i)==0
            cost_matrix(j,i)= Inf;
        else
            cost_matrix(j,i)=1-cost_matrix(j,i);
            cost_matrix(j,i);
        end
    end
end

[assign, unassignedTracks_h, unassignedDetections_h]=assignDetectionsToTracks(cost_matrix, 1-cost);




unassignedTracks= [unassignedTracks unassignedTracks_h'];
unassignedTracks=unique(unassignedTracks);
unassignedDetections=[unassignedDetections unassignedDetections_h'];
unassignedDetections=unique(unassignedDetections);

end


%%%%%%%%%%%%%%
% ncols=length(cost_matrix(1,:));
% nrows=length(cost_matrix(:,1));
% unassignedDetections=[];
% unassignedTracks=[];
% assignments=[];
% k=1;
% 
% assign_new=[];
% assign_previous=[];
% costvalue=[];
% 
% for i=1:ncols
%     idx=[];
%     idx=find(cost_matrix(:,i)<cost);
%     if length(idx)==nrows %new object
%         unassignedDetections = [unassignedDetections  i];
%     else
%         assign_new(k)=i;
%         [costvalue(k),assign_previous(k)]=max(cost_matrix(:,i));
%         k=k+1;
%     end
%     
% end
% 
% m=1;
% while length(costvalue)>=1
%         [~,max_ind]=max(costvalue);
%         if(length(assignments)>0)
%             if(length(find(assignments(:,1)==assign_previous(max_ind)))==0)            
%                 assignments(m,1)=assign_previous(max_ind);  
%                 assignments(m,2)=assign_new(max_ind);
%                 cost_matrix(assign_previous(max_ind),:)=cost_matrix(assign_previous(max_ind),:)*0;
%                 costvalue(max_ind)=[];
%                 assign_previous(max_ind)=[];
%                 assign_new(max_ind)=[];
%                 m=m+1;
%             else
%                 [newmax,newprevious]=max(cost_matrix(:,assign_new(max_ind)));
%                 if length(newmax)>0
%                     if(newmax>=cost) 
%                         costvalue(max_ind)=newmax;
%                         assign_previous(max_ind)=newprevious;
%                     else
%                         unassignedDetections = [unassignedDetections  assign_new(max_ind)];
%                 costvalue(max_ind)=[];
%                 assign_previous(max_ind)=[];
%                 assign_new(max_ind)=[];
%                     end
%                 else
%                     unassignedDetections = [unassignedDetections  assign_new(max_ind)];
%                 costvalue(max_ind)=[];
%                 assign_previous(max_ind)=[];
%                 assign_new(max_ind)=[];
% 
%                 end
%             end
%         else
%                 assignments(m,1)=assign_previous(max_ind);  
%                 assignments(m,2)=assign_new(max_ind);
%                 cost_matrix(assign_previous(max_ind),:)=cost_matrix(assign_previous(max_ind),:)*0;
%                 costvalue(max_ind)=[];
%                 assign_previous(max_ind)=[];
%                 assign_new(max_ind)=[];
%                 m=m+1;
%         end
% end
%         
% 
% 
% 
% % 
% % m=1
% % while m<=length(assign_previous)
% %     repeated_id=find(assign_previous==assign_previous(m));
% %     if length(repeated_id)==1
% %         assignments(m,1)=assign_previous(m);  
% %         assignments(m,2)=assign_new(m);
% %         disp('aqui')
% %         assignments
% %     else
% %         [~,max_id]=max(costvalue(repeated_id)); 
% %         assignments(m,1)=assign_previous(repeated_id(max_id));  
% %         assignments(m,2)=assign_new(repeated_id(max_id));
% %         repeated_id(max_id)=[]; 
% %         
% %         %tirar os outros
% %         unassignedDetections = [unassignedDetections  assign_new(repeated_id)];
% %         unassignedDetections 
% %         assign_previous(repeated_id)=[];
% %        % length_aux=length(assign_previous);
% %     end
% %     m=m+1;
% % end
% 
% for j=1:nrows
%     idx=[];
%     idx=find(cost_matrix(j,:)<cost);
%     if length(idx)==ncols %object that disappear
%         unassignedTracks = [unassignedTracks  j];
%     end
% end
% 

