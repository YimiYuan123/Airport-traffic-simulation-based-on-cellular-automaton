function meisai
% ������ ����ٶ�3��Ԫ�� ���ڱ߽����� ���� ���� �������
clf
clear all
%build the GUI
%define the plot button
plotbutton=uicontrol('style','pushbutton',...
'string','Run', ...
'fontsize',12, ...
'position',[100,400,50,20], ...
'callback', 'run=1;');
%define the stop button
erasebutton=uicontrol('style','pushbutton',...
'string','Stop', ...
'fontsize',12, ...
'position',[200,400,50,20], ...
'callback','freeze=1;');
%define the Quit button
quitbutton=uicontrol('style','pushbutton',...
'string','Quit', ...
'fontsize',12, ...
'position',[300,400,50,20], ...
'callback','stop=1;close;');
number = uicontrol('style','text', ...
'string','1', ...
'fontsize',12, ...
'position',[20,400,50,20]);
%CA setup
n=500; %���ݳ�ʼ��
z=zeros(1,n); %Ԫ������
z=roadstart(z,20); %��·״̬��ʼ����·��������ֲ�5��
cells=z;
vmax=1; %����ٶ�
v=speedstart(cells,vmax); %�ٶȳ�ʼ��
x=1; %��¼�ٶȺͳ���λ��
memor_cells=zeros(3600,n);
memor_v=zeros(3600,n);
imh=imshow(cells); %��ʼ��ͼ���ɫ�г�����ɫ��Ԫ��
set(imh, 'erasemode', 'none')
axis equal
axis tight

stop=0; %wait for a quit button push
run=0; %wait for a draw
freeze=0; %wait for a freeze�����ᣩ
while (stop==0)
      if(run==1)
          %�߽���������������ĩ�������ƽ�����ʹ�ÿ�������
          a=searchleadcar(cells);
          b=searchlastcar(cells);
          [cells,v]=border_control(cells,a,b,v,vmax);
          i=searchleadcar(cells); %�����׳�λ��
          for j=1:i
               if i-j+1==n
                  [z,v]=leadcarupdate(z,v);
                   continue;
               else 
                   %======================================���١����١��������
                   if cells(i-j+1)==0; %�жϵ�ǰλ���Ƿ�ǿ�
                   continue;
                   else v(i-j+1)=min(v(i-j+1)+1,vmax); %����
                        %=================================����
                        k=searchfrontcar((i-j+1),cells); %����ǰ���׸��ǿ�Ԫ��λ��
                        if k==0; %ȷ����ǰ��֮���Ԫ����
                           d=n-(i-j+1);
                        else d=k-(i-j+1)-1;
                        end
                        v(i-j+1)=min(v(i-j+1),d);
                        %==============================%����
                        %�������
                        v(i-j+1)=randslow(v(i-j+1));
                        new_v=v(i-j+1);                       
                       %======================================���١����١��������
                        %���³���λ��
                       z(i-j+1)=0;                    
                       z(i-j+1+new_v)=1;
                      %�����ٶ�
                      v(i-j+1)=0;
                      v(i-j+1+new_v)=new_v;
                   end
               end
          end
          cells=z;
          memor_cells(x,:)=cells; %��¼�ٶȺͳ���λ��
          memor_v(x,:)=v;
          x=x+1;
          set(imh,'cdata',cells) %����ͼ��
          %update the step number diaplay
          %pause(0.001);
          stepnumber = 1 + str2num(get(number,'string'));
          set(number,'string',num2str(stepnumber))
      end
      if (freeze==1)
         run = 0;
         freeze = 0;
      end
      drawnow
end


function [new_matrix_cells,new_v]=border_control(matrix_cells,a,b,v,vmax)
%�߽����������ڱ߽磬���Ƴ�������
%���ڱ߽磬��ͷ���ڵ�·�߽磬����һ����·0.9��ȥ
n=length(matrix_cells);
if a==n
    rand('state',sum(100*clock)*rand(1));%?��?????��??��?
    p_1=rand(1); %�����������
    if p_1<=1 %����������С��0.9�������뿪·�Σ��������
    matrix_cells(n)=0;
    v(n)=0;    
    end
end
%��ڱ߽磬���ɷֲ����1s��ƽ�����ﳵ����Ϊq��tΪ1s
if b>vmax
    t=1;
    q=0.25;
    x=1;
    p=(q*t)^x*exp(-q*t)/prod(x); %1s����1��������ĸ���
   rand('state',sum(100*clock)*rand(1));
    p_2=rand(1);
    if p_2<=p 
       m=min(b-vmax,vmax);
       matrix_cells(m)=1;
       v(m)=m;    
    end
end
new_matrix_cells=matrix_cells;
new_v=v;     


function [new_matrix_cells,new_v]=leadcarupdate(matrix_cells,v)
%��һ�������¹���
n=length(matrix_cells);
if v(n)~=0
   matrix_cells(n)=0;
   v(n)=0;
end
new_matrix_cells=matrix_cells;
new_v=v;


function [new_v]=randslow(v)
p=0.3; %��������
rand('state',sum(100*clock)*rand(1));%?��?????��??��?
p_rand=rand; %�����������
if p_rand<=p
   v=max(v-1,0);
end
new_v=v;   


function [matrix_cells_start]=roadstart(matrix_cells,n)
%��·�ϵĳ�����ʼ��״̬��Ԫ���������Ϊ0��1��matrix_cells��ʼ����n��ʼ������
k=length(matrix_cells);
z=round(k*rand(1,n));
for i=1:n
    j=z(i);
    if j==0 
       matrix_cells(j)=0;
    else
       matrix_cells(j)=1;
    end
end
matrix_cells_start=matrix_cells;

function  [location_frontcar]=searchfrontcar(current_location,matrix_cells)
i=length(matrix_cells);
if current_location==i
   location_frontcar=0;
else
    for j=current_location+1:i
       if matrix_cells(j)~=0
          location_frontcar=j;
       break;
       else
          location_frontcar=0;
       end
    end
end

function [location_lastcar]=searchlastcar(matrix_cells)
%����β��λ��
for i=1:length(matrix_cells)
    if matrix_cells(i)~=0
       location_lastcar=i;
       break;
    else %���·���޳������Ԫ�����趨Ϊ��·����
       location_lastcar=length(matrix_cells);
    end
end

function [location_leadcar]=searchleadcar(matrix_cells)
i=length(matrix_cells);
for j=1:i
    if matrix_cells(i-j+1)~=0
       location_leadcar=i-j+1;
       break;
    else
       location_leadcar=0;
    end
end


function [v_matixcells]=speedstart(matrix_cells,vmax)
%��·��ʼ״̬�����ٶȳ�ʼ��
v_matixcells=zeros(1,length(matrix_cells));
for i=1:length(matrix_cells)
    if matrix_cells(i)~=0
       v_matixcells(i)=round(vmax*rand(1));
    end
end



for i=1:900
    for j=1:100
        if memor_cells(i,j)>0
          plot(j,i,'k.','markersize',5);
          %text(j,i,num2str(memor_v(i,j)),'FontSize',9);
          hold on
          
        end
    end
end
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
