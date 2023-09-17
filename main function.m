% ������  ���ڱ߽����� ���� ���� �������
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
carnumb1=0;
carnumb2=0;
%��1����
n=100; %���ݳ�ʼ��
z1=zeros(1,n); %Ԫ������
z1=roadstart(z1,0); %��·״̬��ʼ
cells1=z1;
vmax=3; %����ٶ�
v1=speedstart(cells1,vmax); %�ٶȳ�ʼ��
x=1; %��¼�ٶȺͳ���λ��
tick1=0;%ʱ�䲽
psg1=zeros(1,n);%���ϳ˿�״̬����

%��2����
n=100; %���ݳ�ʼ��
z2=zeros(1,n); %Ԫ������
z2=roadstart(z2,0); %��·״̬��ʼ��
cells2=z2;
vmax=3; %����ٶ�
v2=speedstart(cells2,vmax); %�ٶȳ�ʼ��
tick2=0;%ʱ�䲽
psg2=zeros(1,n);
memor_cells1=zeros(3600,n);
memor_v1=zeros(3600,n);
memor_cells2=zeros(3600,n);
memor_v2=zeros(3600,n);

%��1�˿Ͷ���
n2=100; %���ݳ�ʼ��
z3=zeros(1,n2); %Ԫ������
z3=roadstart(z3,0); %��·״̬��ʼ����·��������ֲ�10��
cells3=z3;
vmax3=1; %����ٶ�
v3=speedstart(cells3,vmax3); %�ٶȳ�ʼ��
tick3=0;%ʱ�䲽
%��2�˿Ͷ���
 %���ݳ�ʼ��
z4=z3;

cells4=z3;
vmax4=1; %����ٶ�
v4=speedstart(cells4,vmax3); %�ٶȳ�ʼ��
tick4=0;%ʱ�䲽
memor_cells3=zeros(3600,n2);
memor_v3=zeros(3600,n2);
memor_cells4=zeros(3600,n2);
memor_v4=zeros(3600,n2);


%��ť״̬
stop=0; %wait for a quit button push
run=0; %wait for a draw
freeze=0; %wait for a freeze�����ᣩstop=0; %wait for a quit button push
run=0; %wait for a draw
freeze=0; %wait for a freeze�����ᣩ
t=0;
while (stop==0 &&x<10000)
    t=t+1;
      if(run==0)
          a1=searchleadcar(cells1);
          b1=searchlastcar(cells1);
          a2=searchleadcar(cells2);
          b2=searchlastcar(cells2);
          a3=searchleadcar(cells3);
          b3=searchlastcar(cells3);
          a4=searchleadcar(cells4);
          b4=searchlastcar(cells4);
          [cells3,cells1,v3,v1,carnumb1,tick3]=border_control2_2(cells3,cells1,a3,b3,a1,b1,v3,v1,vmax,vmax3,carnumb1,tick3,psg1);
          [cells4,cells2,v4,v2,carnumb2,tick4]=border_control2_2(cells4,cells2,a4,b4,a2,b2,v4,v2,vmax,vmax4,carnumb2,tick4,psg2);

           i11=searchleadcar(cells1(1:50)); %�����׳�λ��
           i21=searchleadcar(cells2(1:50));
           i3=searchleadcar(cells3);
           i4=searchleadcar(cells4);
           i12=searchleadcar(cells4(51:100));
           i22=searchleadcar(cells4(51:100));
          
           %����1
           %�ϳ���ǰ
            for j=1:i11
               if i11-j+1==50
                  [z1(:,1:50),v1(:,1:50)]=leadcarupdate(z1(:,1:50),v1(:,1:50));
                   continue;
               else 
                   %======================================���١����١��������
                   if cells1(i11-j+1)==0; %�жϵ�ǰλ���Ƿ�ǿ�
                   continue;
                   else v1(i11-j+1)=min(v1(i11-j+1)+1,vmax); %����
                        %=================================����
                        k=searchfrontcar((i11-j+1),cells1); %����ǰ���׸��ǿ�Ԫ��λ��
                        if k==0; %ȷ����ǰ��֮���Ԫ����
                           d=50-(i11-j+1);
                        else d=k-(i11-j+1)-1 ;
                        end
                        v1(i11-j+1)=min(v1(i11-j+1),d);
                        %==============================%����
                        %�������
                        v1(i11-j+1)=randslow(v1(i11-j+1));
                        new_v1=v1(i11-j+1);                       
                       %======================================���١����١��������
                        %���³���λ��
                        if i11-j+1<50
                       z1(i11-j+1)=0;                    
                       z1(i11-j+1+min(new_v1,50-(i11-j+1)))=1;
                        end
                      %�����ٶ�
                      
                      v1(i11-j+1)=0;
                      if 50-(i11-j+1)<new_v1
                      v1(i11-j+1+new_v1)=new_v1;
                      elseif 50-(i11-j+1)>=new_v1
                          v1(50)=0;
                      end
                      if 50-(i11-j+1)<new_v1 && psg1(i12-j+1)==1
                      psg1(i11-j+1+new_v1)=1;
                      psg1(i11-j+1)=0;
                       elseif psg1(i11-j+1)==1 && 50-(i11-j+1)<new_v1
                      psg1(i11-j+1)=0;
                      psg1(i11-j+1+new_v1)=1;
                      end
                   end
               end
            end
            %�ϳ����
             for j=51:i12
               if i12-j+1==100
                  [z1(:,51:100),v1(:,51:100)]=leadcarupdate(z1(:,51:100),v1(:,51:100));
                   continue;
               else 
                   %======================================���١����١��������
                   if cells1(i12-j+1)==0; %�жϵ�ǰλ���Ƿ�ǿ�
                   continue;
                   else v1(i12-j+1)=min(v1(i12-j+1)+1,vmax); %����
                        %=================================����
                        k=searchfrontcar((i12-j+1),cells1); %����ǰ���׸��ǿ�Ԫ��λ��
                        if k==0; %ȷ����ǰ��֮���Ԫ����
                           d=100-(i12-j+1);
                        else d=k-(i12-j+1)-1 ;
                        end
                        v1(i12-j+1)=min(v1(i12-j+1),d);
                        %==============================%����
                        %�������
                        v1(i12-j+1)=randslow(v1(i12-j+1));
                        new_v1=v1(i12-j+1);                       
                       %======================================���١����١��������
                        %���³���λ��
                        if i12-j+1<100
                       z1(i12-j+1)=0;                    
                       z1(i12-j+1+new_v1)=1;
                        end
                      %�����ٶ�
                      
                      v1(i12-j+1)=0;
                      v1(i12-j+1+new_v1)=new_v1;
                      %���³˿�״̬
                      if psg1(i12-j+1)==1
                      psg1(i12-j+1)=0;
                      psg1(i12-j+1+new_v1)=1;
                      end
                   end
               end
            end
            
            
        %�ϳ���ǰ    
            
            for j=1:i21
               if i21-j+1==50
                  [z2(:,1:50),v2(:,1:50)]=leadcarupdate(z2(:,1:50),v2(:,1:50));
                   continue;
               else 
                   %======================================���١����١��������
                   if cells2(i21-j+1)==0; %�жϵ�ǰλ���Ƿ�ǿ�
                   continue;
                   else v2(i21-j+1)=min(v2(i21-j+1)+1,vmax); %����
                        %=================================����
                        k=searchfrontcar((i21-j+1),cells2); %����ǰ���׸��ǿ�Ԫ��λ��
                        if k==0; %ȷ����ǰ��֮���Ԫ����
                           d=50-(i21-j+1);
                        else d=k-(i21-j+1)-1 ;
                        end
                        v2(i21-j+1)=min(v2(i21-j+1),d);
                        %==============================%����
                        %�������
                        v2(i21-j+1)=randslow(v2(i21-j+1));
                        new_v2=v2(i21-j+1);                       
                       %======================================���١����١��������
                        %���³���λ��
                        if i21-j+1<50
                       z1(i21-j+1)=0;                    
                       z1(i21-j+1+min(new_v2,50-(i21-j+1)))=1;
                        end
                      %�����ٶ�
                      
                      v2(i21-j+1)=0;
                      if 50-(i21-j+1)<new_v2
                      v2(i21-j+1+new_v2)=new_v2;
                      elseif 50-(i21-j+1)>=new_v2
                          v2(50)=0;
                      end
                      if 50-(i21-j+1)<new_v2 && psg2(i21-j+1)==1
                      psg2(i21-j+1+new_v2)=1;
                      psg2(i21-j+1)=0;
                       elseif psg2(i21-j+1)==1 && 50-(i21-j+1)<new_v2
                      psg2(i21-j+1)=0;
                      psg2(i21-j+1+new_v2)=1;
                      end
                   end
               end
            end
            %�ϳ����
             for j=51:i22
               if i22-j+1==50
                  [z2(:,51:100),v2(:,51:100)]=leadcarupdate(z2(:,51:100),v2(:,51:100));
                   continue;
               else 
                   %======================================���١����١��������
                   if cells2(i22-j+1)==0; %�жϵ�ǰλ���Ƿ�ǿ�
                   continue;
                   else v2(i22-j+1)=min(v2(i22-j+1)+1,vmax); %����
                        %=================================����
                        k=searchfrontcar((i22-j+1),cells2); %����ǰ���׸��ǿ�Ԫ��λ��
                        if k==0; %ȷ����ǰ��֮���Ԫ����
                           d=50-(i22-j+1);
                        else d=k-(i22-j+1)-1 ;
                        end
                        v2(i22-j+1)=min(v2(i22-j+1),d);
                        %==============================%����
                        %�������
                        v2(i22-j+1)=randslow(v2(i22-j+1));
                        new_v2=v2(i22-j+1);                       
                       %======================================���١����١��������
                        %���³���λ��
                        if i22-j+1<100
                       z1(i22-j+1)=0;                    
                       z1(i22-j+1+new_v2)=1;
                        end
                      %�����ٶ�
                      
                      v2(i22-j+1)=0;
                      v2(i22-j+1+new_v2)=new_v2;
                      %���³˿�״̬
                      if psg2(i22-j+1)==1
                      psg2(i22-j+1)=0;
                      psg2(i22-j+1+new_v2)=1;
                      end
                   end
               end
            end
          
            %����
          for j=1:i3
               if i3-j+1==n2
                  [z3,v3]=leadcarupdate(z3,v3);
                   continue;
               else 
                   %======================================���١����١��������
                   if cells3(i3-j+1)==0; %�жϵ�ǰλ���Ƿ�ǿ�
                   continue;
                   else v3(i3-j+1)=min(v3(i3-j+1)+1,vmax3); %����
                        %=================================����
                        k=searchfrontcar((i3-j+1),cells3); %����ǰ���׸��ǿ�Ԫ��λ��
                        if k==0; %ȷ����ǰ��֮���Ԫ����
                           d=n2-(i3-j+1);
                        else d=k-(i3-j+1)-1  ;
                             
                        end
                        v3(i3-j+1)=min(v3(i3-j+1),d);
                        %==============================%����
                        %�������
                        %v(i-j+1)=randslow(v(i-j+1));
                        new_v3=v3(i3-j+1);                       
                       %======================================���١����١��������
                        %���³���λ��
                        if i3-j+1<n2
                       z3(i3-j+1)=0;                    
                       z3(i3-j+1+new_v3)=1;
                        end
                      %�����ٶ�
                      v3(i3-j+1)=0;
                      v3(i3-j+1+new_v3)=new_v3;
                   end
                end
           end
          %�ڶ�������
          for j=1:i4
               if i4-j+1==n2
                  [z4,v4]=leadcarupdate(z4,v4);
                   continue;
               else 
                   %======================================���١����١��������
                   if cells4(i4-j+1)==0; %�жϵ�ǰλ���Ƿ�ǿ�
                   continue;
                   else v4(i4-j+1)=min(v4(i4-j+1)+1,vmax4); %����
                        %=================================����
                        k=searchfrontcar((i4-j+1),cells4); %����ǰ���׸��ǿ�Ԫ��λ��
                        if k==0; %ȷ����ǰ��֮���Ԫ����
                           d=n2-(i4-j+1);
                        else d=k-(i4-j+1)-1  ;
                             
                        end
                        v4(i4-j+1)=min(v4(i4-j+1),d);
                        %==============================%����
                        %�������
                        %v(i-j+1)=randslow(v(i-j+1));
                        new_v4=v4(i4-j+1);                       
                       %======================================���١����١��������
                        %���³���λ��
                        if i4-j+1<n2
                       z4(i4-j+1)=0;                    
                       z4(i4-j+1+new_v4)=1;
                        end
                      %�����ٶ�
                      v4(i4-j+1)=0;
                      v4(i4-j+1+new_v4)=new_v4;
                   end
               end
          end
              
          cells1=z1;
          cells2=z2;
          cells3=z3;
          cells4=z4;
          memor_cells1(x,:)=cells1; %��¼�ٶȺͳ���λ��
          memor_v1(x,:)=v1;
          memor_cells2(x,:)=cells2; %��¼�ٶȺͳ���λ��
          memor_v2(x,:)=v2;
          memor_cells3(x,:)=cells3; %��¼�ٶȺͳ���λ��
          memor_v3(x,:)=v3;
          memor_cells4(x,:)=cells4; %��¼�ٶȺͳ���λ��
          memor_v3(x,:)=v3;
          x=x+1;

          %update the step number diaplay
          pause(0.001);
          stepnumber = 1 + str2num(get(number,'string'));
          set(number,'string',num2str(stepnumber))
      end
      if (freeze==1)
         run = 0;
         freeze = 0;
      end
    
      if stepnumber>=3600
        %  break;
      end
end
























































