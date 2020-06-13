global A = csvread('csv_matter.csv');  #do not change this line
global y=zeros(8002,6);
global k=2;
global gx_old=0.0;
global gy_old=0.0;
global gz_old=0.0;
global pitch=0;
global roll=0;
################################################
#######Declare your global variables here#######
################################################
function read_accel(axl,axh,ayl,ayh,azl,azh)  
  #################################################
  ####### Write a code here to combine the ########
  #### HIGH and LOW values from ACCELEROMETER #####
  #################################################
  ax=double((typecast(uint8([axl,axh]),"int16")))/16384;
  ay=double((typecast(uint8([ayl,ayh]),"int16")))/16384;
  az=double((typecast(uint8([azl,azh]),"int16")))/16384;
  ####################################################
  # Call function lowpassfilter(ax,ay,az,f_cut) here #
  ####################################################
  lowpassfilter(ax,ay,az,5);
endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  #################################################
  ####### Write a code here to combine the ########
  ###### HIGH and LOW values from GYROSCOPE #######
  #################################################
  gx=double((typecast(uint8([gxl,gxh]),"int16")))/131;
  gy=double((typecast(uint8([gyl,gyh]),"int16")))/131;
  gz=double((typecast(uint8([gzl,gzh]),"int16")))/131;

  #####################################################
  # Call function highpassfilter(ax,ay,az,f_cut) here #
  #####################################################;
  highpassfilter(gx,gy,gz,5);
endfunction



function lowpassfilter(ax,ay,az,f_cut)
  global y;
  global k;
  dT =0.01 ;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
  y(k,1)=(1-alpha)*ax +  alpha*y(k-1,1);
  y(k,2)=(1-alpha)*ay +  alpha*y(k-1,2);
  y(k,3)=(1-alpha)*az +  alpha*y(k-1,3);
endfunction



function highpassfilter(gx,gy,gz,f_cut)
  global y;
  global gx_old;
  global gy_old;
  global gz_old;
  global k;
  dT =0.01 ;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
  y(k,4)=(1-alpha)*y(k-1,4) + (1-alpha)*(gx-gx_old);
  gx_old=gx;
  y(k,5)=(1-alpha)*y(k-1,5) + (1-alpha)*(gy-gy_old);
  gy_old=gy;
  y(k,6)=(1-alpha)*y(k-1,6) + (1-alpha)*(gz-gz_old);
  gz_old=gz;
endfunction

function comp_filter_pitch(ax,ay,az,gx,gy,gz)
  global pitch;
  alpha=0.03;
  gyro_angle=0;
  ##############################################
  ####### Write a code here to calculate  ######
  ####### PITCH using complementry filter ######
  ##############################################
  acc = (atand(ay / abs(az)));
  gyro_angle=gyro_angle+ gx *0.01;
  pitch= (1-alpha)*(pitch-gyro_angle) + alpha*acc;
endfunction 

function comp_filter_roll(ax,ay,az,gx,gy,gz)
  global roll;
  alpha=0.03;
  gyro_angle=0;
  ##############################################
  ####### Write a code here to calculate #######
  ####### ROLL using complementry filter #######
  ##############################################
  acc = (atand(ax / abs(az)));
  gyro_angle=gyro_angle+ gy *0.01;
  roll= (1-alpha)*(roll-gyro_angle) + alpha*acc;
endfunction 

function execute_code
  
  global A;
  global k;
  global y;
  global pitch;
  global roll;
  B=[];
  for n = 1:rows(A)                    #do not change this line
    
    ###############################################
    ####### Write a code here to calculate  #######
    ####### PITCH using complementry filter #######
    ###############################################
    read_accel(A(n,2),A(n,1),A(n,4),A(n,3),A(n,6),A(n,5));
    read_gyro(A(n,8),A(n,7),A(n,10),A(n,9),A(n,12),A(n,11));
    comp_filter_pitch(y(n+1,1),y(n+1,2),y(n+1,3),y(n+1,4),y(n+1,5),y(n+1,6));
    B(n,1)=pitch;
    comp_filter_roll(y(n+1,1),y(n+1,2),y(n+1,3),y(n+1,4),y(n+1,5),y(n+1,6));
    B(n,2)=roll;
    k=k+1;
  endfor
  csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line
