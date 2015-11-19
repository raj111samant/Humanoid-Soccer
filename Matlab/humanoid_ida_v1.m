%matlab code`
close all 
clear all 
clc


global xg yg xr yr   %robot and goal please provide
global output cost test 
global trnd
global arrey counter
counter = 0;

arrey = [10,    0;
             10,    3;
               8,    6;
               6,    8;
              3,   10;
              0,   10;
             -3,   10;
              -6,    8;
              -8,    6;
            -10,    3;
            -10,    0;
            -10,   -3;
              -8,   -6;
              -6,   -8;
             -3,  -10;
              0,  -10;
              3,  -10;
              6,   -8;
               8,   -6;
            10,   -3];
           
         
cost = 1;

trnd = 1;
 
test1 = imread('obstacle.jpg');
test2 = rgb2gray(test1);
level = graythresh(test2);
test = im2bw(test2,level);

output=zeros(480,640);
xr = 2 ;%put('enter robots’s x ');
yr = 2; % input('enter robots’s y ');
xg = 40; %input('enter goal’s x ');
yg = 10; % input('enter goal’s y ');
figure(1)
imshow(test)
figure(2)
fans =  ida(xr,yr);

if fans == inf
    disp('FAIL');
elseif fans == 10000
    disp('GOT IT')
end

quantize(yr,xr,20);

imshow(output);