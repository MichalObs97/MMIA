filename = 'ntc.csv';
M = csvread(filename);
R = M(:,2);
t = M(:,1);

ADCvalue = 1023.*(R./(R+10))

plot(ADCvalue,t);
p=polyfit(ADCvalue,t,10);

ad2=0:1023;
t2 = round(polyval(p,ad2),1);

hold on
plot(ad2,t2,'r');

hold on
dlmwrite('data.dlm', t2*10,',');






