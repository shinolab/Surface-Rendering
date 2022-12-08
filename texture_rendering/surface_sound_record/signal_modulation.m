%幅度调制
fc=108;%载波频率
fm=1.8;%信号频率
fs=44100;%采样频率
m=1.5;
ts=1/fs;
N=211611;
t=(0:1:N-1)*ts;
Carrier=cos(2*pi*fc*t);
Signal=sin(2*pi*fm*t);
y=(2+m*Signal).*Carrier;

y = 2 * (normalize(y, 'range') - 0.5);
%画图
subplot(3,1,1);plot(t,Carrier);ylabel('载波信号');xlabel('时间t');
subplot(3,1,2);plot(t,Signal);ylabel('输入信号');xlabel('时间t');
subplot(3,1,3);plot(t,y);ylabel('调幅信号');xlabel('时间t');

audiowrite('modulation.wav', y, fs);
audiowrite('plus.wav', 0.5 * (Carrier + Signal), fs);
sound(y, fs)

