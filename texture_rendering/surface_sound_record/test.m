[x, Fs] = audioread('preprocessed\wallpaper.wav');
x = x(13345:131433); % 选出周期性明显的片段

%% 定义时域采样信号 x
Ts=1/Fs;            % 采样时间间隔
N = size(x);
N = N(1);           % 采样信号的长度
t=(0:1:N-1)*Ts;     % 定义信号采样的时间点 t
t=t';               % 为了方便查看, 将行向量 t 转置成列向量

x_SA_1 = lowpass(x, 30, Fs);  % Slow-adapting type I Merkel
x_SA_2 = lowpass(x, 15, Fs);  % Slow-adapting type II Ruffini
x_FA_1 = bandpass(x, [10, 60], Fs);  % Fast-adapting type I Meissner
x_FA_2 = bandpass(x, [50, 1000], Fs);  % Fast-adapting type II Pacini