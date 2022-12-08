wav_dir = {dir('raw/*.wav').name};
file_num = size(wav_dir);
len_remove_outliers = [];
all_wave = [];

for filename = wav_dir
    [data, fs] = audioread(['raw/', char(filename)]);
    data = bandpass(data, [100, 300], fs);
    remove_outliers = filloutliers(data, 'nearest', 'mean');
    s = size(remove_outliers);
    len_remove_outliers = [len_remove_outliers, s(1)];
    all_wave = [all_wave, remove_outliers'];
end

all_wave = 2 * (normalize(all_wave, 'range') - 0.5);
plot(all_wave)

for i = 1:file_num(2)
    filename = char(wav_dir(i));
    data_len = len_remove_outliers(i);
    data = all_wave(1:data_len);
    all_wave(1:data_len) = [];
    audiowrite(['preprocessed/', filename], data, fs);
end

