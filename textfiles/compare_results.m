%comparing results
clear;
clc;

%user inputs
ws_folder(1) = "1 8ft Still";
ws_folder(2) = "2 8ft Pivot";
ws_folder(3) = "3 8ft MB";
ws_folder(4) = "4 8ft Twirl";
ws_folder(5) = "5 3ft Still";
ws_folder(6) = "6 3ft Pivot";
ws_folder(7) = "7 3ft MB";
ws_folder(8) = "8 3ft Twirl";

s = size(ws_folder);
s = s(2);

for i=1:1:s
    
    ws = load(strcat(ws_folder(i), "/ws.mat"));
    mean_dis_error(i) = ws.mean_dis_error;
    mean_error(i) = ws.mean_error;
    per_mean_error(i) = ws.per_mean_error;
    max_error(i) = ws.max_error;
    per_max_error(i) = ws.per_max_error;
    per_fixed(i) = ws.per_fixed;
end

mean_dis_error = mean_dis_error';
mean_error = mean_error';
per_mean_error = per_mean_error';
max_error = max_error';
per_max_error = per_max_error';
per_fixed = per_fixed';
test = ws_folder';

T = table(test, mean_dis_error, mean_error, per_mean_error, max_error, per_max_error, per_fixed);
save('tabular_results.mat', 'T');


