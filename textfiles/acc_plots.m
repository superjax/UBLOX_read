%RTK relative position plot generator
clear;
clc;

%user inputs
folder(1) = "1 8ft Still";
folder(2) = "2 8ft Pivot";
folder(3) = "3 8ft MB";
folder(4) = "4 8ft Twirl";
Truth(1) = 2.36; %meters
folder(5) = "5 3ft Still";
folder(6) = "6 3ft Pivot";
folder(7) = "7 3ft MB";
folder(8) = "8 3ft Twirl";
Truth(2) = .99; %meters
folder_size = size(folder);
folder_size = folder_size(2);

for i=1:1:folder_size
    if i<5
        calc(folder(i), Truth(1));
    else
        calc(folder(i), Truth(2));
    end
end

function calc(folder, Truth)
    in_file = strcat(folder, '/data.txt');
    out_file = strcat(folder, '/ws');
    fig_file_mag = strcat(folder, '/mag.fig');
    fig_file_ned = strcat(folder, '/ned.fig');

    data = fileread(in_file);

    s = size(data);
    s = s(2);
    t = 1;
    N = 2;
    E = 3;
    D = 4;
    L = 5;
    F = 6;

    var = t;

    j = 1;
    k = 1;
    p = 0;

    for i = 1:1:s
       if data(i) == ' '
           if var == F
               [var, j, k] = swap(t, N, E, D, L, F, var, j, k);
               p = 1;
           else
               [var, j, k] = swap(t, N, E, D, L, F, var, j, k);
           end
       elseif p == 1
           p = 0;
       else
           switch var
               case t
                   time(k,j) = data(i);
               case N
                   north(k,j) = data(i);
               case E
                   east(k,j) = data(i);
               case D
                   down(k,j) = data(i);
               case L
                   length(k,j) = data(i);
               case F
                   flag(k,j) = data(i);
           end
           j = j+1;
       end

    end

    format long;
    Time = str2num(time);
    North = twos_comp(str2num(north));
    East = twos_comp(str2num(east));
    Down = twos_comp(str2num(down));
    Length = str2num(length);
    Flag = str2num(flag);

    Length_calc = sqrt(North.^2+East.^2+Down.^2);
    Length_calc_error = Length-Length_calc;

    Time = Time-Time(1);
    time_size = size(Time);
    time_size = time_size(1);

    flag_size = size(Flag);
    flag_size = flag_size(1);
    num = 0;
    z = 1;
    y = 1;
    for i = 1:1:flag_size
        if Flag(i) == 179 || Flag(i) == 311 || Flag(i) == 243 || Flag(i) == 375
            Fixed_time(z) = Time(i);
            Fixed_length(z) = Length(i);
            num = num + 1;
            z = z+1;
        else
            Float_time(y) = Time(i);
            Float_length(y) = Length(i);
            y = y+1;
            Fixed(i) = 0;
        end
    end
    per_fixed = num/flag_size*100;

    %calculate accuracy
    mean_dis = mean(Length(2:size(Length))); %doesn't include the first point which is 0
    mean_dis_error = mean_dis-Truth;
    mean_error = mean(abs(Length(2:size(Length))-Truth));
    per_mean_error = 100*(mean_error)/Truth

    %calculate max error
    error = Length-Truth;
    max_error = max(error(2:size(Length))) %doesn't include the first point which is 0
    per_max_error = 100*max_error/Truth

    figure(1);
    plot([Time(1),Time(time_size)], [Truth,Truth]);
    hold on;
    plot(Fixed_time ,Fixed_length, 'o b');
    plot(Float_time,Float_length, '^ r');
    legend('Truth', 'Fixed', 'Floating');
    ylabel('distance (m)');
    xlabel('time (s)');
    title(strcat('Time vs. Relative Distance Magnitude ', folder));
    savefig(fig_file_mag);
    close;


    figure(2);
    plot(Time,North);
    hold on;
    plot(Time,East);
    plot(Time,Down);
    legend('North','East','Down');
    ylabel('distance (m)');
    xlabel('time (s)');
    title(strcat('Time vs. Relative Distances North East Down ', folder));
    savefig(fig_file_ned);
    close;

    T = table(mean_error, per_mean_error, max_error, per_max_error, per_fixed)

    save(out_file);

end


function fixed = twos_comp(Vec)
    vec_size = size(Vec);
    for i=1:1:vec_size
        if Vec(i) > 1000%2147483647
            j = 0;
            binary = zeros(1,32);
            D = Vec(i);
            Vec(i) = 0;
            for k = 32:-1:1
                binary(k) = mod(D,2);
                D = (D-binary(k))/2;
                if binary(k) == 1
                    binary(k) = 0;
                else
                    binary(k) = 1;
                end
                Vec(i) = Vec(i) + binary(k)*2^(j);
                j = j+1;
            end
            Vec(i) = -(Vec(i)+1);
        end

    fixed = Vec;
    end
end


function [var, j, k] = swap(t, N, E, D, L, F, var, j, k)
    switch var
       case t
           var = N;
           j = 1;
       case N
           var = E;
           j = 1;
       case E
           var = D;
           j = 1;
       case D
           var = L;
           j = 1;
       case L
           var = F;
           j = 1;
       case F 
           var = t;
           k = k+1;
           j = 1;
   end
end