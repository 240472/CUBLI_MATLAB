clear all;
close all;
clc;

load("acc_calibration.mat")

time = data{1}.Values.Time;

for j = 1:data.numElements
    switch data{j}.Name
        case "ICM_raw_acc_data [m/s^2]"
            acc_raw_vector = data{j}.Values.Data(:,:);   
    end
end

plot(acc_raw_vector)
legend



true_g = 9.81275;

acc_g00     = mean(acc_raw_vector(1830:3600,:));
acc_0g0     = mean(acc_raw_vector(5577:7309,:));
acc_00g     = mean(acc_raw_vector(22824:26243,:));
acc_neg_g00 = mean(acc_raw_vector(8268:9961,:));
acc_neg_0g0 = mean(acc_raw_vector(13112:15918,:));
acc_neg_00g = mean(acc_raw_vector(17810:19847,:));

% Referenční matice (3x6) - ideální hodnoty
A_ref = true_g * [ 1  0   0   -1   0   0;
                   0   1   0  0   -1   0;
                   0   0   1   0   0  -1];

% Naměřená data (3x6) - každý sloupec je jedna poloha
A_raw = [acc_g00; acc_0g0; acc_00g; acc_neg_g00; acc_neg_0g0; acc_neg_00g]';

% Sestavení soustavy a výpočet M a b
D = [A_raw', -ones(6,1)]; % 6x4

M = zeros(3,3);
c = zeros(3,1);

for j = 1:3
    rhs = A_ref(j,:)';
    params = D \ rhs;
    M(j,:) = params(1:3)';
    c(j)   = params(4);
end

% Zpětný výpočet biasu
b = M \ c;

disp('Kalibrační matice M:'); disp(M)
disp('Bias b [m/s^2]:'); disp(b)

% Ověření - residua by měla být blízko nule
a_cal = M * A_raw - c;
disp('Residua [m/s^2]:'); disp(A_ref - a_cal)