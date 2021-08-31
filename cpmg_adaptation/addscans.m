clear;

pathName = pwd;
fileList = dir(fullfile(pathName, 'cpmg-*')); %add name of files, cpmg is cpmg-*
a=length(readmatrix(fileList(1).name));
Itotal = int32(zeros(a,1));
Qtotal = int32(zeros(a,1));
b = zeros(a,1);
ztotal = complex(b,0);


for k = 1:numel(fileList)
    s = readmatrix(fileList(k).name);
    s = int32(s); %turn characters to numbers
    I = s(:,1); %takes first column of matrix
    Q = s(:,2); %takes second column of matrix
  
    
    
    Itotal = Itotal + I;
    Qtotal = Qtotal + Q;
    
end

z = complex(Itotal,Qtotal);
time = 0:1/20000:(length(I)-1)/20000;
time = time';

AmplitudeIQ = double(Itotal.*Itotal+Qtotal.*Qtotal).^0.5;



Fs = 20000;           % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(I);        % Length of signal
t = (0:L-1)*T;        % Time vector
t=t';
f = fft(Itotal);
P2 = abs(f/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
freq = Fs*(0:(L/2))/L;

fouriertot = fft(z);
Lz = length(fouriertot);
P3 = abs(fouriertot/Lz); 
P4 = P3(1:Lz/2+1);
P4(2:end-1) = 2*P4(2:end-1);
freqZ = Fs*(0:(Lz/2))/Lz;

figure
subplot(2,1,1);
plot(time,Itotal);
title('Isignal');
subplot(2,1,2);
plot(freqZ,P4);
title('FFT')
xlabel('f (Hz)')
ylabel('a.u.')
%plot(freq,P1) 
% title('Single-Sided Amplitude Spectrum of I(t)')
% xlabel('f (Hz)')
% ylabel('|I(f)|')







