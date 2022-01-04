numPaths = 5;       % number of propagation paths
propSpeed = 1500;   % speed of every wave propagation 
channelDepth = 100; % depth of the channel
  
% Define the properties of the underwater environment, including the channel depth, 
% the number of propagation paths, the propagation speed, and the bottom loss.
isopath{1} = phased.IsoSpeedUnderwaterPaths(...
          'ChannelDepth',channelDepth,...
          'NumPathsSource','Property',...
          'NumPaths',numPaths,...
          'PropagationSpeed',propSpeed,...
          'BottomLoss',0.5,...
          'TwoWayPropagation',true);
 
isopath{2} = phased.IsoSpeedUnderwaterPaths(...
          'ChannelDepth',channelDepth,...
          'NumPathsSource','Property',...
          'NumPaths',numPaths,...
          'PropagationSpeed',propSpeed,...
          'BottomLoss',0.5,...
          'TwoWayPropagation',true);
      
%create a multipath channel for each target. The multipath channel propagates 
% the waveform along the multiple paths. This two-step process is analogous 
% to designing a filter and using the resulting coefficients to filter a signal.
      
fc = 20e3;   % Operating frequency (Hz)
 
channel{1} = phased.MultipathChannel(...
          'OperatingFrequency',fc);
 
channel{2} = phased.MultipathChannel(...
          'OperatingFrequency',fc);
 
% The first target is more distant but has a larger target strength, 
% and the second is closer but has a smaller target strength. 
% Both targets are isotropic and stationary with respect to the sonar system.
 
tgt{1} = phased.BackscatterSonarTarget(...
    'TSPattern',-5*ones(181,361));
 
tgt{2} = phased.BackscatterSonarTarget(...
    'TSPattern',-20*ones(181,361));
 
tgtplat{1} = phased.Platform(...
    'InitialPosition',[300; 500; -40],'Velocity',[0; 0; 0]); % position(x,y,z)
                                                              % range=sqrt(x^2+y^2)
tgtplat{2} = phased.Platform(...                              % depth=z
    'InitialPosition',[200; 200; -80],'Velocity',[0; 0; 0]);
 
% helperPlotPaths([0;0;-60],[300 200; 500 200; -40 -80], ...
% channelDepth,numPaths)
 
% specify a rectangular waveform to transmit to the targets. 
% The maximum target range and desired range resolution define the properties of the waveform.
 
maxRange = 5000;                         % Maximum unambiguous range
rangeRes = 10;                           % Required range resolution
prf = propSpeed/(2*maxRange);            % Pulse repetition frequency
pulse_width = 2*rangeRes/propSpeed;      % Pulse width
pulse_bw = 1/pulse_width;                % Pulse bandwidth
fs = 2*pulse_bw;                         % Sampling rate
wav = phased.RectangularWaveform(...
    'PulseWidth',pulse_width,...
    'PRF',prf,...
    'SampleRate',fs);
 
% Update the sample rate of the multipath channel with 
% the transmitted waveform sample rate
channel{1}.SampleRate = fs;
channel{2}.SampleRate = fs;
 
% The transmitter consists of a hemispherical array of back-baffled 
% isotropic projector elements. The transmitter is located 60 meters below the surface. 
% Create the array and view the array geometry.
 
plat = phased.Platform(...              % source coordinates
    'InitialPosition',[0; 0; -60],...
    'Velocity',[0; 0; 0]);
 
proj = phased.IsotropicProjector(...
    'FrequencyRange',[0 30e3],'VoltageResponse',80,'BackBaffled',true);
 
[ElementPosition,ElementNormal] = helperSphericalProjector(8,fc,propSpeed);
 
projArray = phased.ConformalArray(...
    'ElementPosition',ElementPosition,...
    'ElementNormal',ElementNormal,'Element',proj);
 
% viewArray(projArray,'ShowNormals',true);
 
% The receiver consists of a hydrophone and an amplifier. 
% The hydrophone is a single isotropic element and has a frequency range 0-30 kHz, 
% which contains the operating frequency of the multipath channel. 
% Specify the hydrophone voltage sensitivity as -140 dB.
 
hydro = phased.IsotropicHydrophone(...
    'FrequencyRange',[0 30e3],'VoltageSensitivity',-140);
 
% Thermal noise is present in the received signal. 
% Assume that the receiver has 20 dB of gain and a noise figure of 10 dB.
 
rx = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',10,...
    'SampleRate',fs,...
    'SeedSource','Property',...
    'Seed',2007);
 
% The radiator generates the spatial dependence of the propagated wave due to the array geometry. 
% The collector combines the backscattered signals received by the hydrophone element 
% from the far-field target.
radiator = phased.Radiator('Sensor',projArray,'OperatingFrequency',...
 fc,'PropagationSpeed',propSpeed);
 
collector = phased.Collector('Sensor',hydro,'OperatingFrequency',fc,...
  'PropagationSpeed',propSpeed);
 
% transmit the rectangular waveform over ten repetition intervals and 
% simulate the signal received at the hydrophone for each transmission.
x = wav();    % Generate pulse
xmits = 10;
rx_pulses = zeros(size(x,1),xmits);
t = (0:size(x,1)-1)/fs;
 
for j = 1:xmits
 
    % Update target and sonar position
    [sonar_pos,sonar_vel] = plat(1/prf);
 
    for i = 1:2 %Loop over targets
       [tgt_pos,tgt_vel] = tgtplat{i}(1/prf);
 
      % Compute transmission paths using the method of images. Paths are
      % updated according to the CoherenceTime property.
      [paths,dop,aloss,tgtAng,srcAng] = isopath{i}(...
            sonar_pos,tgt_pos,...
            sonar_vel,tgt_vel,1/prf);
 
      % Compute the radiated signals. Steer the array towards the target.
      tsig = radiator(x,srcAng);
 
      % Propagate radiated signals through the channel.
      tsig = channel{i}(tsig,paths,dop,aloss);
 
      % Target
      tsig = tgt{i}(tsig,tgtAng);
 
      % Collector
      rsig = collector(tsig,srcAng);
      rx_pulses(:,j) = rx_pulses(:,j) + ...
               rx(rsig);
    end
end
 
% Define the magnitude of non-coherent integration of the received signals 
% to locate the returns of the two targets.
rx_pulses = pulsint(rx_pulses,'noncoherent');
 
 
% Low pass butterworth filter design
wp = 0.05*pi; % Digital passband freq in rad
ws = 0.4*pi; % Digital stopband freq in rad
Rp = 1; % Passband ripple in dB
As = 15; % Stopband attenuation in dB
% Analog Prototype Specifications:
T = 1; % Set T=1
OmegaP = (2/T)*tan(wp/2); % Prewarp prototype passband freq
OmegaS = (2/T)*tan(ws/2); % Prewarp prototype stopband freq
% Analog Prototype Order Calculation:
N =ceil((log10((10^(Rp/10)-1)/(10^(As/10)-1)))/(2*log10(OmegaP/OmegaS)));
fprintf('\n*** Butterworth Filter Order = %2.0f \n',N)
 
OmegaC = OmegaP/((10^(Rp/10)-1)^(1/(2*N))); % Analog BF prototype cutoff
wn = 2*atan((OmegaC*T)/2); % Digital BF cutoff freq
% Digital butterworth filter design:
wn = wn/pi; % Digital BF cutoff in pi units
[b,a]=butter(N,wn);
[b0,B,A] = dir2cas(b,a);
data=abs(rx_pulses);
filtered_data=filtfilt(b,a,data);
 
% plot the result
subplot(211);
plot(t,data)
grid on
xlabel('Time (s)')
ylabel('Amplitude (V)')
title('Integrated Received Pulses')
 
subplot(212);
plot(t,filtered_data)
grid on
xlabel('Time (s)')
ylabel('Amplitude (V)')
title('Filtered Integrated Received Pulses')
 

