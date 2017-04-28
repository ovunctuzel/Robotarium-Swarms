import graph.*
import controllers.*
import transformations.*

clc
close all
% clear all
% Radii configurations (repulsion-orientation-attraction)
% Robot is roughly 0.03m
% model is either 'M', 'T' or 'V'. 'T3': topological with 2 neighbors
% goalPos is 1x2 vector of the goal position
% initPos is Nx2 matrix of the goal position
% informed is the percentage of informed agents
% radii is a 1x3 vector of rRep, rOri, rAtt

% Init Routine ------------------------------------------------------------
% Number of agents
% N = 8;
% Init Robotarium
rb = RobotariumBuilder();
r = rb.set_number_of_agents(N).set_save_data(false).build();
flock = Flock(r, N);

% Number of iteration before calling quits
timeout = 900;
% Number of experiments
% experiments = 10;
% experimentSet = 'A1_M_N8';

data = Data(flock, timeout);
% -------------------------------------------------------------------------

radii_ar = zeros(1, 3, experiments);
model_ar = cell(1, experiments);
nTop_ar = zeros(1, experiments);

% Parse Input File ----------------------------------------------------
for i=1:experiments
    fid=fopen(['../Input/Avoid/', experimentSet, '/AvoidInput', num2str(i), '.csv'],'rt');
    input = textscan(fid, '%f %f %f %s %f','HeaderLines',1,'Delimiter',',');
    c = cell2mat([input(1,1) input(1,2) input(1,3)]);
    radii_ar(:,:,i) = c(1,:);
    c = input(1,4);
    c = c{1}{1};
    model_ar{1,i} = c(1);
    if char(model_ar(1,i)) == 'T'
        nTop_ar(1,i) = str2num(c(2));
    else
        nTop_ar(1,i) = 0;
    end
    fclose(fid);
end
% ---------------------------------------------------------------------

for i=1:experiments
    radii = radii_ar(:,:,i);
    flock.radiusRep = radii(1);
    flock.radiusOri = radii(2);
    flock.radiusAtr = radii(3);
    flock.wallRepRadius = 0.125;
    flock.blindspot = pi/3;
    flock.numObstacles = 0;
    flock.obstaclePos = [0.4; 0];

    % Barrier Certificate Init--------------------------------------------
    position_int = create_si_position_controller('XVelocityGain', 1, 'YVelocityGain', 1);
    si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.1);
    % (Consensus Ex: 0.009)
    uni_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', radii(1), 'ProjectionDistance', 0.05);
    si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.75, 'AngularVelocityLimit', pi);
    
    flock.barrierCert = si_barrier_certificate;
    flock.unibarrierCert = uni_barrier_certificate;
    flock.si2uni = si_to_uni_dyn;
    flock.positionCont = position_int;
    % --------------------------------------------------------------------
     
    %flock.senseObstacleRadius = 0.1;
    flock.communicationRadius = flock.radiusAtr;
    flock.model = model_ar{i};
    flock.nTop = nTop_ar(i);
    %Fixed start for avoid task
    flock.initPos = repmat([-0.3; 0],1,N);
    
    % -----------------------
    flock.goToStart();
    flock.initialRotations(zeros(1,N));
    flock.updateLocations();
    data.resetData(timeout);
    t = 0;
    while t<=timeout % Run until timeout (~flock.check_goal() &&) 
        flock.run();
        flock.updateLocations();
        t = t + 1
        data.calc_all(t);
    end
    data.calc_all(t);
%     
    cd Data
    if ~exist(['Data_',experimentSet], 'dir')
        mkdir(['Data_',experimentSet]);
    end
    cd(['Data_',experimentSet]);
    data.generateDataFile(['Avoid', num2str(i)]);
    cd ../..
    
    flock.run();
    flock.resetBoids();
end
r.call_at_scripts_end();


