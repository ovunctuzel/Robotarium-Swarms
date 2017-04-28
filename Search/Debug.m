import graph.*
import controllers.*
import transformations.*
clc
close all;
% Init Routine ------------------------------------------------------------
% Number of agents
N = 8;
goals = 4;
% Init Robotarium
rb = RobotariumBuilder();
r = rb.set_number_of_agents(N).set_save_data(false).build();
flock = Flock(r, N);

% Number of iteration before calling quits
timeout = 1;
% Number of experiments
experiments = 1;
experimentSet = 'R2_M_C1';

data = Data(flock, timeout);
mkdir('Data')
% -------------------------------------------------------------------------

initPos_ar = zeros(N, 2, experiments);
goalPos_ar = zeros(goals, 2, experiments);
radii_ar = zeros(1, 3, experiments);
model_ar = cell(1, experiments);
nTop_ar = zeros(1, experiments);

% Parse Input File ----------------------------------------------------
for i=1:experiments
    fid=fopen(['../Input/Search/', experimentSet, '/SearchInput', num2str(i), '.csv'],'rt');
    input = textscan(fid, '%f %f %f %f %f %f %f %s %f %f','HeaderLines',1,'Delimiter',',');
    % cell2mat([input(:,1); input(:,2)]')
    %initPos_ar(:,:,i) = cell2mat([input(:,1); input(:,2)]');
    c = cell2mat([input(:,3) input(:,4)]);
    goalPos_ar(:,:,i) = c(1:goals,:);
    c = cell2mat([input(1,5) input(1,6) input(1,7)]);
    radii_ar(:,:,i) = c(1,:);
    c = input(1,8);
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
    flock.radiusRep = 0.001;
    flock.radiusOri = 0.25;
    flock.radiusAtr = 0.25;
    flock.wallRepRadius = 0.125;
    flock.senseGoalRadius = 0.15;
    flock.foundGoalRadius = 0.15;
    flock.blindspot = 10*pi/6;
    flock.numObstacles = 0;
    
    % Barrier Certificate Init--------------------------------------------
    position_int = create_si_position_controller('XVelocityGain', 1, 'YVelocityGain', 1);
    si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.1);
    % Maybe we can play around with the safety radius? (Consensus Ex: 0.009)
    uni_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', 0.01, 'ProjectionDistance', 0.05);
    si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.75, 'AngularVelocityLimit', pi);
    
    flock.barrierCert = si_barrier_certificate;
    flock.unibarrierCert = uni_barrier_certificate;
    flock.si2uni = si_to_uni_dyn;
    flock.positionCont = position_int;
    % --------------------------------------------------------------------
        
    %flock.senseObstacleRadius = 0.1;
    flock.communicationRadius = flock.radiusAtr;
    flock.model = 'V';
    flock.nTop = 0;
    % Updating Goal & InitPos
    flock.goals = goalPos_ar(:,:,i);
    flock.goalMarks = zeros(1,goals);
    flock.goalVisits = zeros(N,goals);

%     flock.initPos = rand(2,8)/2-0.25;
    flock.initPos = [0 0
                    0.1 0
                    0.2 0
                    0.3 0
                    0.4 0
                    0.1 0.1
                    0.2 0.1
                    0.3 0.1]';
    
    % -----------------------
    flock.goToStart();
    flock.updateLocations();
    flock.run();

end
r.call_at_scripts_end();

