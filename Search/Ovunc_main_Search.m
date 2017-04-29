import graph.*
import controllers.*
import transformations.*
clc;
close all;
% Init Routine ------------------------------------------------------------
% Number of agents
N = AGENT_COUNT;
goals = TARGET_COUNT;
% Init Robotarium
rb = RobotariumBuilder();
r = rb.set_number_of_agents(N).set_save_data(false).build();
flock = Flock(r, N);

% Number of iteration before calling quits
timeout = TIMEOUT;
% Number of experiments
experiments = 10;
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
    cell2mat([input(:,1); input(:,2)]')
    initPos_ar(:,:,i) = cell2mat([input(:,1); input(:,2)]');
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
    radii = radii_ar(:,:,i);
    flock.radiusRep = radii(1);
    flock.radiusOri = radii(2);
    flock.radiusAtr = radii(3);
    flock.wallRepRadius = WALL_REP_RAD;
    flock.senseGoalRadius = SENSE_GOAL_RAD;
    flock.foundGoalRadius = FOUND_GOAL_RAD;
    flock.blindspot = BLINDSPOT;
    flock.vdist = VISUAL_DIST;
    flock.numObstacles = 0;
    
    % Barrier Certificate Init--------------------------------------------
    position_int = create_si_position_controller('XVelocityGain', 1, 'YVelocityGain', 1);
    si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.1);
    % Maybe we can play around with the safety radius? (Consensus Ex: 0.009)
    uni_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', flock.radiusRep, 'ProjectionDistance', 0.05);
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
    % Updating Goal & InitPos
    flock.goals = goalPos_ar(:,:,i);
    flock.goalMarks = zeros(1,goals);
    flock.goalVisits = zeros(N,goals);
    flock.initPos = initPos_ar(:,:,i)';
    % Draw goal
    plotGoals(flock);
    
    % -----------------------
    flock.goToStart();
    flock.updateLocations();
    data.resetData(timeout);
    t = 0;
    while t<=timeout
        clearPlot();
        plotGoals(flock);
        flock.run();
        flock.updateLocations();
        flock.updateGoals();
        t = t + 1;
        data.calc_all(t);
    end
    
    data.calc_all(t);
    cd Data
    if ~exist(['Data_',experimentSet], 'dir')
       mkdir(['Data_',experimentSet]);
    end
    cd(['Data_',experimentSet]);
    data.generateDataFile(['Rally', num2str(i)]);
    cd ../..
    
    flock.run();
    flock.resetBoids();
    clearPlot();
end
r.call_at_scripts_end();

function plotGoals(flock)
    % Plot goals
    global handles
    for i=1:length(flock.goals)
        if flock.goalMarks(i) == 1
            c = [0.8 0.8 0.8];
        else
            c = [1 0.8 0];
        end
        handle = plot(flock.goals(i, 1), flock.goals(i, 2), '.', 'Color', c, 'MarkerSize', 50);
        handles(i) = handle;
    end
end

function clearPlot()
    % Clear goals
    global handles
    for i=1:length(handles)
        delete(handles(i));
    end
end
