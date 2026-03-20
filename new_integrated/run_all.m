%% run_all.m
% =========================================================
% AE700 — Master Runner Script
% IIT Bombay | F-4 Phantom
%
% Runs all five project sections in order and writes a
% consolidated results summary to AE700_Results.txt.
%
% Usage: >> run_all
%        or run individual sections separately.
% =========================================================

clear; clc; close all;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║  AE700 - Guidance & Control of UAVs              ║\n');
fprintf('║  F-4 Phantom  |  IIT Bombay                      ║\n');
fprintf('╚══════════════════════════════════════════════════╝\n\n');

if ~exist('report_figures','dir'), mkdir('report_figures'); end

%% Open results file
fid=fopen('AE700_Results.txt','w');
log=@(varargin) log_tee(fid,varargin{:});
log('AE700 - F-4 Phantom Project Results\n');
log('IIT Bombay | %s\n\n', datestr(now));

%% ============================================================
t0=tic;
log('=== SECTION 1: Coordinate Frames & Animation ===\n');
fprintf('\n>>> Running Section 1...\n');
try
    Section1_Animation;
    log('  Status: COMPLETE\n\n');
catch e
    log('  Error: %s\n\n',e.message);
    fprintf('  Section 1 error: %s\n',e.message);
end

%% ============================================================
log('=== SECTION 2: Kinematics and Dynamics ===\n');
fprintf('\n>>> Running Section 2...\n');
try
    Section2_EOM;
    log('  Status: COMPLETE\n\n');
catch e
    log('  Error: %s\n\n',e.message);
    fprintf('  Section 2 error: %s\n',e.message);
end

%% ============================================================
log('=== SECTION 3: Forces and Moments ===\n');
fprintf('\n>>> Running Section 3...\n');
try
    Section3_Forces;
    log('  Status: COMPLETE\n\n');
catch e
    log('  Error: %s\n\n',e.message);
    fprintf('  Section 3 error: %s\n',e.message);
end

%% ============================================================
log('=== SECTION 4: Linear Design Models ===\n');
fprintf('\n>>> Running Section 4...\n');
try
    Section4_Linear;
    log('  Status: COMPLETE\n\n');
catch e
    log('  Error: %s\n\n',e.message);
    fprintf('  Section 4 error: %s\n',e.message);
end

%% ============================================================
log('=== SECTION 5: Autopilot Design ===\n');
fprintf('\n>>> Running Section 5...\n');
try
    Section5_Autopilot;
    log('  Status: COMPLETE\n\n');
catch e
    log('  Error: %s\n\n',e.message);
    fprintf('  Section 5 error: %s\n',e.message);
end

%% Summary
elapsed=toc(t0);
png_files=dir('report_figures/*.png');
mp4_files=dir('report_figures/*.mp4');

log('=== SUMMARY ===\n');
log('  Total runtime: %.0f s\n', elapsed);
log('  PNG figures:   %d files\n', length(png_files));
log('  MP4 videos:    %d files\n', length(mp4_files));
fclose(fid);

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║  All sections complete.                           ║\n');
fprintf('║  Results : AE700_Results.txt                     ║\n');
fprintf('║  Figures : report_figures/ (%d PNG, %d MP4)         ║\n', length(png_files),length(mp4_files));
fprintf('║  Elapsed : %.0f seconds                             ║\n', elapsed);
fprintf('╚══════════════════════════════════════════════════╝\n\n');

%% Helper
function log_tee(fid,varargin)
    s=sprintf(varargin{:});
    fprintf('%s',s);
    fprintf(fid,'%s',s);
end
