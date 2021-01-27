function [conGUI] = a1_load_animation(robot, gait, terrain, varargin)

      
    ip = inputParser;
    ip.addParameter('UseExported',true,@(x) isequal(x,true) || isequal(x,false));
    ip.addParameter('ExportPath','',@(x)ischar(x));
    ip.addParameter('SkipExporting',false,@(x) isequal(x,true) || isequal(x,false));
    
    ip.parse(varargin{:});
    
    opts = ip.Results;
    if isempty(opts.ExportPath)
        root_path = get_root_path();
        export_path = fullfile(root_path,'gen','anim');
        opts.ExportPath = export_path;
    else
        export_path = opts.ExportPath;
    end
    if ~exist(export_path,'dir')
        mkdir(export_path);
    end
    addpath(export_path);
    
    cont_domain_idx = find(arrayfun(@(x)~isempty(x.tspan),gait));
    t = [];
    q = [];
    
    for j=cont_domain_idx.'
        t = [t,gait(j).tspan];         %#ok<*AGROW>
        q = [q,gait(j).states.x];
    end
    
    
    f = figure(1000); clf;
    a1_disp = frost.Animator.Display(f, robot, opts);
    
    % Add Pelvis Sphere
    pelvis = robot.Links(getLinkIndices(robot,'base'));
    torso = frost.Animator.LinkSphere(a1_disp.axs, robot, pelvis, 'torso', opts);
    torso.radius = 0.02;
    a1_disp.addItem(torso);

    % Add Right Toe
%     front_left_toe_anim_frame = CoordinateFrame(...
%          'Name','FrontLeftToeAnim',...
%          'Reference',robot.ContactPoints.front_left_toe,...
%          'Offset',[0,0,0.02],...
%          'R',[0,0,pi/2]);
%     top_offset = [0,0,0];
%     wf = 0.02; % toe width
%     lt = 0.02; % length to front
%     lh = 0.02; % length to back
%     bottom_offset = [lt,-wf/2,0;
%                      lt, wf/2,0;
%                     -lh, wf/2,0;
%                     -lh,-wf/2,0];
%     front_left_toe = frost.Animator.Pyramid(a1_disp.axs, robot, front_left_toe_anim_frame, top_offset, bottom_offset, 'FrontLeftToe', opts);
%     a1_disp.addItem(front_left_toe);
%     
%     front_right_toe_anim_frame = CoordinateFrame(...
%          'Name','FrontLeftToeAnim',...
%          'Reference',robot.ContactPoints.front_right_toe,...
%          'Offset',[0,0,0.02],...
%          'R',[0,0,pi/2]);
%     top_offset = [0,0,0];
%     wf = 0.02; % toe width
%     lt = 0.02; % length to front
%     lh = 0.02; % length to back
%     bottom_offset = [lt,-wf/2,0;
%                      lt, wf/2,0;
%                     -lh, wf/2,0;
%                     -lh,-wf/2,0];
%     front_right_toe = frost.Animator.Pyramid(a1_disp.axs, robot, front_right_toe_anim_frame, top_offset, bottom_offset, 'FrontRightToe', opts);
%     a1_disp.addItem(front_right_toe);
%     
%     rear_left_toe_anim_frame = CoordinateFrame(...
%          'Name','RearLeftToeAnim',...
%          'Reference',robot.ContactPoints.rear_left_toe,...
%          'Offset',[0,0,0.02],...
%          'R',[0,0,pi/2]);
%     top_offset = [0,0,0];
%     wf = 0.02; % toe width
%     lt = 0.02; % length to front
%     lh = 0.02; % length to back
%     bottom_offset = [lt,-wf/2,0;
%                      lt, wf/2,0;
%                     -lh, wf/2,0;
%                     -lh,-wf/2,0];
%     rear_left_toe = frost.Animator.Pyramid(a1_disp.axs, robot, rear_left_toe_anim_frame, top_offset, bottom_offset, 'RearLeftToe', opts);
%     a1_disp.addItem(rear_left_toe);
%     
%     rear_right_toe_anim_frame = CoordinateFrame(...
%          'Name','RearLeftToeAnim',...
%          'Reference',robot.ContactPoints.rear_right_toe,...
%          'Offset',[0,0,0.02],...
%          'R',[0,0,pi/2]);
%     top_offset = [0,0,0];
%     wf = 0.02; % toe width
%     lt = 0.02; % length to front
%     lh = 0.02; % length to back
%     bottom_offset = [lt,-wf/2,0;
%                      lt, wf/2,0;
%                     -lh, wf/2,0;
%                     -lh,-wf/2,0];
%     rear_right_toe = frost.Animator.Pyramid(a1_disp.axs, robot, rear_right_toe_anim_frame, top_offset, bottom_offset, 'RearRightToe', opts);
%     a1_disp.addItem(rear_right_toe);
    
  

    % Display
    if isempty(terrain)
        [terrain.Tx, terrain.Ty] = meshgrid(-10:1:10, -10:1:10);
        terrain.Tz = 0.*terrain.Tx;
    end
    anim = frost.Animator.AbstractAnimator(a1_disp, t, q, terrain);
    anim.isLooping = true;
    anim.speed = 1;
    anim.pov = frost.Animator.AnimatorPointOfView.Free;
    anim.Animate(true);
    conGUI = frost.Animator.AnimatorControls();
    conGUI.anim = anim;
    
end