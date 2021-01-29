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
    
    t = [t,gait(1).tspan];         %#ok<*AGROW>
    q = [q,gait(1).states.x];
    t = [t,gait(3).tspan+0.3];         %#ok<*AGROW>
    q = [q,gait(3).states.x];
    
    
    f = figure(1000); clf;
    a1_disp = frost.Animator.Display(f, robot, opts);
    
    % Add Pelvis Sphere
    pelvis = robot.Links(getLinkIndices(robot,'base'));
    torso = frost.Animator.LinkSphere(a1_disp.axs, robot, pelvis, 'torso', opts);
    torso.radius = 0.02;
    a1_disp.addItem(torso);
    
    % add front left calf
    front_left_leg_anim_frame = CoordinateFrame(...
                                'Name','FrontLeftLegAnim',...
                                'Reference',robot.OtherPoints.fl_leg,...
                                'Offset',[0,0,0],...
                                'R',[0,0,0]);
                            
    front_left_leg = frost.Animator.Cylinder(a1_disp.axs, robot, front_left_leg_anim_frame, [0,0,-0.2], 'FrontLeftLeg', opts);
    a1_disp.addItem(front_left_leg);
    
    front_left_leg_anim_frame = CoordinateFrame(...
                                'Name','FrontLeftLegAnimC',...
                                'Reference',robot.OtherPoints.fl_leg,...
                                'Offset',[0,0,-0.2],...
                                'R',[0,0,0]);
                            
    front_left_leg_c = frost.Animator.Sphere(a1_disp.axs, robot, front_left_leg_anim_frame, 'FrontLeftLegC', opts);
    a1_disp.addItem(front_left_leg_c);
    
    % add rear right calf
    rear_right_leg_anim_frame = CoordinateFrame(...
                                'Name','RearRightLegAnim',...
                                'Reference',robot.OtherPoints.rr_leg,...
                                'Offset',[0,0,0],...
                                'R',[0,0,0]);
                            
    rear_right_leg = frost.Animator.Cylinder(a1_disp.axs, robot, rear_right_leg_anim_frame, [0,0,-0.2], 'RearRightLeg', opts);
    a1_disp.addItem(rear_right_leg);
    
    rear_right_leg_anim_frame = CoordinateFrame(...
                                'Name','ReartRightLegAnimC',...
                                'Reference',robot.OtherPoints.rr_leg,...
                                'Offset',[0,0,-0.2],...
                                'R',[0,0,0]);
                            
    rear_right_leg_c = frost.Animator.Sphere(a1_disp.axs, robot, rear_right_leg_anim_frame, 'RearRightLegC', opts);
    a1_disp.addItem(rear_right_leg_c);
    
    % add rear left calf
    rear_left_leg_anim_frame = CoordinateFrame(...
                                'Name','RearLeftLegAnim',...
                                'Reference',robot.OtherPoints.rl_leg,...
                                'Offset',[0,0,0],...
                                'R',[0,0,0]);
                            
    rear_left_leg = frost.Animator.Cylinder(a1_disp.axs, robot, rear_left_leg_anim_frame, [0,0,-0.2], 'RearLeftLeg', opts);
    a1_disp.addItem(rear_left_leg);
    
    rear_left_leg_anim_frame = CoordinateFrame(...
                                'Name','RearLeftLegAnimC',...
                                'Reference',robot.OtherPoints.rl_leg,...
                                'Offset',[0,0,-0.2],...
                                'R',[0,0,0]);
                            
    rear_left_leg_c = frost.Animator.Sphere(a1_disp.axs, robot, rear_left_leg_anim_frame, 'RearLeftLegC', opts);
    a1_disp.addItem(rear_left_leg_c);
    
    % add front right calf
    front_right_leg_anim_frame = CoordinateFrame(...
                                'Name','FrontRightLegAnim',...
                                'Reference',robot.OtherPoints.fr_leg,...
                                'Offset',[0,0,0],...
                                'R',[0,0,0]);
                            
    front_right_leg = frost.Animator.Cylinder(a1_disp.axs, robot, front_right_leg_anim_frame, [0,0,-0.2], 'FrontRightLeg', opts);
    a1_disp.addItem(front_right_leg);
    
    front_right_leg_anim_frame = CoordinateFrame(...
                                'Name','FrontRightLegAnimC',...
                                'Reference',robot.OtherPoints.fr_leg,...
                                'Offset',[0,0,-0.2],...
                                'R',[0,0,0]);
                            
    front_right_leg_c = frost.Animator.Sphere(a1_disp.axs, robot, front_right_leg_anim_frame, 'FrontRightLegC', opts);
    a1_disp.addItem(front_right_leg_c);
    

  

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