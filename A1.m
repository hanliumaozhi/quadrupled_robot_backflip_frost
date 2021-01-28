classdef A1 < RobotLinks
    
    
    properties
        ContactPoints = struct;
        OtherPoints = struct;
        
    end
    
    methods
        
        function obj = A1(urdf)
            
            % Floating base model
            base(6) = struct(); % 6-DOF base coordinates
            
            % the name of the base dofs
            [base(1:6).Name] = deal('BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY','BaseRotX');
            
            % the type of the base dofs
            [base(1:3).Type] = deal('prismatic'); % the first three are prismatic joints
            [base(4:6).Type] = deal('revolute');  % the last three are revolute joints
            
            % the origin are all zeros
            [base.Offset] = deal([0,0,0]);
            [base.R] = deal([0,0,0]);
            
            % the axis of the base dofs
            [base(1:3).Axis] = deal([1,0,0],[0,1,0],[0,0,1]);
            [base(4:6).Axis] = deal([0,0,1],[0,1,0],[1,0,0]);
            
            % the parent link of the base dofs
            [base.Parent] =  deal('Origin', 'BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY');
            
            % the child link of the base dofs
            [base.Child] = deal('BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY','');
            
            % Set base DOF limits (Joint limits are read from URDF)
            [limits(1:6).lower] = deal(-10, -10, 0, -pi, -inf, deg2rad(-3));
            [limits(1:6).upper] = deal(10, 10, 2, pi, inf, deg2rad(3));
            [limits(1:6).velocity] = deal(20, 20, 20, 20, 20, 20);
            [limits.effort] = deal(0);
            for i=1:length(base)
                base(i).Limit = limits(i);
            end
            
            % load model from the URDF file
            config = struct();
            config_file = GetFullPath(urdf);
            [config.name, config.links, config.joints, config.transmissions] = ros_load_urdf(config_file);
            obj = obj@RobotLinks(config, base, [], 'removeFixedJoints', true);
            obj.ConfigFile = config_file;
            
            %% Define contact frames
            
                 
            % front_left_toe
            FL_calf_frame = obj.Joints(getJointIndices(obj, 'FL_calf_joint'));
            obj.ContactPoints.front_left_toe = CoordinateFrame(...
                'Name','FrontLeftToe',...
                'Reference', FL_calf_frame,...
                'Offset',[0, 0, -0.22],...
                'R',[0, 0, 0]...
                );
            
            % front_right_toe
            FR_calf_frame = obj.Joints(getJointIndices(obj, 'FR_calf_joint'));
            obj.ContactPoints.front_right_toe = CoordinateFrame(...
                'Name','FrontRightToe',...
                'Reference', FR_calf_frame,...
                'Offset',[0, 0, -0.22],...
                'R',[0, 0, 0]...
                );
            
            % Rear_left_toe
            RL_calf_frame = obj.Joints(getJointIndices(obj, 'RL_calf_joint'));
            obj.ContactPoints.rear_left_toe = CoordinateFrame(...
                'Name','RearLeftToe',...
                'Reference',RL_calf_frame,...
                'Offset',[0, 0, -0.22],...
                'R',[0, 0, 0]...
                );
            
            % Rear_right_toe
            RR_calf_frame = obj.Joints(getJointIndices(obj, 'RR_calf_joint'));
            obj.ContactPoints.rear_right_toe = CoordinateFrame(...
                'Name','RearRightToe',...
                'Reference', RR_calf_frame,...
                'Offset',[0, 0, -0.22],...
                'R',[0, 0, 0]...
                );
            
             %% Define other frames
             
             obj.OtherPoints.fl_leg = CoordinateFrame(...
                 'Name','FrontLeftToe',...
                 'Reference', FL_calf_frame,...
                 'Offset',[0, 0, 0],...
                 'R',[0, 0, 0]...
                 );
             
             obj.OtherPoints.fr_leg = CoordinateFrame(...
                 'Name','FrontRightToe',...
                 'Reference', FR_calf_frame,...
                 'Offset',[0, 0, 0],...
                 'R',[0, 0, 0]...
                 );
             
             obj.OtherPoints.rl_leg = CoordinateFrame(...
                 'Name','RearLeftToe',...
                 'Reference',RL_calf_frame,...
                 'Offset',[0, 0, 0],...
                 'R',[0, 0, 0]...
                 );
             
             obj.OtherPoints.rr_leg = CoordinateFrame(...
                 'Name','RearRightToe',...
                 'Reference', RR_calf_frame,...
                 'Offset',[0, 0, 0],...
                 'R',[0, 0, 0]...
                 );
             
                        
        end
        
        function ExportEnergy(obj, export_function, export_path)
            % Generates code for kinetic and potential energy
            
            % Kinetic Energy
            T = 0.5 * obj.States.dx' * obj.Mmat * obj.States.dx;
            export_function(T, 'KineticEnergy', export_path, {obj.States.x, obj.States.dx});
            
            % Potential Energy
            V = SymExpression();
            g = 9.81;
            for i = 1:length(obj.Links)
                p = obj.Links(i).computeCartesianPosition;
                m = obj.Links(i).Mass;
                V = V + m*g*p(3);
            end
            export_function(V, 'PotentialEnergy', export_path, {obj.States.x});
            
        end
        
        function ExportKinematics(obj, export_function, export_path)
            % Generates code for forward kinematics
            
            if ~exist(export_path,'dir')
                mkdir(export_path);
                addpath(export_path);
            end
            
            % Compute positions of all joints
            for i = 1:length(obj.Joints)
                % Create fixed frame on link after rotation of joint
                % (same origin as joint)
                frame = CoordinateFrame(...
                    'Name',obj.Joints(i).Name,...
                    'Reference',obj.Joints(i),...
                    'Offset',[0, 0, 0],...
                    'R',[0, 0, 0]);
                H = frame.computeForwardKinematics;
                p = H(1:3,end);
                J = jacobian(p, obj.States.x);
                R = H(1:3,1:3);
                export_function(p, ['p_', obj.Joints(i).Name], export_path, obj.States.x);
                export_function(J, ['Jp_', obj.Joints(i).Name], export_path, obj.States.x);
                export_function(H, ['H_', obj.Joints(i).Name], export_path, obj.States.x);
                export_function(R, ['R_', obj.Joints(i).Name], export_path, obj.States.x);
            end
            
            % Compute positions of contact points
            cp_fields = fields(obj.ContactPoints);
            for i = 1:length(cp_fields)
                p = obj.ContactPoints.(cp_fields{i}).computeCartesianPosition;
                J = jacobian(p, obj.States.x);
                H = obj.ContactPoints.(cp_fields{i}).computeForwardKinematics;
                R = H(1:3,1:3);
                export_function(p, ['p_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
                export_function(J, ['Jp_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
                export_function(H, ['H_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
                export_function(R, ['R_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
            end
            
            % Compute positions of other points
            op_fields = fields(obj.OtherPoints);
            for i = 1:length(op_fields)
                p = obj.OtherPoints.(op_fields{i}).computeCartesianPosition;
                J = jacobian(p, obj.States.x);
                H = obj.OtherPoints.(op_fields{i}).computeForwardKinematics;
                R = H(1:3,1:3);
                export_function(p, ['p_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
                export_function(J, ['Jp_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
                export_function(H, ['H_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
                export_function(R, ['R_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
            end
            
        end

        function ExportCoM(obj, export_function, export_path)
            % Generates code for forward kinematics
            
            if ~exist(export_path,'dir')
                mkdir(export_path);
                addpath(export_path);
            end
            
            p_com = obj.getComPosition();
            J_com = jacobian(p_com, obj.States.x);
            v_com = J_com*obj.States.dx;
            export_function(p_com, ['pcom_', obj.Name], export_path, obj.States.x);
            export_function(v_com, ['vcom_', obj.Name], export_path, {obj.States.x, obj.States.dx});
            
        end
        
        function ExportDynamics(obj, export_function, export_path)
            % Generates code for system dynamics
            
            if ~exist(export_path,'dir')
                mkdir(export_path);
                addpath(export_path);
            end
            
            n_link = length(obj.Links);
            links = getTwists(obj.Links);
            for i=1:n_link
                links{i}.Mass = obj.Links(i).Mass;
                links{i}.Inertia = obj.Links(i).Inertia;
            end
    
            De = eval_math_fun('InertiaMatrix',[links,{obj.numState}]);
            
            De_motor = zeros(obj.numState);
            for i=1:obj.numState
                if ~isempty(obj.Joints(i).Actuator)
                    actuator = obj.Joints(i).Actuator;
                    
                    if ~isempty(actuator.Inertia) && ~isempty(actuator.Ratio)
                        % reflected motor inertia: I*r^2
                        De_motor(i,i) = actuator.Inertia * actuator.Ratio^2;
                        
                    end
                end
                
            end
            
            De_full = De + De_motor;
            
            
            export_function(De_full, ['De_', obj.Name], export_path, obj.States.x);
            
            Ge = eval_math_fun('GravityVector',[links,{obj.States.x}]);
            export_function(Ge, ['Ge_', obj.Name], export_path, {obj.States.x});
            
            Ce = eval_math_fun('InertiaToCoriolis',{De_full,obj.States.x, obj.States.dx});
            export_function(Ce, ['Ce_', obj.Name], export_path, {obj.States.x, obj.States.dx});
            
            Be = SymExpression(obj.Gmap.Control.u);
            export_function(Be, ['Be_', obj.Name], export_path, {obj.States.x});
        end
        
        
        function ExportHolonomicConstraints(obj, export_function, export_path)
            % Generates code for holonomic constraints
            
            if ~exist(export_path,'dir')
                mkdir(export_path);
                addpath(export_path);
            end
            
            h_constr_names = fieldnames(obj.HolonomicConstraints);
            if ~isempty(h_constr_names)
                for i=1:length(h_constr_names)
                    constr_name = h_constr_names{i};
                    constr = obj.HolonomicConstraints.(constr_name);
                    export_function(SymExpression(constr.ConstrJac), constr.Jh_name, export_path, obj.States.x);
                    export_function(SymExpression(constr.ConstrJacDot), constr.dJh_name, export_path, {obj.States.x,obj.States.dx});
                end
                
            end
            
        end
        
        function [hg] = getMomentum(obj)
            pcom = obj.getComPosition();
            Ag = zeros(6,obj.numState);
            n_link = length(obj.Links);
            Jb = cell(obj.numState,1);
            R = cell(obj.numState,1);
            p = cell(obj.numState,1);
            I = cell(obj.numState,1);
            Xg = cell(obj.numState,1);
            A = cell(obj.numState,1);
            for i=1:n_link
                frame = obj.Links(i);
                Jb{i} = getBodyJacobian(obj,frame);
                gst = computeForwardKinematics(frame);
                R{i} = frame.RigidOrientation(gst);
                p{i} = frame.RigidPosition(gst);
                I{i} = blkdiag(frame.Mass*eye(3),frame.Inertia);
                Xg{i} = [ R{i}', -R{i}'*obj.skew(p{i}-pcom'); zeros(3), R{i}' ];
                A{i} =  Xg{i}'*I{i}*Jb{i};
                Ag = Ag + A{i};
                %     vb{i} = Jb{i}*dq;
                %     h{i} = I{i}*Jb{i}*dq;
                %         De = De + Jb{i}'*I{i}*Jb{i};
                %         KE = KE + 0.5 * dq'*Jb{i}'*I{i}*Jb{i}*dq;
                %     h_tot = h_tot + Xg{i}'*h{i};
            end
            hg = Ag*obj.States.dx;
        end
        
        function ExportMomentum(obj, export_function, export_path)
            % Generates code for forward kinematics
            
            if ~exist(export_path,'dir')
                mkdir(export_path);
                addpath(export_path);
            end
            
            pcom = obj.getComPosition();
            Ag = zeros(6,obj.numState);
            n_link = length(obj.Links);
            Jb = cell(obj.numState,1);
            R = cell(obj.numState,1);
            p = cell(obj.numState,1);
            I = cell(obj.numState,1);
            Xg = cell(obj.numState,1);
            A = cell(obj.numState,1);
            for i=1:n_link
                frame = obj.Links(i);
                Jb{i} = getBodyJacobian(obj,frame);
                gst = computeForwardKinematics(frame);
                R{i} = frame.RigidOrientation(gst);
                p{i} = frame.RigidPosition(gst);
                I{i} = blkdiag(frame.Mass*eye(3),frame.Inertia);
                Xg{i} = [ R{i}', -R{i}'*obj.skew(p{i}-pcom'); zeros(3), R{i}' ];
                A{i} =  Xg{i}'*I{i}*Jb{i};
                Ag = Ag + A{i};
                %     vb{i} = Jb{i}*dq;
                %     h{i} = I{i}*Jb{i}*dq;
                %         De = De + Jb{i}'*I{i}*Jb{i};
                %         KE = KE + 0.5 * dq'*Jb{i}'*I{i}*Jb{i}*dq;
                %     h_tot = h_tot + Xg{i}'*h{i};
            end
            hg = Ag*obj.States.dx;
            dAg = jacobian(Ag*obj.States.dx,obj.States.x);
           
            export_function(hg, ['hg_',obj.Name], export_path, {obj.States.x,obj.States.dx});
            export_function(Ag, ['Ag_',obj.Name], export_path, {obj.States.x});
            export_function(dAg, ['dAg_',obj.Name], export_path, {obj.States.x,obj.States.dx});
        end
        
        function [Ax] = skew(~, v)
            % Convert from vector to skew symmetric matrix
            Ax = [    0, -v(3),  v(2);
                v(3),     0, -v(1);
                -v(2),  v(1),     0];
        end
        
    end
    
    
end




