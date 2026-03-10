classdef RTD_agent_2D < agent
    %% properties
    properties
        % agent state info
        heading_index = 3 ;
        
        % trajectory info
        desired_time
        desired_input
        desired_trajectory
        stopping_time = 1 ;
        
        % footprint info
        footprint = 1 ;
        
        % plotting info
        footprint_vertices
        arrow_vertices
        plot_footprint_color = [0 0 1] ;
        plot_footprint_edge_color = [0 0 1] ;
        plot_footprint_opacity = 0.2 ;
        plot_footprint_edge_opacity = 1 ;
        plot_arrow_color = [0 0 0.2] ;
        plot_arrow_opacity = 0.5 ;
        plot_trajectory_at_time_flag = true ;
    end
    
    %% methods
    methods
        %% constructor
        function A = RTD_agent_2D(varargin)
            A@agent('name','RTD Agent 2D',varargin{:}) ;
            
            % set up plot data
            A.plot_data.trajectory = [] ;
            A.plot_data.footprint = [] ;
            A.plot_data.arrow = [] ;
            
            % set up footprint and arrow for plot
            A.make_footprint_plot_data() ;
            A.make_arrow_plot_data() ;
            
            % reset time, states, and inputs, just in case
            A.reset() ;
        end
        
        %% reset
        function reset(A,state)
            if nargin < 2   % 判断是否传入了自定义初始状态state。
                state = zeros(A.n_states,1) ; 
            end
            
            % do the reset
            A.state = zeros(A.n_states,1) ;
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ;
            A.input_time = 0 ;
            
            % reset the state
            switch length(state)  % 传入不同维度的状态，然后按照不同维度进行赋值
                case A.n_states
                    A.state = state ;
                case 2
                    A.state(A.position_indices) = state ;
                case 3
                    A.state([A.position_indices,A.heading_index]) = state ;
                otherwise
                    error(['The provided state has an incorrect number of elements!',...
                        ' Please provide either a 2-by-1 position, a 3-by-1 position',...
                        ' and heading, or an n_states-by-1 full state vector.'])
            end
        end
        
        %% get agent into
        function agent_info = get_agent_info(A)
            % call superclass method
            agent_info = get_agent_info@agent(A) ;
            
            % additional fields
            agent_info.desired_time = A.desired_time ;
            agent_info.desired_input = A.desired_input ;
            agent_info.desired_trajectory = A.desired_trajectory ;
            agent_info.heading_index = A.heading_index ;
            agent_info.footprint = A.footprint ;
            agent_info.footprint_vertices = A.footprint_vertices ;
        end
        
        %% commit move data
        function commit_move_data(A,T_state,Z_state,T_used,U_used,Z_used)
            % commit the usual move data
            commit_move_data@agent(A,T_state,Z_state,T_used,U_used,Z_used) ;
            
            % also update the desired time/input/trajectory
            if ~isempty(A.desired_time)
                t_cur = A.desired_time(end-1) ;
            else
                t_cur = 0 ;
            end
            A.desired_time = [A.desired_time, T_used(2:end) + t_cur, nan(1,1)] ;
            
            % save the desired input
            n_U = size(U_used,1) ;
            A.desired_input = [A.desired_input, U_used(:,2:end), nan(n_U,1)] ;
            
            % save the desired trajectory
            if ~isempty(Z_used)
                n_Z = size(Z_used,1) ;
                A.desired_trajectory = [A.desired_trajectory, Z_used(:,2:end), nan(n_Z,1)] ;
            end
        end
        
        %% emergency stop
        % note, this ignores any previous trajectory the agent may have
        % been tracking
        function stop(A,t_stop)
            if nargin < 2
                t_stop = A.stopping_time ;
            end
            
            T_input = 0:0.1:t_stop ;
            N_t = length(T_input) ;
            
            pose =  A.state([A.position_indices,A.heading_index],end) ;
            stopped_state = [pose ; zeros(A.n_states-3,1)] ;
            
            Z_desired = repmat(stopped_state,1,N_t) ;
            
            U_input = zeros(A.n_inputs,N_t) ;
            
            A.move(t_stop,T_input,U_input,Z_desired) ;
        end
        
        %% utility
        function make_footprint_plot_data(A)
            switch length(A.footprint)
                case 1
                    A.footprint_vertices = make_circle(A.footprint) ;
                case 2
                    A.footprint_vertices = make_box(A.footprint) ;
            end
        end
        
        function make_arrow_plot_data(A)
            % make arrow for plot
            t_arrow = [0, 2*pi/3, 4*pi/3] ;
            
            switch length(A.footprint)
                case 1
                    % btw, this is 0.4x because the footprint is a radius
                    x_arrow = 0.4*A.footprint*cos(t_arrow) ;
                    y_arrow = 0.4*A.footprint*sin(t_arrow) ;
                case 2
                    x_arrow = 0.2*A.footprint(1)*cos(t_arrow) ;
                    y_arrow = 0.2*A.footprint(2)*sin(t_arrow) ;
            end
            A.arrow_vertices = [x_arrow ; y_arrow] ;
        end
        
        %% plotting
        function plot(A,~)
            A.plot_at_time(A.time(end)) ;   % 绘制车身矩形、箭头、已经行驶过的轨迹
            plot@agent(A,A.plot_footprint_color) ;
        end
        
        % function plot_at_time(A,t)   % 绘制车辆本身矩形和箭头，没有车轮；已经执行的轨迹
        %     % compute footprint for plot
        %     z_t = match_trajectories(t,A.time,A.state) ;  % 从时间序列A.time和状态序列A.state中，匹配时刻t对应的状态z_t
        %     p_t = z_t(A.position_indices) ;    % 提取位置信息（x,y坐标）
        %     h_t = z_t(A.heading_index) ;       % 提取航向角（朝向）
        %     R_t = rotation_matrix_2D(h_t) ;    % 生成2D旋转矩阵（用于将智能体的“本地坐标系轮廓”旋转到“全局坐标系”）
        %     fp_t = A.footprint_vertices(:,1:end-1) ;  % A.footprint_vertices是智能体的“本地轮廓顶点”（
        %     N_fp = size(fp_t,2) ;     % 顶点数量
        %     V_fp = R_t*fp_t + repmat(p_t,1,N_fp) ;  % 将本地轮廓旋转（R_t）并平移（p_t）到全局坐标系，得到实际轮廓V_fp
        % 
        %     % make arrow for plot
        %     V_arrow = R_t*A.arrow_vertices + repmat(p_t,1,3) ;  % 旋转（R_t）并平移（p_t）到全局坐标系，得到实际箭头V_arrow
        % 
        %     % plot
        %     if check_if_plot_is_available(A,'footprint')   % 检查轮廓绘图对象是否可用
        %         A.plot_data.footprint.Vertices = V_fp' ;
        %         A.plot_data.arrow.Vertices = V_arrow' ;
        %     else  % 新建对象
        %         % plot footprint   % 绘制轮廓（patch函数画多边形，设置填充色、边缘色、透明度）
        %         fp_data = patch(V_fp(1,:),V_fp(2,:),A.plot_footprint_color,...
        %             'EdgeColor',A.plot_footprint_edge_color,...
        %             'FaceAlpha',A.plot_footprint_opacity,...
        %             'EdgeAlpha',A.plot_footprint_edge_opacity) ;
        % 
        %         % plot arrow on footprint   % 绘制箭头（patch函数画多边形，与轮廓风格统一）
        %         arrow_data = patch(V_arrow(1,:),V_arrow(2,:),A.plot_arrow_color,...
        %             'EdgeColor',A.plot_arrow_color,...
        %             'FaceAlpha',A.plot_arrow_opacity,...
        %             'EdgeAlpha',A.plot_arrow_opacity) ;
        % 
        %         % save plot data   % 保存绘图对象，供后续复用
        %         A.plot_data.footprint = fp_data ;
        %         A.plot_data.arrow = arrow_data ;
        %     end
        % 
        %     if A.plot_trajectory_at_time_flag   % 若开启轨迹绘制开关
        %         % get the executed path up to the current time   % 提取截至时刻t的所有位置数据（已执行的路径）
        %         X = A.state(A.position_indices,:) ;  % 所有时刻的位置（x,y）
        %         T_log = A.time <= t ;   % 筛选出时间≤t的时刻（已执行部分）
        %         X = X(:,T_log) ;    % 已执行轨迹的位置坐标（2×M矩阵，M为已执行步数）
        % 
        %         % plot it   % 绘制已经执行的轨迹
        %         if check_if_plot_is_available(A,'trajectory')
        %             A.plot_data.trajectory.XData = X(1,:) ;
        %             A.plot_data.trajectory.YData = X(2,:) ;
        %         end
        %             traj_data = plot_path(X,'b-') ;
        %             A.plot_data.trajectory = traj_data ;
        %     end
        % end

        function plot_at_time(A,t)   % 绘制车辆本身矩形和箭头，没有车轮；已经执行的轨迹
            % compute footprint for plot
            z_t = match_trajectories(t,A.time,A.state) ;  % 从时间序列A.time和状态序列A.state中，匹配时刻t对应的状态z_t
            p_t = z_t(A.position_indices) ;    % 提取位置信息（x,y坐标）
            h_t = z_t(A.heading_index) ;       % 提取航向角（朝向）
            R_t = rotation_matrix_2D(h_t) ;    % 生成2D旋转矩阵（用于将智能体的“本地坐标系轮廓”旋转到“全局坐标系”）
            fp_t = A.footprint_vertices(:,1:end-1) ;  % A.footprint_vertices是智能体的“本地轮廓顶点”（
            N_fp = size(fp_t,2) ;     % 顶点数量
            V_fp = R_t*fp_t + repmat(p_t,1,N_fp) ;  % 将本地轮廓旋转（R_t）并平移（p_t）到全局坐标系，得到实际轮廓V_fp

            % make arrow for plot
            V_arrow = R_t*A.arrow_vertices + repmat(p_t,1,3) ;  % 旋转（R_t）并平移（p_t）到全局坐标系，得到实际箭头V_arrow

            % plot
            % ---- MOD: 只允许在 figure(9) 里画 footprint/arrow（左图保留蓝色，右图不再出现蓝色patch） ----
            fnum = get(gcf,'Number') ;
            allow_footprint_here = (fnum == 9) ;

            if allow_footprint_here
                if check_if_plot_is_available(A,'footprint')   % 检查轮廓绘图对象是否可用
                    A.plot_data.footprint.Vertices = V_fp' ;
                    A.plot_data.arrow.Vertices = V_arrow' ;
                else  % 新建对象
                    % plot footprint   % 绘制轮廓（patch函数画多边形，设置填充色、边缘色、透明度）
                    fp_data = patch(V_fp(1,:),V_fp(2,:),A.plot_footprint_color,...
                        'EdgeColor',A.plot_footprint_edge_color,...
                        'FaceAlpha',A.plot_footprint_opacity,...
                        'EdgeAlpha',A.plot_footprint_edge_opacity) ;

                    % plot arrow on footprint   % 绘制箭头（patch函数画多边形，与轮廓风格统一）
                    arrow_data = patch(V_arrow(1,:),V_arrow(2,:),A.plot_arrow_color,...
                        'EdgeColor',A.plot_arrow_color,...
                        'FaceAlpha',A.plot_arrow_opacity,...
                        'EdgeAlpha',A.plot_arrow_opacity) ;

                    % save plot data   % 保存绘图对象，供后续复用
                    A.plot_data.footprint = fp_data ;
                    A.plot_data.arrow = arrow_data ;
                end
            end

            if A.plot_trajectory_at_time_flag   % 若开启轨迹绘制开关
                % get the executed path up to the current time   % 提取截至时刻t的所有位置数据（已执行的路径）
                X = A.state(A.position_indices,:) ;  % 所有时刻的位置（x,y）
                T_log = A.time <= t ;   % 筛选出时间≤t的时刻（已执行部分）
                X = X(:,T_log) ;    % 已执行轨迹的位置坐标（2×M矩阵，M为已执行步数）

                % plot it   % 绘制已经执行的轨迹
                % if check_if_plot_is_available(A,'trajectory')
                %     A.plot_data.trajectory.XData = X(1,:) ;
                %     A.plot_data.trajectory.YData = X(2,:) ;
                % else
                %     traj_data = plot_path(X,'b-') ;
                %     A.plot_data.trajectory = traj_data ;
                % end

                % if check_if_plot_is_available(A,'trajectory')
                %     A.plot_data.trajectory.XData = X(1,:) ;
                %     A.plot_data.trajectory.YData = X(2,:) ;
                % end
                % traj_data = plot_path(X,'b-') ;
                % A.plot_data.trajectory = traj_data ;

                if check_if_plot_is_available(A,'trajectory') && isgraphics(A.plot_data.trajectory)
                    % 已经有线了：只更新数据（不会新建对象 -> 不闪）
                    A.plot_data.trajectory.XData = X(1,:) ;
                    A.plot_data.trajectory.YData = X(2,:) ;
                else
                    % 第一次：创建一次线对象（保持你原来的画法：蓝色实线）
                    traj_data = plot_path(X,'b-') ;
                    A.plot_data.trajectory = traj_data ;
                end
            end
        end
    end
end